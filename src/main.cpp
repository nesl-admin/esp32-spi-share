// JSON over SPI between two Olimex ESP32-POE-ISO boards, wired through UEXT1.
//
//   -DROLE_SERVER : SPI master (ESP-IDF spi_master + DMA). Sends the hardcoded
//                   meter snapshot JSON (include/meter_snapshot.h) back to back,
//                   paced only by the client's handshake line.
//   -DROLE_CLIENT : SPI slave (ESP-IDF spi_slave + DMA). Queues a receive
//                   buffer, raises the handshake line, and verifies the JSON.
//
// Frames are variable length: the server clocks only header + payload, rounded
// up to 4 bytes. Both sides size their buffers for MAX_FRAME_SIZE, so the JSON
// can grow up to MAX_FRAME_SIZE - 12 bytes without code changes.
//
// SPI is full duplex, so every transaction also carries a header-only ack frame
// from the client back to the server on MISO. That ack is the one the client
// prepared *before* the transaction, so it acknowledges the previous frame.
//
// Both roles print one summary line every REPORT_EVERY transactions. Build with
// -DVERBOSE_LOG=1 (and a slow SEND_INTERVAL_US) for the per-transaction CS /
// hexdump / JSON trace.
//
// See README.md for wiring, control flow and caveats.

#include <Arduino.h>

#include "meter_snapshot.h"

#if defined(ROLE_SERVER) == defined(ROLE_CLIENT)
#error "Define exactly one of ROLE_SERVER or ROLE_CLIENT (use: pio run -e server / -e client)"
#endif

// ---------------------------------------------------------------------------
// Configuration (override with -D build flags)
// ---------------------------------------------------------------------------

// UEXT1 pin -> ESP32 GPIO on the ESP32-POE-ISO
#ifndef PIN_MISO
#define PIN_MISO 15  // UEXT pin 7
#endif
#ifndef PIN_MOSI
#define PIN_MOSI 2   // UEXT pin 8
#endif
#ifndef PIN_SCK
#define PIN_SCK 14   // UEXT pin 9
#endif
#ifndef PIN_CS
#define PIN_CS 5     // UEXT pin 10
#endif
#ifndef PIN_HANDSHAKE
#define PIN_HANDSHAKE 4  // UEXT pin 3 (TXD1), client output -> server input
#endif

#ifndef SPI_CLOCK_HZ
#define SPI_CLOCK_HZ 8000000  // master rounds down to 80 MHz / n
#endif
#ifndef SPI_INPUT_DELAY_NS
#define SPI_INPUT_DELAY_NS 75  // server: ESP32 slave's MISO output delay via the GPIO matrix
#endif
#ifndef SPI_MODE_NUM
#define SPI_MODE_NUM 1
#endif
#ifndef MAX_FRAME_SIZE
#define MAX_FRAME_SIZE 16384  // largest frame either side can handle; multiple of 4
#endif
#ifndef USE_HANDSHAKE
#define USE_HANDSHAKE 1
#endif
#ifndef SEND_INTERVAL_US
#define SEND_INTERVAL_US 0  // 0 = back to back, as fast as the client can keep up
#endif
#ifndef HANDSHAKE_TIMEOUT_US
#define HANDSHAKE_TIMEOUT_US 10000
#endif
#ifndef REPORT_EVERY
#define REPORT_EVERY 100  // transactions per summary line
#endif
#ifndef VERBOSE_LOG
#define VERBOSE_LOG 0  // 1 = per-transaction trace (use SEND_INTERVAL_US=1000000)
#endif
#ifndef HEXDUMP_BYTES
#define HEXDUMP_BYTES 32  // verbose only: bytes of each raw frame to print; 0 disables
#endif

static_assert(MAX_FRAME_SIZE % 4 == 0, "MAX_FRAME_SIZE must be a multiple of 4 (ESP32 SPI DMA requirement)");

// Per-transaction trace; compiled in always so it can't rot, printed only when VERBOSE_LOG=1.
#define VLOG(...)                                 \
  do {                                            \
    if (VERBOSE_LOG) Serial.printf(__VA_ARGS__); \
  } while (0)

static const size_t SNAPSHOT_LEN = sizeof(METER_SNAPSHOT_JSON) - 1;

// ---------------------------------------------------------------------------
// Frame format (shared by both roles)
//
//   offset size field
//   0      2    magic     0xA5 0x5A
//   2      1    type      'D' = data from server, 'A' = ack from client
//   3      1    reserved  0
//   4      2    seq       server: frame counter; client: last seq received OK
//   6      2    len       payload length in bytes (0 for an ack)
//   8      2    crc       CRC-16/CCITT-FALSE over the payload
//   10     2    reserved  0
//   12     len  payload   JSON text (not NUL terminated)
//   ...         padding   zeros up to the next multiple of 4 bytes
// ---------------------------------------------------------------------------

struct __attribute__((packed)) FrameHeader {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t reserved0;
  uint16_t seq;
  uint16_t len;
  uint16_t crc;
  uint16_t reserved1;
};

static constexpr uint8_t MAGIC0 = 0xA5;
static constexpr uint8_t MAGIC1 = 0x5A;
static constexpr uint8_t TYPE_DATA = 'D';
static constexpr uint8_t TYPE_ACK = 'A';
static constexpr size_t MAX_PAYLOAD = MAX_FRAME_SIZE - sizeof(FrameHeader);

static_assert(MAX_PAYLOAD <= 0xFFFF, "len field is 16 bits");
static_assert(sizeof(METER_SNAPSHOT_JSON) - 1 <= MAX_PAYLOAD, "meter snapshot does not fit in MAX_FRAME_SIZE");

// Bytes actually clocked for a payload of this length.
static constexpr size_t wireLen(size_t payloadLen) { return (sizeof(FrameHeader) + payloadLen + 3) & ~(size_t)3; }

// Table-driven CRC: the bitwise version costs ~1 ms per 5 KB frame at 240 MHz.
struct Crc16Table {
  uint16_t t[256];
  constexpr Crc16Table() : t() {
    for (int i = 0; i < 256; i++) {
      uint16_t c = i << 8;
      for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
      t[i] = c;
    }
  }
};
static constexpr Crc16Table CRC_TABLE;

static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) crc = (crc << 8) ^ CRC_TABLE.t[((crc >> 8) ^ *data++) & 0xFF];
  return crc;
}

// Writes header + payload + zero padding into buf. Returns the number of bytes to clock.
static size_t buildFrame(uint8_t *buf, uint8_t type, uint16_t seq, const char *payload, size_t len) {
  if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
  size_t n = wireLen(len);
  FrameHeader h = {};
  h.magic0 = MAGIC0;
  h.magic1 = MAGIC1;
  h.type = type;
  h.seq = seq;
  h.len = len;
  h.crc = crc16((const uint8_t *)payload, len);
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), payload, len);
  memset(buf + sizeof(h) + len, 0, n - sizeof(h) - len);
  return n;
}

static void setFrameSeq(uint8_t *buf, uint16_t seq) { memcpy(buf + offsetof(FrameHeader, seq), &seq, sizeof(seq)); }

// Returns nullptr on success, otherwise a description of what is wrong.
static const char *parseFrame(const uint8_t *buf, size_t rxBytes, FrameHeader &h) {
  if (rxBytes < sizeof(FrameHeader)) return "short transfer (less than a header)";
  memcpy(&h, buf, sizeof(h));
  if (h.magic0 != MAGIC0 || h.magic1 != MAGIC1) return "bad magic (wiring, SPI mode, or peer not ready?)";
  if (h.len > MAX_PAYLOAD) return "length field larger than MAX_FRAME_SIZE";
  if (sizeof(FrameHeader) + h.len > rxBytes) return "payload truncated";
  if (crc16(buf + sizeof(h), h.len) != h.crc) return "CRC mismatch (bit errors on the wire?)";
  return nullptr;
}

// True if buf holds a valid frame of the given type whose payload is exactly payload[0..len).
static bool frameMatches(const uint8_t *buf, size_t rxBytes, uint8_t type, const char *payload, size_t len,
                         FrameHeader &h) {
  if (parseFrame(buf, rxBytes, h)) return false;
  return h.type == type && h.len == len && memcmp(buf + sizeof(h), payload, len) == 0;
}

static void hexdump(const char *tag, const char *label, const uint8_t *buf, size_t len) {
#if HEXDUMP_BYTES > 0
  if (len > HEXDUMP_BYTES) len = HEXDUMP_BYTES;
  Serial.printf("[%s] %s (first %u bytes):", tag, label, (unsigned)len);
  for (size_t i = 0; i < len; i++) {
    if (i % 16 == 0) Serial.printf("\n[%s]   %04x:", tag, (unsigned)i);
    Serial.printf(" %02x", buf[i]);
  }
  Serial.println();
#endif
}

static void printFrame(const char *tag, const char *dir, const uint8_t *buf, size_t rxBytes) {
  FrameHeader h;
  const char *err = parseFrame(buf, rxBytes, h);
  if (err) {
    Serial.printf("[%s] %s frame INVALID: %s\n", tag, dir, err);
    hexdump(tag, "raw", buf, rxBytes);
    return;
  }
  Serial.printf("[%s] %s frame OK: type='%c' seq=%u len=%u crc=0x%04x\n", tag, dir, h.type, h.seq, h.len,
                h.crc);
  if (h.len) Serial.printf("[%s] %s JSON: %.*s\n", tag, dir, (int)h.len, (const char *)(buf + sizeof(h)));
}

// Summary accumulator: one line per REPORT_EVERY transactions.
//   all good   -> "[TAG] #1200 OK  190 tx/s  969 kB/s"
//   otherwise  -> "[TAG] #1300 OK:97 FAIL:3  188 tx/s  941 kB/s"
// Rates count JSON payload bytes of successful transactions only.
static uint32_t txTotal = 0;
static uint32_t blockOk = 0;
static uint32_t blockFail = 0;
static uint64_t blockBytes = 0;
static uint32_t blockStartUs = 0;

static void tally(const char *tag, bool ok, size_t payloadBytes) {
  if (ok) {
    blockOk++;
    blockBytes += payloadBytes;
  } else {
    blockFail++;
  }
  txTotal++;
  if (txTotal % REPORT_EVERY) return;

  uint32_t now = micros();
  uint32_t elapsed = now - blockStartUs;
  if (elapsed == 0) elapsed = 1;
  unsigned long txps = (unsigned long)((uint64_t)REPORT_EVERY * 1000000 / elapsed);
  unsigned long kBps = (unsigned long)(blockBytes * 1000000 / elapsed / 1000);
  if (blockFail == 0)
    Serial.printf("[%s] #%lu OK  %lu tx/s  %lu kB/s\n", tag, (unsigned long)txTotal, txps, kBps);
  else
    Serial.printf("[%s] #%lu OK:%lu FAIL:%lu  %lu tx/s  %lu kB/s\n", tag, (unsigned long)txTotal,
                  (unsigned long)blockOk, (unsigned long)blockFail, txps, kBps);
  blockOk = 0;
  blockFail = 0;
  blockBytes = 0;
  blockStartUs = micros();  // exclude the print itself from the next block
}

// DMA-capable buffers, allocated in setup()
static uint8_t *txBuf;
static uint8_t *rxBuf;

static void allocBuffers() {
  txBuf = (uint8_t *)heap_caps_calloc(1, MAX_FRAME_SIZE, MALLOC_CAP_DMA);
  rxBuf = (uint8_t *)heap_caps_calloc(1, MAX_FRAME_SIZE, MALLOC_CAP_DMA);
  if (!txBuf || !rxBuf) {
    Serial.println("FATAL: could not allocate DMA buffers");
    while (true) delay(1000);
  }
}

static void printBanner(const char *role) {
  Serial.printf("\n=== ESP32-POE-ISO SPI JSON link: %s ===\n", role);
  Serial.printf("pins: MISO=GPIO%d(UEXT7) MOSI=GPIO%d(UEXT8) SCK=GPIO%d(UEXT9) CS=GPIO%d(UEXT10)", PIN_MISO,
                PIN_MOSI, PIN_SCK, PIN_CS);
#if USE_HANDSHAKE
  Serial.printf(" HS=GPIO%d(UEXT3)", PIN_HANDSHAKE);
#endif
  Serial.printf("\nmode=%d max frame=%u bytes, snapshot JSON=%u bytes (%u on the wire), report every %u\n",
                SPI_MODE_NUM, (unsigned)MAX_FRAME_SIZE, (unsigned)SNAPSHOT_LEN, (unsigned)wireLen(SNAPSHOT_LEN),
                (unsigned)REPORT_EVERY);
}

// ===========================================================================
#if defined(ROLE_SERVER)
// ===========================================================================

#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "SERVER";
static constexpr spi_host_device_t HOST = SPI2_HOST;  // HSPI, routed to the UEXT pins via the GPIO matrix
static spi_device_handle_t dev;
static uint16_t seq = 0;
static size_t txLen = 0;  // bytes clocked per transaction

void setup() {
  Serial.begin(115200);
  delay(200);
  printBanner("SERVER (SPI master)");
  allocBuffers();

  // The payload never changes, so build the frame once and only update seq per transaction.
  txLen = buildFrame(txBuf, TYPE_DATA, 0, METER_SNAPSHOT_JSON, SNAPSHOT_LEN);

  // CS is driven manually so every assert/deassert can be logged.
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
#if USE_HANDSHAKE
  pinMode(PIN_HANDSHAKE, INPUT_PULLDOWN);  // reads LOW if the client is absent
#endif

  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = MAX_FRAME_SIZE;  // default is only 4092 with DMA
  esp_err_t err = spi_bus_initialize(HOST, &bus, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    Serial.printf("FATAL: spi_bus_initialize failed: %s\n", esp_err_to_name(err));
    while (true) delay(1000);
  }

  spi_device_interface_config_t dc = {};
  dc.mode = SPI_MODE_NUM;
  dc.clock_speed_hz = SPI_CLOCK_HZ;
  dc.input_delay_ns = SPI_INPUT_DELAY_NS;
  dc.spics_io_num = -1;  // manual CS
  dc.queue_size = 1;
  // Skip the driver's full-duplex frequency check. MISO only carries the small ack,
  // which is CRC-checked, so a MISO timing failure shows up as FAIL in the summary.
  dc.flags = SPI_DEVICE_NO_DUMMY;
  err = spi_bus_add_device(HOST, &dc, &dev);
  if (err != ESP_OK) {
    Serial.printf("FATAL: spi_bus_add_device failed: %s\n", esp_err_to_name(err));
    while (true) delay(1000);
  }
  spi_device_acquire_bus(dev, portMAX_DELAY);  // sole user of the bus; skips per-transaction locking

  int actualKHz = 0;
  spi_device_get_actual_freq(dev, &actualKHz);
  int limitHz = spi_get_freq_limit(true, SPI_INPUT_DELAY_NS);
  Serial.printf("clock: requested %u Hz, actual %d kHz; full-duplex MISO limit for %d ns input delay: %d kHz\n",
                (unsigned)SPI_CLOCK_HZ, actualKHz, SPI_INPUT_DELAY_NS, limitHz / 1000);
  if (actualKHz * 1000 > limitHz)
    Serial.println("WARNING: clock above the MISO limit; acks may fail CRC (MOSI data is unaffected)");
  Serial.printf("interval=%u us (0 = back to back)\n", (unsigned)SEND_INTERVAL_US);

  blockStartUs = micros();
}

// One full transaction. Returns true if the client's ack frame was valid.
static bool transact() {
  seq++;
  setFrameSeq(txBuf, seq);
  memset(rxBuf, 0, sizeof(FrameHeader));

  VLOG("\n[%s] ---- transaction seq=%u ----\n", TAG, seq);
  VLOG("[%s] TX JSON (%u bytes, %u on the wire)\n", TAG, (unsigned)SNAPSHOT_LEN, (unsigned)txLen);

#if USE_HANDSHAKE
  VLOG("[%s] waiting for handshake (GPIO%d HIGH = client has a buffer queued)...\n", TAG, PIN_HANDSHAKE);
  uint32_t waitStart = micros();
  while (gpio_get_level((gpio_num_t)PIN_HANDSHAKE) == 0) {
    if (micros() - waitStart > HANDSHAKE_TIMEOUT_US) {
      VLOG("[%s] handshake TIMEOUT after %u us: client not ready/connected, skipping seq=%u\n", TAG,
           (unsigned)HANDSHAKE_TIMEOUT_US, seq);
      return false;
    }
  }
  VLOG("[%s] handshake HIGH after %lu us\n", TAG, (unsigned long)(micros() - waitStart));
#endif

  spi_transaction_t t = {};
  t.length = txLen * 8;  // bits; full duplex, so rxBuf receives the same number
  t.tx_buffer = txBuf;
  t.rx_buffer = rxBuf;

  VLOG("[%s] CS -> LOW  (GPIO%d asserted, client selected)\n", TAG, PIN_CS);
  if (VERBOSE_LOG) Serial.flush();  // keep UART work out of the CS window
  uint32_t tCsLow = micros();
  gpio_set_level((gpio_num_t)PIN_CS, 0);
  delayMicroseconds(2);  // CS-to-first-clock setup time for the slave

  esp_err_t err = spi_device_polling_transmit(dev, &t);

  delayMicroseconds(1);
  gpio_set_level((gpio_num_t)PIN_CS, 1);
  uint32_t tCsHigh = micros();

  VLOG("[%s] CS -> HIGH (GPIO%d deasserted) after %lu us, %u bytes clocked each way\n", TAG, PIN_CS,
       (unsigned long)(tCsHigh - tCsLow), (unsigned)txLen);
  (void)tCsLow;
  (void)tCsHigh;

#if USE_HANDSHAKE
  // Back to back, the next handshake check could otherwise see this transaction's stale HIGH
  // before the client's post_trans_cb drops it, and clock into a slave with nothing queued.
  uint32_t lowStart = micros();
  while (gpio_get_level((gpio_num_t)PIN_HANDSHAKE) && micros() - lowStart < 1000) {
  }
#endif

  if (err != ESP_OK) {
    VLOG("[%s] spi_device_polling_transmit failed: %s\n", TAG, esp_err_to_name(err));
    return false;
  }

  if (VERBOSE_LOG) {
    hexdump(TAG, "sent on MOSI", txBuf, txLen);
    hexdump(TAG, "received on MISO", rxBuf, txLen);
    printFrame(TAG, "RX (client ack for previous frame)", rxBuf, txLen);
  }

  FrameHeader h;
  return frameMatches(rxBuf, txLen, TYPE_ACK, "", 0, h);
}

void loop() {
#if SEND_INTERVAL_US > 0
  static uint32_t nextUs = micros();
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += SEND_INTERVAL_US;
  // If we fell behind (e.g. a slow handshake), resume the schedule instead of bursting to catch up.
  if ((int32_t)(now - nextUs) > 0) nextUs = now + SEND_INTERVAL_US;
#endif

  tally(TAG, transact(), SNAPSHOT_LEN);
}

// ===========================================================================
#else  // ROLE_CLIENT
// ===========================================================================

#include "driver/gpio.h"
#include "driver/spi_slave.h"

static const char *TAG = "CLIENT";
static constexpr spi_host_device_t HOST = SPI2_HOST;  // HSPI, routed to the UEXT pins via the GPIO matrix

static uint16_t lastGoodSeq = 0;
static uint32_t goodCount = 0;
static uint32_t badCount = 0;

// --- Handshake: high while a transaction is loaded and waiting for CS -------
static void IRAM_ATTR onPostSetup(spi_slave_transaction_t *) {
#if USE_HANDSHAKE
  gpio_set_level((gpio_num_t)PIN_HANDSHAKE, 1);
#endif
}

static void IRAM_ATTR onPostTrans(spi_slave_transaction_t *) {
#if USE_HANDSHAKE
  gpio_set_level((gpio_num_t)PIN_HANDSHAKE, 0);
#endif
}

// --- CS edge capture for logging (ISR can't print, so buffer the edges) -----
struct CsEdge {
  uint32_t us;
  uint8_t level;
};
static constexpr uint8_t CS_EDGE_SLOTS = 16;
static volatile CsEdge csEdges[CS_EDGE_SLOTS];
static volatile uint8_t csHead = 0;
static uint8_t csTail = 0;

static void IRAM_ATTR onCsEdge() {
  uint8_t i = csHead % CS_EDGE_SLOTS;
  csEdges[i].us = micros();
  csEdges[i].level = gpio_get_level((gpio_num_t)PIN_CS);
  csHead = csHead + 1;
}

static void printCsEdges() {
  static uint32_t lastFallUs = 0;
  uint8_t head = csHead;
  if ((uint8_t)(head - csTail) > CS_EDGE_SLOTS) {
    Serial.printf("[%s] (CS edge log overflowed, %u edges dropped)\n", TAG,
                  (unsigned)(uint8_t)(head - csTail - CS_EDGE_SLOTS));
    csTail = head - CS_EDGE_SLOTS;
  }
  while (csTail != head) {
    uint8_t i = csTail % CS_EDGE_SLOTS;
    uint32_t us = csEdges[i].us;
    if (csEdges[i].level == 0) {
      lastFallUs = us;
      Serial.printf("[%s] CS LOW  seen (GPIO%d asserted by server) @ %lu us\n", TAG, PIN_CS, (unsigned long)us);
    } else {
      Serial.printf("[%s] CS HIGH seen (GPIO%d released by server) @ %lu us, held low %lu us\n", TAG, PIN_CS,
                    (unsigned long)us, (unsigned long)(us - lastFallUs));
    }
    csTail++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  printBanner("CLIENT (SPI slave)");
  allocBuffers();

#if USE_HANDSHAKE
  gpio_config_t hs = {};
  hs.pin_bit_mask = 1ULL << PIN_HANDSHAKE;
  hs.mode = GPIO_MODE_OUTPUT;
  gpio_config(&hs);
  gpio_set_level((gpio_num_t)PIN_HANDSHAKE, 0);
#endif

  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = MAX_FRAME_SIZE;  // default is only 4092 with DMA

  spi_slave_interface_config_t slave = {};
  slave.spics_io_num = PIN_CS;
  slave.flags = 0;
  slave.queue_size = 1;
  slave.mode = SPI_MODE_NUM;
  slave.post_setup_cb = onPostSetup;
  slave.post_trans_cb = onPostTrans;

  // Pull-ups so a disconnected/resetting server doesn't look like clock or CS activity.
  gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_SCK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_CS, GPIO_PULLUP_ONLY);

  esp_err_t err = spi_slave_initialize(HOST, &bus, &slave, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    Serial.printf("FATAL: spi_slave_initialize failed: %s\n", esp_err_to_name(err));
    while (true) delay(1000);
  }

  // Also watch CS with a GPIO interrupt, purely for logging.
  attachInterrupt(PIN_CS, onCsEdge, CHANGE);

  blockStartUs = micros();
}

void loop() {
  // Header-only ack, clocked back out on MISO during the *next* transaction.
  // The rest of txBuf stays zero from calloc, so MISO reads 0 after the header.
  size_t ackLen = buildFrame(txBuf, TYPE_ACK, lastGoodSeq, "", 0);

  spi_slave_transaction_t t = {};
  t.length = MAX_FRAME_SIZE * 8;  // bits; the upper bound, the server may clock less
  t.tx_buffer = txBuf;
  t.rx_buffer = rxBuf;

  esp_err_t err = spi_slave_queue_trans(HOST, &t, portMAX_DELAY);
  if (err != ESP_OK) {
    Serial.printf("[%s] spi_slave_queue_trans failed: %s\n", TAG, esp_err_to_name(err));
    delay(1000);
    return;
  }
  VLOG("\n[%s] buffer queued, handshake -> HIGH; ack ready (%u bytes, acks seq=%u)\n", TAG, (unsigned)ackLen,
       lastGoodSeq);
  VLOG("[%s] waiting for server to assert CS...\n", TAG);
  (void)ackLen;

  // Stays on in both modes: it only fires when the server has gone quiet.
  spi_slave_transaction_t *done = nullptr;
  while (spi_slave_get_trans_result(HOST, &done, pdMS_TO_TICKS(5000)) == ESP_ERR_TIMEOUT) {
    bool sawEdges = csTail != csHead;
    printCsEdges();
    Serial.printf("[%s] ...still waiting, no complete transaction in 5 s (CS pin now %s%s)\n", TAG,
                  gpio_get_level((gpio_num_t)PIN_CS) ? "HIGH" : "LOW",
                  sawEdges ? "" : ", NO CS edges seen: check UEXT pin 10 wiring");
    blockStartUs = micros();  // don't count the idle time in the next rate
  }

  if (VERBOSE_LOG)
    printCsEdges();
  else
    csTail = csHead;  // discard edges we won't print

  size_t rxBytes = done->trans_len / 8;
  VLOG("[%s] transaction complete, handshake -> LOW; received %u bits (%u bytes)\n", TAG,
       (unsigned)done->trans_len, (unsigned)rxBytes);
  if (done->trans_len % 8) VLOG("[%s] WARNING: bit count not a whole number of bytes\n", TAG);

  if (VERBOSE_LOG) {
    hexdump(TAG, "received on MOSI", rxBuf, rxBytes);
    hexdump(TAG, "sent on MISO", txBuf, sizeof(FrameHeader));
    printFrame(TAG, "RX", rxBuf, rxBytes);
  }

  FrameHeader h;
  bool ok = frameMatches(rxBuf, rxBytes, TYPE_DATA, METER_SNAPSHOT_JSON, SNAPSHOT_LEN, h);
  if (ok) {
    goodCount++;
    if (lastGoodSeq && h.seq != (uint16_t)(lastGoodSeq + 1))
      VLOG("[%s] note: seq jumped %u -> %u (%u frame(s) missed)\n", TAG, lastGoodSeq, h.seq,
           (unsigned)(uint16_t)(h.seq - lastGoodSeq - 1));
    lastGoodSeq = h.seq;
  } else {
    badCount++;
  }
  VLOG("[%s] payload %s the hardcoded meter snapshot\n", TAG, ok ? "MATCHES" : "DOES NOT MATCH");
  VLOG("[%s] totals: good=%lu bad=%lu\n", TAG, (unsigned long)goodCount, (unsigned long)badCount);

  tally(TAG, ok, ok ? h.len : 0);
}

#endif
