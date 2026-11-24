#include "edt_scope.h"
#include "edt_rtt.h"
#include <string.h>

#define EDT_SYNC0 0xA5u
#define EDT_SYNC1 0x5Au
#define EDT_TYPE_CONFIG 0x02u
#define EDT_TYPE_LOG 0x04u
#define EDT_TYPE_SCOPE_INDEXED 0x05u
#define EDT_TYPE_SCOPE_RAW 0x06u
#define EDT_TYPE_SELECT 0x10u
#define EDT_TYPE_CONTROL_ERROR 0x11u
/* 主机保活心跳（主机→下位机，零载荷）。主机 2s 一拍；超过 6s（3 拍+裕量）
 * 无任何下行帧（心跳/SELECT）即判定主机断连：停止采样并完全静默，
 * 收到 SELECT/心跳即恢复。 */
#define EDT_TYPE_HEARTBEAT 0x12u
#define EDT_HOST_TIMEOUT_MS 6000u
#define EDT_MAX_PAYLOAD 200u

static uint8_t ch_count;
static uint32_t rate_hz = 1000;
static float ch_value[8];
static const volatile float *ch_source[8];
static uint8_t scope_seq, log_seq;
static uint32_t start_ms, last_config_ms, sample_index;
static uint8_t sent_config;
static uint8_t config_dirty;
static uint8_t dynamic_scope;
static uint32_t generation, ch_addr[8], ch_size[8];
static uint8_t ctrl_rtt[128];
static size_t ctrl_rtt_len;
/* 主机保活：最近收到下行帧（心跳/SELECT）的 tick；0=从未收到 */
static uint32_t last_host_ms;
static uint8_t host_hb_seen;
static uint8_t host_link_down;
volatile uint32_t edt_scope_dropped_frames, edt_log_dropped_frames;

static uint16_t crc16(const uint8_t *d, uint32_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*d++) << 8;
    for (int b = 0; b < 8; ++b)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

static void put_u32(uint8_t *d, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) d[i] = (uint8_t)(value >> (8u * i));
}

static uint32_t get_u32(const uint8_t *d) {
  return d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

/* 覆盖 STM32H723VGTx 全内存区间（参考 STM32H723XG_FLASH.ld） */
__attribute__((weak)) int edt_scope_address_valid(uint32_t addr, uint32_t size) {
  if (!size || addr > UINT32_MAX - size) return 0;
  uint32_t end = addr + size;
  return (addr >= 0x20000000u && end <= 0x20020000u)  /* DTCMRAM (128 KB) */
      || (addr >= 0x24000000u && end <= 0x24050000u)  /* RAM_D1  (320 KB) */
      || (addr >= 0x30000000u && end <= 0x30008000u)  /* RAM_D2  (32 KB, .dma_heap) */
      || (addr >= 0x38000000u && end <= 0x38004000u)  /* RAM_D3  (16 KB) */
      || (addr >= 0x08000000u && end <= 0x08100000u)  /* FLASH   (1024 KB) */
      || (addr >= 0x00001000u && end <= 0x00010000u); /* ITCMRAM (>= 0x1000) */
}

__attribute__((weak)) int edt_scope_read_target(uint32_t addr, uint8_t *data, uint32_t size) {
  if (size == 4 && (addr & 3u) == 0u) {
    const volatile uint32_t *w = (const volatile uint32_t *)(uintptr_t)addr;
    uint32_t v = *w;
    data[0] = (uint8_t)v; data[1] = (uint8_t)(v >> 8);
    data[2] = (uint8_t)(v >> 16); data[3] = (uint8_t)(v >> 24);
    return 1;
  }
  if (size == 2 && (addr & 1u) == 0u) {
    const volatile uint16_t *h = (const volatile uint16_t *)(uintptr_t)addr;
    uint16_t v = *h;
    data[0] = (uint8_t)v; data[1] = (uint8_t)(v >> 8);
    return 1;
  }
  const volatile uint8_t *source = (const volatile uint8_t *)(uintptr_t)addr;
  for (uint32_t i = 0; i < size; ++i) data[i] = source[i];
  return 1;
}

/* SKIP mode applies to one complete frame, never header/payload/CRC separately. */
static int send_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
  if (len > EDT_MAX_PAYLOAD) return 0;
  const int is_log = type == EDT_TYPE_LOG;
  uint8_t frame[EDT_MAX_PAYLOAD + 8];
  frame[0] = EDT_SYNC0; frame[1] = EDT_SYNC1; frame[2] = type;
  frame[3] = is_log ? log_seq++ : scope_seq++;
  frame[4] = (uint8_t)len; frame[5] = (uint8_t)(len >> 8);
  memcpy(frame + 6, payload, len);
  uint16_t crc = crc16(frame + 2, (uint32_t)(4u + len));
  frame[6 + len] = (uint8_t)crc;
  frame[7 + len] = (uint8_t)(crc >> 8);
  size_t n = is_log ? edt_rtt_write_log(frame, len + 8u) : edt_rtt_write(frame, len + 8u);
  if (n != len + 8u) {
    if (is_log) ++edt_log_dropped_frames; else ++edt_scope_dropped_frames;
    return 0;
  }
  return 1;
}

void edt_scope_init(uint32_t hz, uint8_t m) {
  rate_hz = !hz ? 1000u : (hz > 1000u ? 1000u : hz);
  ch_count = m > 8 ? 8 : m;
  memset(ch_value, 0, sizeof(ch_value));
  memset(ch_source, 0, sizeof(ch_source));
  scope_seq = log_seq = 0;
  sample_index = start_ms = last_config_ms = 0;
  sent_config = 0;
  config_dirty = 1;
  dynamic_scope = m == 0;
  generation = 0;
  memset(ch_addr, 0, sizeof(ch_addr));
  memset(ch_size, 0, sizeof(ch_size));
  ctrl_rtt_len = 0;
  last_host_ms = 0;
  host_hb_seen = 0;
  host_link_down = 0;
  edt_scope_dropped_frames = edt_log_dropped_frames = 0;
}

void edt_scope_bind_channel(uint8_t idx, const volatile float *source) {
  if (idx < ch_count && ch_source[idx] != source) {
    ch_source[idx] = source;
    config_dirty = 1;
  }
}

void edt_scope_set_channel(uint8_t idx, float v) {
  if (idx < ch_count) ch_value[idx] = v;
}

int edt_scope_host_link_down(void) { return host_link_down; }

void edt_log_send(const char *text) {
  if (host_link_down) return;
  uint16_t n = 0;
  if (!text) return;
  while (n < EDT_MAX_PAYLOAD && text[n]) ++n;
  (void)send_frame(EDT_TYPE_LOG, (const uint8_t *)text, n);
}

static void control_error(uint32_t id, uint8_t code) {
  uint8_t payload[5]; put_u32(payload, id); payload[4] = code;
  (void)send_frame(EDT_TYPE_CONTROL_ERROR, payload, sizeof(payload));
}

static void apply_selection(const uint8_t *p, size_t n) {
  if (n < 12) return;
  uint32_t id = get_u32(p + 4), hz = get_u32(p + 8);
  unsigned count = p[1];
  if (p[0] != 1 || p[2] || p[3] || count > 8 || n != 12u + 8u * count || !hz) {
    control_error(id, 1); return;
  }
  for (unsigned i = 0; i < count; ++i) {
    uint32_t addr = get_u32(p + 12 + i * 8), size = get_u32(p + 16 + i * 8);
    if (size != 1 && size != 2 && size != 4 && size != 8) { control_error(id, 3); return; }
    if (!edt_scope_address_valid(addr, size)) { control_error(id, 2); return; }
  }
  hz = hz > 1000u ? 1000u : hz;
  int same = dynamic_scope && id == generation && hz == rate_hz && count == ch_count;
  for (unsigned i = 0; same && i < count; ++i)
    same = ch_addr[i] == get_u32(p + 12 + i * 8) && ch_size[i] == get_u32(p + 16 + i * 8);
  if (!same) {
    dynamic_scope = 1; generation = id; rate_hz = hz; ch_count = (uint8_t)count;
    for (unsigned i = 0; i < count; ++i) {
      ch_addr[i] = get_u32(p + 12 + i * 8); ch_size[i] = get_u32(p + 16 + i * 8);
    }
    sent_config = 0; sample_index = 0;
  }
  config_dirty = 1;
}

static void control_parse(uint8_t *buf, size_t *len_io) {
  size_t len = *len_io;
  while (len >= 2) {
    if (buf[0] != EDT_SYNC0 || buf[1] != EDT_SYNC1) {
      memmove(buf, buf + 1, --len); continue;
    }
    if (len < 6) break;
    size_t payload_len = buf[4] | ((size_t)buf[5] << 8);
    if (payload_len > 76) { memmove(buf, buf + 1, --len); continue; }
    size_t total = payload_len + 8;
    if (len < total) break;
    uint16_t expected = buf[6 + payload_len] | ((uint16_t)buf[7 + payload_len] << 8);
    if (crc16(buf + 2, (uint32_t)(4 + payload_len)) != expected) {
      memmove(buf, buf + 1, --len); continue;
    }
    if (buf[2] == EDT_TYPE_SELECT) {
      apply_selection(buf + 6, payload_len);
      host_hb_seen = 1;
    } else if (buf[2] == EDT_TYPE_HEARTBEAT) {
      host_hb_seen = 1;
    }
    len -= total; memmove(buf, buf + total, len);
  }
  *len_io = len;
}

static void poll_control(void) {
  for (unsigned pass = 0; pass < 4; ++pass) {
    size_t got = edt_rtt_read_down(ctrl_rtt + ctrl_rtt_len, sizeof(ctrl_rtt) - ctrl_rtt_len);
    ctrl_rtt_len += got;
    control_parse(ctrl_rtt, &ctrl_rtt_len);
    if (!got) break;
  }
}

void edt_scope_task(uint32_t now_ms) {
  poll_control();
  if (host_hb_seen) {
    host_hb_seen = 0;
    last_host_ms = now_ms;
    host_link_down = 0;
  }
  if (last_host_ms && now_ms - last_host_ms > EDT_HOST_TIMEOUT_MS) {
    host_link_down = 1;
  }
  if (host_link_down) {
    return;
  }
  if (!sent_config || config_dirty || (ch_count && now_ms - last_config_ms >= 1000u)) {
    uint8_t cfg[12 + 8 * 8] = {0};
    cfg[0] = ch_count;
    cfg[5] = dynamic_scope ? 2 : 1;
    cfg[6] = dynamic_scope ? 8 : 4;
    put_u32(cfg + 1, rate_hz);
    uint16_t length;
    if (dynamic_scope) {
      put_u32(cfg + 8, generation);
      for (unsigned i = 0; i < ch_count; ++i) {
        put_u32(cfg + 12 + 8 * i, ch_addr[i]); put_u32(cfg + 16 + 8 * i, ch_size[i]);
      }
      length = (uint16_t)(12u + 8u * ch_count);
    } else {
      for (unsigned i = 0; i < ch_count; ++i) put_u32(cfg + 8 + 4 * i, (uint32_t)(uintptr_t)ch_source[i]);
      length = (uint16_t)(8u + 4u * ch_count);
    }
    int ok = send_frame(EDT_TYPE_CONFIG, cfg, length);
    if (!sent_config) {
      if (!ok) return;
      sent_config = 1;
      start_ms = now_ms;
    }
    if (!ok && config_dirty) return;
    if (ok) config_dirty = 0;
    last_config_ms = now_ms;
  }
  if (!ch_count) return;
  uint32_t index = (uint32_t)(((uint64_t)(now_ms - start_ms) * rate_hz) / 1000u);
  if (index == sample_index) return;
  sample_index = index;
  if (dynamic_scope) {
    uint8_t payload[8 + 8 * 8]; put_u32(payload, sample_index); put_u32(payload + 4, generation);
    uint16_t length = 8;
    for (unsigned i = 0; i < ch_count; ++i) {
      if (!edt_scope_read_target(ch_addr[i], payload + length, ch_size[i])) {
        ++edt_scope_dropped_frames; return;
      }
      length += (uint16_t)ch_size[i];
    }
    (void)send_frame(EDT_TYPE_SCOPE_RAW, payload, length);
    return;
  }
  for (unsigned i = 0; i < ch_count; ++i)
    if (ch_source[i]) ch_value[i] = *ch_source[i];
  uint8_t payload[4 + sizeof(ch_value)];
  put_u32(payload, sample_index);
  memcpy(payload + 4, ch_value, ch_count * 4u);
  (void)send_frame(EDT_TYPE_SCOPE_INDEXED, payload, (uint16_t)(4u + ch_count * 4u));
}
