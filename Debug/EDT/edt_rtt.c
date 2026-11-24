#include "edt_rtt.h"
#include "SEGGER_RTT.h"

/* One writer per physical channel; Terminal (0) remains available to printf.
 * Initialize once: a later SEGGER_RTT_Init() would erase all configured channels. */
_Alignas(4) static uint8_t scope_buffer[8192];
_Alignas(4) static uint8_t log_buffer[512];
_Alignas(4) static uint8_t control_buffer[256];
static uint8_t initialized;
volatile int edt_rtt_init_status = -1;

void edt_rtt_init(void) {
  if (initialized) return;
  SEGGER_RTT_Init();
  int scope = SEGGER_RTT_ConfigUpBuffer(1, "EDT_SCOPE", scope_buffer,
      sizeof(scope_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  int log = SEGGER_RTT_ConfigUpBuffer(2, "EDT_LOG", log_buffer,
      sizeof(log_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  int control = SEGGER_RTT_ConfigDownBuffer(1, "EDT_CTRL", control_buffer,
      sizeof(control_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  edt_rtt_init_status = (scope < 0 || log < 0 || control < 0) ? -1 : 0;
  initialized = 1;
}

size_t edt_rtt_write(const uint8_t *data, size_t len) {
  return edt_rtt_init_status == 0 ? SEGGER_RTT_Write(1, data, (unsigned)len) : 0;
}

size_t edt_rtt_write_log(const uint8_t *data, size_t len) {
  return edt_rtt_init_status == 0 ? SEGGER_RTT_Write(2, data, (unsigned)len) : 0;
}

size_t edt_rtt_read_down(uint8_t *data, size_t len) {
  return edt_rtt_init_status == 0 ? SEGGER_RTT_Read(1, data, (unsigned)len) : 0;
}
