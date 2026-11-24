#ifndef EDT_SCOPE_H
#define EDT_SCOPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ch_count = 0: generic dynamic scope; the Agent sends addresses/sizes/rate. */
void edt_scope_init(uint32_t rate_hz, uint8_t ch_count);
/* Bind a target f32 variable. Its address is advertised in CONFIG and its
 * value is copied at each sample. Keep the variable alive while streaming. */
void edt_scope_bind_channel(uint8_t idx, const volatile float *source);
void edt_scope_set_channel(uint8_t idx, float value);
void edt_scope_task(uint32_t now_ms);
void edt_log_send(const char *text);
/* 主机保活状态：1 = 主机断连超时（目标已停采并完全静默）。
 * 周期日志等空闲输出可据此静默（SELECT/心跳到达即恢复 0）。 */
int edt_scope_host_link_down(void);
/* Weak defaults cover STM32H723 RAM/FLASH regions. */
int edt_scope_address_valid(uint32_t addr, uint32_t size);
int edt_scope_read_target(uint32_t addr, uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
