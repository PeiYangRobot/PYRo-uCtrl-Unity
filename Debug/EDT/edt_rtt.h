#ifndef EDT_RTT_H
#define EDT_RTT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void edt_rtt_init(void);
size_t edt_rtt_write(const uint8_t *data, size_t len);
size_t edt_rtt_write_log(const uint8_t *data, size_t len);
size_t edt_rtt_read_down(uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
