#ifndef __PYRO_DWT_DRV_H__
#define __PYRO_DWT_DRV_H__

#include <stdint.h>

class pyro_dwt_drv_t
{
public:
    void init(uint32_t system_frequency);
    float get_dt(void);
    float get_dt64(void);
    static float get_time_s(void);
    static float get_time_ms(void);
    static uint64_t get_time_us(void); 
    static uint64_t get_total_cnt(void);
    static void delay_ms(void);
    static void delay_us(void);
    
private:
    static uint32_t _freq_s;
    static uint32_t _freq_ms;
    static uint32_t _freq_us;
    static uint64_t _total_cnt;
    static uint32_t _toatl_round;
    static uint32_t _sys_s;
    static uint32_t _sys_ms;
    static uint32_t _sys_us;

    static void __update_cnt(void);
    static void __update_sys(void);

};


#endif