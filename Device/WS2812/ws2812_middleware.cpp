#include "ws2812_middleware.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "pyro_dwt_drv.h"

/* ============================================================
 *  STM32H7: 定时器时钟 = APB时钟 × 2 (当APB分频≠1时)
 *  自动判断定时器挂在APB1还是APB2
 *  APB2 (D2PPRE2): TIM1, TIM8, TIM15, TIM16, TIM17, HRTIM
 *  APB1 (D2PPRE1): TIM2~TIM7, TIM12~TIM14, LPTIM1
 * ============================================================ */
static uint32_t ws2812_get_timer_clock(TIM_HandleTypeDef* htim)
{
    uint32_t apb_clk;
    uint32_t psc_div;

    /* 判断定时器实例: APB2还是APB1 */
    if (htim->Instance == TIM1 || htim->Instance == TIM8 ||
        htim->Instance == TIM15 || htim->Instance == TIM16 || htim->Instance == TIM17)
    {
        /* APB2 定时器 */
        apb_clk = HAL_RCC_GetPCLK2Freq();
        /* H7: D2PPRE2 在 RCC->D2CFGR bit8~10 */
        psc_div = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos;
    }
    else
    {
        /* APB1 定时器 */
        apb_clk = HAL_RCC_GetPCLK1Freq();
        /* H7: D2PPRE1 在 RCC->D2CFGR bit4~6 */
        psc_div = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    }

    /* 分频值: 0xx=不分频(×1), 100=×2, 101=×4, 110=×8, 111=×16
     * 如果分频≠1(即psc_div >= 4), 定时器时钟 = APB × 2
     */
    if (psc_div >= 4U) {
        return apb_clk * 2U;
    } else {
        return apb_clk;
    }
}

void ws2812_hw_init(ws2812_hw_t* hw)
{
    (void)hw;
    /* 硬件配置由CubeMX完成, 此处无需操作 */
}

void ws2812_hw_deinit(ws2812_hw_t* hw)
{
    ws2812_hw_stop_dma(hw);
}

void ws2812_hw_start_dma(ws2812_hw_t* hw, const uint8_t* pulse_buf, uint16_t length)
{
    HAL_TIM_PWM_Start_DMA(hw->htim, hw->channel, (uint32_t*)pulse_buf, (uint32_t)length);
}

void ws2812_hw_stop_dma(ws2812_hw_t* hw)
{
    HAL_TIM_PWM_Stop_DMA(hw->htim, hw->channel);
}

uint8_t ws2812_hw_is_dma_busy(ws2812_hw_t* hw)
{
    if (hw->htim->hdma[hw->dma_id] != NULL) {
        if (hw->htim->hdma[hw->dma_id]->State == HAL_DMA_STATE_BUSY) {
            return 1U;
        }
    }
    return 0U;
}

void ws2812_hw_get_tim_config(ws2812_hw_t* hw, uint32_t* tim_clk, uint32_t* psc, uint32_t* arr)
{
    if (tim_clk != NULL) {
        *tim_clk = ws2812_get_timer_clock(hw->htim);
    }
    if (psc != NULL) {
        *psc = hw->htim->Init.Prescaler;
    }
    if (arr != NULL) {
        *arr = hw->htim->Init.Period;
    }
}

void ws2812_hw_delay_us(uint16_t us)
{
    pyro::dwt_drv_t::delay_us((uint32_t)us);
}

void ws2812_hw_delay_ms(uint16_t ms)
{
    pyro::dwt_drv_t::delay_us((uint32_t)ms * 1000U);
}
