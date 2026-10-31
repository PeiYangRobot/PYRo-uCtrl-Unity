#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// MCU / OS（CFG_TUSB_MCU、CFG_TUSB_OS 由 CMake 传入，此处只校验）
//--------------------------------------------------------------------
#if !defined(CFG_TUSB_MCU)
#error CFG_TUSB_MCU must be defined by the build system
#endif

#define CFG_TUSB_DEBUG            0
#define CFG_TUD_ENABLED           1

//--------------------------------------------------------------------
// FreeRTOS 版本兼容补丁
//   本项目 FreeRTOS 为 V10.3.1（CubeMX 自带），而 TinyUSB 的
//   src/osal/osal_freertos.h::osal_time_millis() 使用了 pdTICKS_TO_MS()，
//   该宏自 FreeRTOS V10.4.0 才引入，因此不补会编译报错：
//     "implicit declaration of function 'pdTICKS_TO_MS'"
//   这里按 FreeRTOS V11 的语义补上（uint64 中间量避免 ticks*1000 溢出；
//   本项目 configTICK_RATE_HZ = 1000，结果即等于 tick 数）。
//   将来升级到 FreeRTOS >= V10.4.0 时，本定义会被 #ifndef 守卫自动让位。
//--------------------------------------------------------------------
#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks) \
    ((uint32_t)(((uint64_t)(xTicks) * 1000U) / configTICK_RATE_HZ))
#endif

//--------------------------------------------------------------------
// ★ STM32H723：只有 USB1_OTG_HS + 内置 FS PHY
//   CFG_TUD_MAX_SPEED 必须是 FULL：
//     -> TUD_OPT_HIGH_SPEED = 0
//     -> dwc2_core_is_highspeed_phy() = false
//     -> phy_fs_init() -> dwc2_phy_init(PHY_NOT_SUPPORTED) -> GCCFG.PWRDWN = 1（开内置 FS PHY）
//   若误设为 HIGH_SPEED，TinyUSB 会清掉 PWRDWN 导致完全不枚举。
//--------------------------------------------------------------------
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED
// 说明：TUD_OPT_HIGH_SPEED 只由 CFG_TUD_MAX_SPEED 决定（src/tusb_option.h:502）；
//       BOARD_TUD_MAX_SPEED 仅为兼容 board.mk 惯例保留，在 STM32 dwc2 端口上不生效。
#define BOARD_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

// ★ 必须与 pyro_usb_cdc_drv.cpp 中 tusb_int_handler(BOARD_TUD_RHPORT, true) 一致
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT          1
#endif

#define CFG_TUD_ENDPOINT0_SIZE    64

//--------------------------------------------------------------------
// Class
//--------------------------------------------------------------------
#define CFG_TUD_CDC               1
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0
#define CFG_TUD_CDC_NOTIFY        1   // 与 TUD_CDC_DESCRIPTOR 中的 notify 端点对应

// CDC FIFO（单帧仅 ~29B，给足余量便于吸收突发）
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    256
#define CFG_TUD_CDC_RX_EPSIZE     64
#define CFG_TUD_CDC_TX_EPSIZE     64

//--------------------------------------------------------------------
// TX 覆盖行为与 DTR 策略
//   默认 1：DTR=0（上位机未打开端口）时 TX FIFO 可被覆盖，tud_cdc_write() 不会失败，
//           即"设备以为在发、其实无人接收"，上层只能靠 50ms 无收包超时感知。
//   若实测确认上位机必然置 DTR，可改为 0 并让 write() 依赖 tud_cdc_connected()，
//   使"发送失败"能被上层立刻观测到。
//--------------------------------------------------------------------
#define CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED 1

// slave 模式（默认），与 CubeMX dma_enable = DISABLE 一致；无需 dcache 维护
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

#ifdef __cplusplus
}
#endif
#endif /* TUSB_CONFIG_H_ */
