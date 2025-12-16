/**
 * @file pyro_vt03_rc_drv.h
 * @brief Header file for the PYRO VT03 Remote Control Driver.
 *
 * This file defines the `pyro::vt03_drv_t` class, which implements the
 * protocol-specific logic for decoding the data packets from a DJI VT03
 * receiver (used with remote controllers like the RoboMaster/DJI FPV).
 * It inherits from the generic `rc_drv_t` base class.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_VT03_RC_DRV_H__
#define __PYRO_VT03_RC_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"

/* Defines -------------------------------------------------------------------*/

namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Driver class for the DJI VT03 remote control protocol.
 *
 * Implements the concrete logic for handling VT03 data packets, including
 * unpacking channels, mouse/keyboard inputs, error checking, and signaling
 * the main processing thread.
 */
class vt03_drv_t : public rc_drv_t
{
    friend class rc_hub_t;
    /* Private Types - Raw Buffer --------------------------------------------*/
    /**
     * @brief Raw structure of the 21-byte VT03 data packet.
     *
     * Uses bit-fields to extract 11-bit channel data and switch data.
     * The `__packed` attribute is necessary for direct memory mapping.
     */
    typedef struct __packed
    {
        uint8_t sof1;
        uint8_t sof2;

        uint64_t ch0     : 11; // 11 bits
        uint64_t ch1     : 11; // 11 bits
        uint64_t ch2     : 11; // 11 bits
        uint64_t ch3     : 11; // 11 bits
        uint64_t gear    : 2;  // 2 bits
        uint64_t pause   : 1;  // 1 bit
        uint64_t fn_l    : 1;  // 1 bit
        uint64_t fn_r    : 1;  // 1 bit
        uint64_t wheel   : 11; // 11 bits
        uint64_t trigger : 1;  // 1 bit

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l : 2;
        uint8_t press_r : 2;
        uint8_t press_m : 2;
        uint16_t key_code;

        uint16_t crc;

    } vt03_buf_t;

    /* Private Types - Control Data ------------------------------------------*/
    /**
     * @brief Decoded structure for consumer use.
     *
     * Contains separated and scaled data for RC, mouse, and keyboard inputs.
     */

    /**
     * @brief Type alias for the data output callback function signature.
     */

  public:
    enum class gear_state_t
    {
        GEAR_LEFT  = 0,
        GEAR_MID   = 1,
        GEAR_RIGHT = 2,
    };
    enum class gear_ctrl_t
    {
        GEAR_NO_CHANGE    = 0,
        GEAR_LEFT_TO_MID  = 1,
        GEAR_MID_TO_RIGHT = 2,
        GEAR_RIGHT_TO_MID = 3,
        GEAR_MID_TO_LEFT  = 4,
    };
    enum class key_ctrl_t
    {
        KEY_RELEASED = 0,
        KEY_PRESSED  = 1,
        KEY_HOLD     = 2
    };
    typedef struct key_t
    {
        key_ctrl_t ctrl;
        uint32_t time;
    } key_t;
    typedef struct vt03_gear_t
    {
        gear_state_t state;
        gear_ctrl_t ctrl;
    } vt03_gear_t;
    typedef struct vt03_ctrl_t
    {
        struct
        {
            float ch_lx;       ///< Channel values scaled to [-1.0, 1.0]
            float ch_ly;       ///< Left stick Y
            float ch_rx;       ///< Right stick X
            float ch_ry;       ///< Right stick Y
            float wheel; ///< Wheel value scaled to [-1.0, 1.0]
            vt03_gear_t gear;
            key_t fn_l;
            key_t fn_r;
            key_t pause;
            key_t trigger;
        } rc;

        struct
        {
            float x;
            float y;
            float z;
            key_t press_l;
            key_t press_r;
            key_t press_m;
        } mouse;

        struct
        {
            key_t w;
            key_t s;
            key_t a;
            key_t d;
            key_t shift;
            key_t ctrl;
            key_t q;
            key_t e;
            key_t r;
            key_t f;
            key_t g;
            key_t z;
            key_t x;
            key_t c;
            key_t v;
            key_t b;
        } key;
    } vt03_ctrl_t;
    /* Public Members --------------------------------------------------------*/

    /* Public Methods - Construction and Lifecycle (Override)
     * ------------------*/
    status_t init() override;
    void enable() override;
    void disable() override;
    void thread() override;

    /* Public Methods - Configuration
     * ------------------------------------------*/


  private:
    explicit vt03_drv_t(uart_drv_t *vt03_uart);
    vt03_ctrl_t _vt03_ctrl{}; ///< The latest decoded control data.
    /* Private Methods - Overrides
     * ---------------------------------------------*/
    /**
     * @brief The callback registered with the UART driver (ISR context).
     *
     * Receives raw UART data and forwards it to the FreeRTOS message buffer.
     */
    bool rc_callback(uint8_t *buf, uint16_t len,
                     BaseType_t xHigherPriorityTaskWoken) override;

    /* Private Methods - Processing
     * --------------------------------------------*/
    /**
     * @brief Performs range checking and CRC on raw VT03 data.
     * @param vt03_buf Pointer to the raw data buffer.
     * @return PYRO_OK if data is valid, PYRO_ERROR otherwise.
     */
    static status_t error_check(const vt03_buf_t *vt03_buf);
    /**
     * @brief Checks for gear (switch) state changes.
     * @param vt03_gear The gear state object (to be updated).
     * @param state The new raw state from the receiver.
     */
    static void check_ctrl(vt03_gear_t &vt03_gear, uint8_t raw_state);
    /**
     * @brief Checks for key state changes (PRESSED, HOLD, RELEASED).
     * @param key The key state object (to be updated).
     * @param state The new raw state (0 or 1) from the receiver.
     */
    static void check_ctrl(key_t &key, uint8_t raw_state);
    /**
     * @brief Unpacks raw VT03 data into the `vt03_ctrl_t` structure.
     * @param vt03_buf Pointer to the raw data buffer to unpack.
     */
    void unpack(const vt03_buf_t *vt03_buf);
};

} // namespace pyro
#endif