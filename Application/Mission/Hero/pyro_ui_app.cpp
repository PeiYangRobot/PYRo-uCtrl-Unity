// ReSharper disable CppExpressionWithoutSideEffects
#include "pyro_com_canrx.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_ui_drv.h"
#include "pyro_quad_booster.h"
#include "pyro_rc_hub.h"
#include "struct.h"

using namespace pyro;

static pyro::vt03_drv_t::vt03_ctrl_t const *vt03_ctrl_ptr = nullptr;
static pyro::referee_drv_t *referee_ptr                   = nullptr;
static pyro::ui_drv_t *ui_ptr                             = nullptr;

static float fric1_mps                                    = 0.0f;
static float fric2_mps                                    = 0.0f;
static bool fric_en                                       = false;
static bool flush_flag                                    = false;
static bool speed_control_en                              = false;
static bool fric1_online                                  = false;
static bool fric2_online                                  = false;

/**
 * @brief 静态 UI 绘制（仅在初始化或手动刷新时调用，使用 ADD）
 */
void ui_draw_static()
{
    // ui_ptr
    //     ->draw_circle("C01", pyro::ui_operate::ADD, 1, pyro::ui_color::GREEN,
    //     2,
    //                   960, 540, 100)
    // 准星竖线
    ui_ptr
        ->draw_line("L01", pyro::ui_operate::ADD, 1, pyro::ui_color::GREEN, 4,
                    860, 615, 1060, 615)
        .draw_line("L02", ui_operate::ADD, 1, pyro::ui_color::ORANGE, 4, 860,
                   585, 1060, 585);
    ui_ptr->flush(); // 拼包发送
    vTaskDelay(pdMS_TO_TICKS(50));
    // 1. 绘制静态文本标签 (注意名字不能重复)
    ui_ptr->draw_string("ST1", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 2, 110, 800, "FRIC1:");
    vTaskDelay(pdMS_TO_TICKS(50));
    ui_ptr->draw_string("ST2", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 2, 110, 750, "FRIC2:");
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 为动态数值提前进行 ADD 占位，赋予初始值，方便后续直接 MODIFY
    ui_ptr
        ->draw_float("DF1", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                     2, 230, 800, 0.0f)
        .draw_float("DF2", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                    2, 230, 750, 0.0f);
    ui_ptr->flush(); // 拼包发送

    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief 动态 UI 更新（定时刷新，必须使用 MODIFY）
 */
void ui_update_dynamic()
{
    // 修改摩擦轮速度值 (MODIFY)
    if (fric_en)
    {
        if (fric1_online)
        {
            if (speed_control_en)
            {
                ui_ptr->draw_float("DF1", pyro::ui_operate::MODIFY, 4,
                                   pyro::ui_color::YELLOW, 20, 2, 230, 800,
                                   fric1_mps);
            }
            else
            {
                ui_ptr->draw_float("DF1", pyro::ui_operate::MODIFY, 4,
                                   pyro::ui_color::WHITE, 20, 2, 230, 800,
                                   fric1_mps);
            }
        }
        else
        {
            ui_ptr->draw_float("DF1", pyro::ui_operate::MODIFY, 4,
                               pyro::ui_color::BLACK, 20, 2, 230, 800, 0.0f);
        }
        if (fric2_online)
        {
            if (speed_control_en)
            {
                ui_ptr->draw_float("DF2", pyro::ui_operate::MODIFY, 4,
                                   pyro::ui_color::YELLOW, 20, 2, 230, 750,
                                   fric2_mps);
            }
            else
            {
                ui_ptr->draw_float("DF2", pyro::ui_operate::MODIFY, 4,
                                   pyro::ui_color::WHITE, 20, 2, 230, 750,
                                   fric2_mps);
            }
        }
        else
        {
            ui_ptr->draw_float("DF2", pyro::ui_operate::MODIFY, 4,
                               pyro::ui_color::BLACK, 20, 2, 230, 750, 0.0f);
        }
    }
    else
    {
        ui_ptr->draw_float("DF1", pyro::ui_operate::MODIFY, 4,
                               pyro::ui_color::WHITE, 20, 2, 230, 800, 0.0f);
        ui_ptr->draw_float("DF2", pyro::ui_operate::MODIFY, 4,
                               pyro::ui_color::WHITE, 20, 2, 230, 750, 0.0f);
    }
    ui_ptr->flush();
}

void uirxbooster()
{
    std::array<uint8_t, 8> raw_data{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can2, 0x110,
                                     raw_data))
    {
        fric1_mps        = static_cast<float>(raw_data[0]) / 10.0f;
        fric2_mps        = static_cast<float>(raw_data[1]) / 10.0f;
        fric_en          = (raw_data[2] & 0x01) != 0;
        flush_flag       = (raw_data[2] & 0x02) != 0;
        speed_control_en = (raw_data[2] & 0x04) != 0;
        fric1_online     = (raw_data[2] & 0x08) != 0;
        fric2_online     = (raw_data[2] & 0x10) != 0;
    }
}


extern "C"
{
    void hero_ui_thread(void *argument)
    {
        // 1. 阻塞等待裁判系统链路连通，并确保获取到了真实机器人ID
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // 2. 初始清理操作，并绘制静态结构
        ui_ptr->clear_all();
        vTaskDelay(pdMS_TO_TICKS(200));
        ui_draw_static();
        vTaskDelay(pdMS_TO_TICKS(100));

        while (true)
        {
            uirxbooster();

            if (referee_ptr->is_online())
            {
                if (flush_flag)
                {
                    ui_ptr->clear_all();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    ui_draw_static();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    flush_flag = false;
                }
                else
                {
                    ui_update_dynamic();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void hero_ui_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(can_hub_t::can2, 0x110);
        vt03_ctrl_ptr = static_cast<pyro::vt03_drv_t::vt03_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->read());
        referee_ptr = pyro::referee_drv_t::get_instance();
        ui_ptr      = new pyro::ui_drv_t(referee_ptr);

        xTaskCreate(hero_ui_thread, "hero_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);

        vTaskDelete(nullptr);
    }
}