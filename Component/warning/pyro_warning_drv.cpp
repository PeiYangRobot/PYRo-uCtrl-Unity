#include "pyro_warning_drv.h"
#include "pyro_core_def.h"
#include "string.h"

using namespace pyro;


// ===================================================================
// 私有内部任务类实现 (代理桥接)
// ===================================================================
warning_drv_t::warning_task_t::warning_task_t(warning_drv_t *parent, const char *name)
    : task_base_t(name, 0, 256, task_base_t::priority_t::REALTIME),
      _parent(parent)
{
}


warning_drv_t::warning_drv_t(const char *task_name)
    : _task(this, task_name)
{
}


status_t warning_drv_t::warning_task_t::init()
{
    return _parent->task_init();//_parent是拥有这个任务对象的类，放在这里就是warning_drv_t
    
}

void warning_drv_t::warning_task_t::run_loop()
{
    _parent->task_run_loop();
}

void warning_drv_t::start()
{
    _task.start();
}

void warning_drv_t::stop()
{
    _task.stop();
}

status_t warning_drv_t::task_init()
{
    //RGB初始化
    _rgb = new ws2812_t(RGB_HW);
    if(RGB_NUM > 0)
    {
        status_t ret1 = _rgb->init(RGB_NUM);
        if (ret1 != pyro::PYRO_OK)
        {
            return pyro::PYRO_ERROR;
        }
    }
    else 
    {
        return pyro::PYRO_ERROR;
    }
    available_color_count = 0;
    memset(color_lists, 0, sizeof(color_lists));
    memset(rgb_waring_count, 0, sizeof(rgb_waring_count));


    //蜂鸣器初始化
    _buzzer_task = new BuzzerTask();
    BuzzerPlayer::getInstance().init();

    // // See You Again 前奏
    // _buzzer_task = new BuzzerTask();
    // _buzzer_task->add_event(0,      BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(788,    0, 100);

    // _buzzer_task->add_event(888,    BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(1150,   0, 100);

    // _buzzer_task->add_event(1250,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(2038,   0, 100);

    // _buzzer_task->add_event(2138,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(2400,   0, 100);

    // _buzzer_task->add_event(2500,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(2762,   0, 100);

    // _buzzer_task->add_event(2862,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(3125,   0, 100);

    // _buzzer_task->add_event(3225,   BUZZER_GET_FREQ(BUZZER_NOTE_D5),  100);
    // _buzzer_task->add_event(3488,   0, 100);

    // _buzzer_task->add_event(3588,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(3850,   0, 100);

    // _buzzer_task->add_event(3950,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(4212,   0, 100);

    // _buzzer_task->add_event(4312,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(4575,   0, 100);

    // _buzzer_task->add_event(4675,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(5462,   0, 100);

    // _buzzer_task->add_event(5562,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(5825,   0, 100);

    // _buzzer_task->add_event(5925,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(6188,   0, 100);

    // _buzzer_task->add_event(6288,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(7075,   0, 100);

    // _buzzer_task->add_event(7175, 0, 100);
    // _buzzer_task->set_duration(7175);


    // 欢乐颂
    // 第一句：E4 E4 F4 G4
    _buzzer_task->add_event(0,    BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(300,  0, 100);   // 静音停200ms
    _buzzer_task->add_event(500,  BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(800,  0, 100);   // 静音停200ms
    _buzzer_task->add_event(1000, BUZZER_GET_FREQ(BUZZER_NOTE_F4), 100);
    _buzzer_task->add_event(1300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(1500, BUZZER_GET_FREQ(BUZZER_NOTE_G4), 100);
    _buzzer_task->add_event(1800, 0, 100);   // 静音停200ms

    // 第二句：G4 F4 E4 D4
    _buzzer_task->add_event(2000, BUZZER_GET_FREQ(BUZZER_NOTE_G4), 100);
    _buzzer_task->add_event(2300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(2500, BUZZER_GET_FREQ(BUZZER_NOTE_F4), 100);
    _buzzer_task->add_event(2800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(3000, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(3300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(3500, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(3800, 0, 100);   // 静音停200ms

    // 第三句：C4 C4 D4 E4
    _buzzer_task->add_event(4000, BUZZER_GET_FREQ(BUZZER_NOTE_C4), 100);
    _buzzer_task->add_event(4300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(4500, BUZZER_GET_FREQ(BUZZER_NOTE_C4), 100);
    _buzzer_task->add_event(4800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(5000, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(5300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(5500, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(5800, 0, 100);   // 静音停200ms

    // 第四句：E4 D4 D4
    _buzzer_task->add_event(6000, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(6300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(6500, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(6800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(7000, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(7300, 0, 100);

    _buzzer_task->add_event(8000, 0, 100);
    _buzzer_task->set_duration(8000);

    return status_t::PYRO_OK;
}

void warning_drv_t::task_run_loop()
{
    //根据当前的报警等级，控制蜂鸣器和灯带的状态
    auto& buzzer = BuzzerPlayer::getInstance();

    while (true)
    {
        static int tick = 0;
        tick += TICK_STEP;
        tick %= TICK_PERIOD; // 每5秒钟循环一次
        //蜂鸣器部分
        handle_buzzer(tick, buzzer);
        
        //rgb部分
        static int current_each_color_shine_times = 0;        //目前每个灯亮几个循环
        if(tick == 0)
        {
            available_color_count = 0;
            memset(color_lists, 0, sizeof(color_lists));
            //一轮循环后集体减一整轮循环的时间,并重新计算接下来这段时间会亮几种颜色
            for(int i = 0; i < (int)RGB_waring_type_t::COUNT - 1; i++)
            {
                if(rgb_waring_count[i] >= RGB_DECAY_STEP)
                {
                    rgb_waring_count[i] -= RGB_DECAY_STEP;
                    //记录下要亮的颜色
                    color_lists[available_color_count] = (RGB_waring_type_t)i;
                    available_color_count ++;
                    
                }
                else 
                {
                    rgb_waring_count[i] = 0;
                }
            }
            if (available_color_count > 0) 
            {
                current_each_color_shine_times = TICK_PERIOD / available_color_count;
            } else 
            {
                current_each_color_shine_times = 0;
            }
        }

        if(available_color_count != 0)
        {
            if(available_color_count == 6)
            {
                //不是哥们，六种灯全亮上了，那还说啥了
                //我直接给你个彩虹灯得了
                static uint16_t hue = 0;
                hue = (hue + 5) % 360; // 色相滚动
                auto output = _rgb->hsv_to_rgb(hue, 255, 100);
                _rgb->set_led(0, output.r, output.g, output.b);
            }
            else 
            {
                int idx = tick / current_each_color_shine_times;
                if (idx >= available_color_count) idx = available_color_count - 1;
                switch (color_lists[idx])
                {
                    case RGB_waring_type_t::BLUE:
                        _rgb->set_led(0, 0, 0, 255);
                        break;
                    case RGB_waring_type_t::CYAN:
                        _rgb->set_led(0, 0, 255, 255);
                        break;
                    case RGB_waring_type_t::GREEN:
                        _rgb->set_led(0, 0, 255, 0);
                        break;
                    case RGB_waring_type_t::PURPLE:
                        _rgb->set_led(0, 255, 0, 255);
                        break;
                    case RGB_waring_type_t::RED:
                        _rgb->set_led(0, 255, 0, 0);
                        break;
                    case RGB_waring_type_t::YELLOW:
                        _rgb->set_led(0, 255, 255, 0);
                        break;
                    default:
                        _rgb->set_led(0, 0, 0, 0);
                        break;
                }
            }
        }
        else 
        {
            //没有逝故，不亮灯
            _rgb->set_led(0, 0, 0, 0);
        }
        _rgb->update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
}

bool warning_drv_t::should_play_warning_tone(int tick, int level) const
{
    for (int i = 0; i < level; ++i)
    {
        int start = i * WARN_PULSE_GAP_TICK;
        if (tick >= start && tick < start + WARN_PULSE_ON_TICK)
            return true;
    }
    return false;
}

void warning_drv_t::handle_buzzer(int tick, BuzzerPlayer &buzzer)
{
    switch (_current_buzzer_warning_level)
    {
    case buzzer_warning_level_t::NONE:
        buzzer.stop();
        buzzer.tone_stop();
        return;

    case buzzer_warning_level_t::PLAYING:
        if (!melody_loaded && _buzzer_task != nullptr)
        {
            buzzer.loadTask(_buzzer_task);
            melody_loaded = true;
        }
        if (!(buzzer.getState() & BUZZER_STATE_MUSIC_PLAYING_BIT))
        {
            melody_loaded = false;
        }
        return;

    default:
        break;
    }
    buzzer.stop();
    // WARNING_LEVEL_1 ~ WARNING_LEVEL_8 统一处理
    int pulses = (int)_current_buzzer_warning_level -
                 (int)buzzer_warning_level_t::WARNING_LEVEL_1 + 1;

    if (should_play_warning_tone(tick, pulses))
    {
        buzzer.tone_set(WARN_TONE_FREQ_HZ, WARN_TONE_VOLUME);
        buzzer.tone_start();
    }
    else
    {
        buzzer.tone_stop();
    }
}





void warning_drv_t::set_buzzer_warning_level(buzzer_warning_level_t level)
    {
        taskENTER_CRITICAL();
        if(level == warning_drv_t::buzzer_warning_level_t::NONE)
        {
            _current_buzzer_warning_level = level;
        }
        _current_buzzer_warning_level = (int)level > (int)_current_buzzer_warning_level ? level : _current_buzzer_warning_level;
        melody_loaded = false;
        taskEXIT_CRITICAL();
    }
    void warning_drv_t::add_rgb_warning_type(RGB_waring_type_t type)
    {
        taskENTER_CRITICAL();
        if(type != RGB_waring_type_t::NONE && type != RGB_waring_type_t::COUNT)
        {
            rgb_waring_count[(int)type] = 15000;//周期数，换算出来是5min
        }
        taskEXIT_CRITICAL();
    }
    void warning_drv_t::clear_rgb_warning_type(RGB_waring_type_t type)
    {
        taskENTER_CRITICAL();
        if(type != RGB_waring_type_t::NONE && type != RGB_waring_type_t::COUNT)
        {
            rgb_waring_count[(int)type] = 0;
        }
        taskEXIT_CRITICAL();
    }