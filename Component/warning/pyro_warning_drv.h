#ifndef WARNING_H
#define WARNING_H

#include "pyro_task.h"
#include "ws2812_driver.h"
#include "pyro_buzzer_player.h"
#include "pyro_buzzer_task.h"
#include "pyro_buzzer_notes.h"

using namespace pyro;

class warning_drv_t
{
  public:
    enum class RGB_waring_type_t
    {
        GREEN,
        RED,
        BLUE,
        PURPLE,
        YELLOW,
        CYAN,  //青色
        NONE,
        COUNT,
        
    };
    enum class buzzer_warning_level_t
    {
        NONE,//无报警
        PLAYING,//供自定义娱乐
        WARNING_LEVEL_1,
        WARNING_LEVEL_2,
        WARNING_LEVEL_3,
        WARNING_LEVEL_4,
        WARNING_LEVEL_5,
        WARNING_LEVEL_6,
        WARNING_LEVEL_7,
        WARNING_LEVEL_8,
        
    };
    static warning_drv_t& instance()
    {
        static warning_drv_t _inst("warning_task");
        return _inst;
    }

    // 暴露给 App 层的生命周期控制接口
    void start();
    void stop();


    void set_buzzer_warning_level(buzzer_warning_level_t level);
    void add_rgb_warning_type(RGB_waring_type_t type);
    void clear_rgb_warning_type(RGB_waring_type_t type);

  protected:

    // 构造函数接收引用
    warning_drv_t(const char *task_name);


  private:
    class warning_task_t : public task_base_t
    {
      public:
      
        warning_task_t(warning_drv_t *parent, const char *name);

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        warning_drv_t *_parent;
    };

    // 供内部任务调用的实际执行函数
    status_t task_init();
    void task_run_loop();

    warning_task_t _task;
    
    //蜂鸣器模块所需内容
    bool should_play_warning_tone(int tick, int level) const;
    void handle_buzzer(int tick, BuzzerPlayer &buzzer);
    BuzzerTask* _buzzer_task = nullptr;  
    buzzer_warning_level_t _current_buzzer_warning_level = buzzer_warning_level_t::NONE;
    bool melody_loaded = false;

    //ws2812模块所需内容
    static constexpr uint16_t RGB_NUM = 1;
    static constexpr ws2812_hw_t RGB_HW = WS2812_HW_TIM3_CH2;
    ws2812_t* _rgb = nullptr;

    uint16_t rgb_waring_count[(int)RGB_waring_type_t::COUNT] = {0}; //每种颜色都持有一个倒计时
    RGB_waring_type_t color_lists[(int)RGB_waring_type_t::COUNT];
    uint8_t available_color_count = 0;

    static constexpr int TICK_PERIOD         = 1000; // 一个完整周期 tick 数 (5s)
    static constexpr int TICK_STEP           = 8;    // 每次循环 tick 增量
    static constexpr int WARN_TONE_FREQ_HZ   = 1000;
    static constexpr int WARN_TONE_VOLUME    = 100;
    static constexpr int WARN_PULSE_ON_TICK  = 50;   // 每段蜂鸣持续 tick 数
    static constexpr int WARN_PULSE_GAP_TICK = 100;  // 相邻蜂鸣起始间隔 tick 数
    static constexpr int RGB_DECAY_STEP      = 250;  // 每周期扣减的倒计时量

};



#endif