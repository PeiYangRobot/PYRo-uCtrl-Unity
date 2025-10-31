# Shoot Component


该目录存放发射机构相关的代码。

---
**Change Log**

* V1.0, 2025-10-26, By Pason: created
  * 发弹机构目前可以实现设置摩擦轮、拨弹盘速度，通过接收遥控器数据进行发弹控制，连续打弹
  已经测试，单发还没写，目前正在写拨弹盘的防堵弹机制，各个pid还没有经过图形化细调
* V1.0.1, 2025-10-28, By Pason: modified
  1. fric_drv
    a. 枚举类``rotate_direction_t``
      顺时针/逆时针
    b. 构造函数
      构造时需要传入电机对象指针，速度环pid参数，摩擦轮半径，旋转方向
    c. ``set_dt``
      传入控制周期，若不调用，``_dt``默认为0.001f
    d. ``set_speed``
      设置摩擦轮旋转线速度
    e. ``zero_force``
      发送电流为0
    f. ``get_speed``
      获取当前线速度
    g. ``update_feedback``
      更新电机反馈数据
    h. 声明了友元类``vofa_drv_t``，便于vofa示波
  
  2. trigger_drv_t
    a. 枚举类``trigger_mode_t``
      位置环/速度环
    b. 构造函数
      构造时需要传入电机对象指针，速度环pid参数，位置环pid参数，单发转动弧度
    c. ``set_dt``
      传入控制周期，若不调用，``_dt``默认为0.001f
    d. ``set_gear_ratio``
      设置减速比，若不调用默认为1
    e. ``set_rotate``
      设置转动角速度，调用函数将直接让电机转动
    f. ``set_radian``
      设置电机转动目标位置（弧度），调用函数将直接让电机转动
    g. ``get_rotate``
      获取当前角速度
    h. ``get_radian``
      获取当前位置（弧度）
    i. ``zero_force``
      发送电流为0
    j. ``update_feedback``
      更新电机反馈数据
    k. 声明了友元类``vofa_drv_t``，便于vofa示波

  3. shoot_base_t
    a. 枚举类``total_mode_t``
        ```
        enum total_mode_t
        {
            ZERO_FORCE         = 0x00, //无力
           RC_CONTROL         = 0x01, //遥控器控制
            AUTO_AIM_CONTROL   = 0x02  //自瞄控制
        };
        ```
      b. 枚举类``local_mode_t``
      ```
      enum local_mode_t
      {
          SHOOT_STOP         = 0x00,
          SHOOT_SETUP        = 0x01,
          SHOOT_READY        = 0x02,
          SHOOT_START        = 0x03,
          SHOOT_WAIT         = 0x04,
          SHOOT_CONTINUOUS   = 0x05
      };
      ```
  - ``SHOOT_STOP``先设定当前速度为0，检测到当前速度小于某个阈值后无力，拨弹盘需要
    ``set_angle = current_angle``
  - ``SHOOT_SETUP``设定摩擦轮到指定速度，检查摩擦轮当前速度与设定速度的误差是否在一定
    阈值内，拨弹盘``set_angle = 进入此状态时的角度`并且在当前状态中不再修改`
  - ``SHOOT_READY``检测到摩擦轮正常，允许拨弹盘转动
  - ``SHOOT_START``移动拨弹盘打一发弹
  - ``SHOOT_WAIT``拨弹盘正在转动，确定下一个状态，检测状态保持时间，小弹丸：暂定，
    大弹丸：``MAX_DELAY``
    - 对于小弹丸，进``SHOOT_READY``或者``SHOOT_CONTINUOUS``
    - 对于大弹丸，进``SHOOT_READY``

  - ``SHOOT_CONTINUOUS``小弹丸独有，连续发射，拨弹盘速度环控制

    c. ``get_mode``
      获取遥控器数据，直接改变模式
    d. ``update_feedback``
      虚函数，需要重写
    e. ``zero_force``
      虚函数，需要重写
    f. ``virtual void shoot_control()``
      虚函数，需要重写
    g. ``_total_mode _local_mode``
      私有，模式变量

* V1.0.2, 2025-10-29, By Pason: modified
  shoot_base_t 类基本完成
