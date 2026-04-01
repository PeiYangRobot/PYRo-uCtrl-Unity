target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        PYRo/Application/Mission/Infantry_2/pyro_infantry2_chassis_app.cpp
        PYRo/Application/Mission/Infantry_2/pyro_infantry2_chassis_cfg.cpp
        
        PYRo/Module/Chassis/Wheel_Legged/pyro_wl_chassis.cpp
        PYRo/Module/Chassis/Wheel_Legged/fsm/pyro_wl_passive_state.cpp
        PYRo/Module/Chassis/Wheel_Legged/fsm/pyro_wl_active_state/pyro_wl_active_state.cpp
        PYRo/Module/Chassis/Wheel_Legged/fsm/pyro_wl_active_state/pyro_wl_test_state.cpp
        PYRo/Module/Chassis/Wheel_Legged/fsm/pyro_wl_active_state/pyro_wl_normal_state.cpp
    )
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        PYRo/Module/Chassis/Wheel_Legged
    )