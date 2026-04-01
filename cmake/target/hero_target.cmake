    target_sources(${CMAKE_PROJECT_NAME} PRIVATE

            PYRo/Module/Booster/Quad_pink/pyro_quad_booster.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_passive_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_active_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_homing_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_ready_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_stall_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_interim_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_busy_state.cpp

            PYRo/Module/Gimbal/Hero/Direct/pyro_direct_gimbal.cpp
            PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_active_state.cpp
            PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_passive_state.cpp

            PYRo/Module/Chassis/Mecanum/pyro_mec_chassis.cpp
            PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_passive_state.cpp
            PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_active_state.cpp

            PYRo/Application/Mission/Hero/pyro_chassis_app.cpp
            PYRo/Application/Mission/Hero/pyro_gimbal_app.cpp
            PYRo/Application/Mission/Hero/pyro_booster_app.cpp
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Booster/Quad_pink
            PYRo/Module/Gimbal/Hero/Direct
            PYRo/Module/Chassis/Mecanum
    )