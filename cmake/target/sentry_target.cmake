    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
            PYRo/Application/Mission/Sentry/pyro_sentry_chassis_app.cpp

            PYRo/Module/Chassis/Rudder/pyro_rud_chassis.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_active_state.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_passive_state.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_moving_state.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_braking_state.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_turning_state.cpp           
            
            PYRo/Module/Yaw/pyro_yaw.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_passive_state.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_active_state.cpp
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Chassis/Rudder
            PYRo/Module/Yaw
    )