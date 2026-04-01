    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
            PYRo/Application/Mission/Hero_hybrid/pyro_chassis_app.cpp
            PYRo/Application/Mission/Hero_hybrid/pyro_booster_app.cpp

            PYRo/Module/Booster/Quad_tango/pyro_quad_booster.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_passive_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_active_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_homing_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_ready_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_stall_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_interim_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_busy_state.cpp

            PYRo/Module/Chassis/Hybrid/pyro_hybrid_chassis.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_active_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_passive_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_cruising_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_climbing_state.cpp
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Booster/Quad_tango
            PYRo/Module/Chassis/Hybrid
    )