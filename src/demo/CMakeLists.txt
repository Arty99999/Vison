

add_executable(automatic_shoot_armor_demo
        src/automatic_shoot_armor_demo.cpp )
target_link_libraries(automatic_shoot_armor_demo
        brain
        predictor)

add_executable(compensate_armor_velocity_demo
               src/compensate_armor_velocity_demo.cpp)
target_link_libraries(compensate_armor_velocity_demo
                      camera
                      armor
                      solver)

add_executable(compensate_large_buff_velocity_demo
               src/compensate_large_buff_velocity_demo.cpp)
target_link_libraries(compensate_large_buff_velocity_demo
                      camera
                      windmill
                      solver)

add_executable(compensate_small_buff_velocity_demo
               src/compensate_small_buff_velocity_demo.cpp)
target_link_libraries(compensate_small_buff_velocity_demo
                      camera
                      windmill
                      solver)

add_executable(exchange_data_demo
               src/exchange_data_demo.cpp)
target_link_libraries(exchange_data_demo
                      uart
                      brain)

add_executable(issue_control_command
               src/issue_control_command.cpp)
target_link_libraries(issue_control_command
                      brain)

add_executable(issue_notify_command
               src/issue_notify_command.cpp)
target_link_libraries(issue_notify_command
                      brain)

add_executable(shoot_base_demo
               src/shoot_base_demo.cpp)
target_link_libraries(shoot_base_demo
                      brain)

add_executable(shoot_ground_robot_demo_1
               src/shoot_ground_robot_demo_1.cpp)
target_link_libraries(shoot_ground_robot_demo_1
                      brain)

add_executable(shoot_ground_robot_demo_2
               src/shoot_ground_robot_demo_2.cpp)
target_link_libraries(shoot_ground_robot_demo_2
                      brain)

add_executable(shoot_large_buff_demo
               src/shoot_large_buff_demo.cpp)
target_link_libraries(shoot_large_buff_demo
                      brain)

add_executable(shoot_outpost_demo
               src/shoot_outpost_demo.cpp)
target_link_libraries(shoot_outpost_demo
                      brain)

add_executable(shoot_sentry_demo
               src/shoot_sentry_demo.cpp)
target_link_libraries(shoot_sentry_demo
                      brain)

add_executable(shoot_small_buff_demo
               src/shoot_small_buff_demo.cpp)
target_link_libraries(shoot_small_buff_demo
                      brain)