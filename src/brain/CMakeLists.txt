add_library(brain SHARED
            src/robot_buff_data.cpp
            src/robot_fight_data.cpp
            src/robot_body_request.cpp
            src/robot_brain.cpp
            src/robot_brain_control_command.cpp
            src/robot_brain_core.cpp
            src/robot_brain_core_param.cpp
            src/robot_brain_core_statistics.cpp
            src/robot_brain_notify_command.cpp
            src/robot_brain_param.cpp)
target_link_libraries(brain
                      armor
                      windmill
                      camera
#                      common
                      solver
                      uart
                     ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(test_robot_body_request
               test/test_robot_body_request.cpp)
target_link_libraries(test_robot_body_request
                      brain)

add_executable(test_robot_brain_control_command
               test/test_robot_brain_control_command.cpp)
target_link_libraries(test_robot_brain_control_command
                      brain)

add_executable(test_robot_brain_notify_command
               test/test_robot_brain_notify_command.cpp)
target_link_libraries(test_robot_brain_notify_command
                      brain)

add_executable(test_robot_brain_param
               test/test_robot_brain_param.cpp)
target_link_libraries(test_robot_brain_param
                      brain)