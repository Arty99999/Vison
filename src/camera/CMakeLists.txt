add_library(camera SHARED
            include/huaray_camera.h
            src/huaray_camera.cpp
            src/huaray_camera_data.cpp
            src/huaray_camera_hardware_param.cpp
            src/huaray_camera_model_param.cpp
            src/huaray_camera_param.cpp
            src/huaray_camera_runtime_param.cpp
            src/huaray_camera_status.cpp)
target_link_libraries(camera
                      ${OpenCV_LIBS}
                      MVSDK
                      common
                      logger)

add_executable(test_huaray_camera_offline
               test/test_huaray_camera_offline.cpp)
target_link_libraries(test_huaray_camera_offline
                      camera)

add_executable(test_huaray_camera_online_1
               test/test_huaray_camera_online_1.cpp)
target_link_libraries(test_huaray_camera_online_1
                      camera)

add_executable(test_huaray_camera_online_2
               test/test_huaray_camera_online_2.cpp)
target_link_libraries(test_huaray_camera_online_2
                      camera)

add_executable(test_huaray_camera_param
               test/test_huaray_camera_param.cpp)
target_link_libraries(test_huaray_camera_param
                      camera)

add_executable(test_huaray_camera_reconnect
               test/test_huaray_camera_reconnect.cpp)
target_link_libraries(test_huaray_camera_reconnect
                      camera)

add_executable(test_huaray_camera_switch_hardware_param
               test/test_huaray_camera_switch_hardware_param.cpp)
target_link_libraries(test_huaray_camera_switch_hardware_param
                      camera)