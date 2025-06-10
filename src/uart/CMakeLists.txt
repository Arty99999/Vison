add_library(uart SHARED
            src/serial_port.cpp
            src/serial_port_param.cpp)
target_link_libraries(uart
                      ${OpenCV_LIBS}
                      logger)

add_executable(test_serial_port
               test/test_serial_port.cpp)
target_link_libraries(test_serial_port
                      uart solver camera armor)

add_executable(test_serial_port_param
               test/test_serial_port_param.cpp)
target_link_libraries(test_serial_port_param
                      uart)

add_executable(test_search_ttyusb
               test/test_search_ttyusb.cpp)