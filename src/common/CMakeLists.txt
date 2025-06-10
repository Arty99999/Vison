#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=haswell")

# 需要加入该编译器设置，否则在Debug模式下，DeltaCV类下的方法耗时非常长
add_compile_options(-O3)

add_library(common SHARED
            src/bgr_weight.cpp
            src/delta_cv.cpp
            src/hsv_threshold.cpp
            src/keyboard_scanner.cpp
            src/shared_memory.cpp
            src/shared_memory_param.cpp
            src/system_configurator.cpp)
target_link_libraries(common
                      ${OpenCV_LIBS}
                      logger ${CERES_LIBRARIES})

add_executable(test_delta_cv
               test/test_delta_cv.cpp)
target_link_libraries(test_delta_cv
                      common)

add_executable(test_delta_cv_binarize_image
               test/test_delta_cv_binarize_image.cpp)
target_link_libraries(test_delta_cv_binarize_image
                      common)

add_executable(test_shared_memory_publish
               test/test_shared_memory_publish.cpp)
target_link_libraries(test_shared_memory_publish
                      common)

add_executable(test_shared_memory_subscribe
               test/test_shared_memory_subscribe.cpp)
target_link_libraries(test_shared_memory_subscribe
                      common)

add_executable(test_simd_instruction
               test/test_simd_instruction.cpp)