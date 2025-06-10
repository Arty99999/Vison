add_library(armor SHARED
            src/classical_armor.cpp
            src/classical_armor_evaluation_weight.cpp
            src/classical_armor_recognizer.cpp
            src/classical_armor_recognizer_param.cpp
            src/classical_light_bar.cpp)
target_link_libraries(armor
                      ${OpenCV_LIBS}
#                      common
                      ${CERES_LIBRARIES})
#
add_executable(test_classical_armor_recognizer_param
               test/test_classical_armor_recognizer_param.cpp)
target_link_libraries(test_classical_armor_recognizer_param
                      armor
                      camera)

add_executable(test_classical_detect_light_bar_pairs
               test/test_classical_detect_light_bar_pairs.cpp)
target_link_libraries(test_classical_detect_light_bar_pairs
                      armor
                      camera)

add_executable(test_classical_detect_polished_armors
               test/test_classical_detect_polished_armors.cpp)
target_link_libraries(test_classical_detect_polished_armors
                      armor
                      camera)

add_executable(test_classical_detect_polished_light_bars
               test/test_classical_detect_polished_light_bars.cpp)
target_link_libraries(test_classical_detect_polished_light_bars
                      armor
                      camera)

add_executable(test_classical_detect_raw_armors
               test/test_classical_detect_raw_armors.cpp)
target_link_libraries(test_classical_detect_raw_armors
                      armor
                      camera)

add_executable(test_classical_detect_raw_light_bars
               test/test_classical_detect_raw_light_bars.cpp)
target_link_libraries(test_classical_detect_raw_light_bars
                      armor
                      camera)

add_executable(test_classical_evaluate_polished_armors
               test/test_classical_evaluate_polished_armors.cpp)
target_link_libraries(test_classical_evaluate_polished_armors
                      armor
                      camera)

add_executable(test_classical_preprocess_armor_image
               test/test_classical_preprocess_armor_image.cpp)
target_link_libraries(test_classical_preprocess_armor_image
                      armor
                      camera)