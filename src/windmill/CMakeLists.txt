add_library(windmill SHARED
            src/classical_windmill_fan.cpp
            src/classical_windmill_logo.cpp
            src/classical_windmill_predictor.cpp
            src/classical_windmill_recognizer.cpp
            src/classical_windmill_recognizer_param.cpp
            src/classcal_windmill_cerese_predictor.cpp )
target_link_libraries(windmill
                      ${OpenCV_LIBS}
                      common)

add_executable(test_classical_windmill_compute_angle
               test/test_classical_windmill_compute_angle.cpp)
target_link_libraries(test_classical_windmill_compute_angle
                      windmill
                      camera)

add_executable(test_classical_windmill_compute_angles
               test/test_classical_windmill_compute_angles.cpp)
target_link_libraries(test_classical_windmill_compute_angles
                      windmill
                      camera)

add_executable(test_classical_windmill_detect_fans
               test/test_classical_windmill_detect_fans.cpp)
target_link_libraries(test_classical_windmill_detect_fans
                      windmill
                      camera)

add_executable(test_classical_windmill_detect_logos
               test/test_classical_windmill_detect_logos.cpp)
target_link_libraries(test_classical_windmill_detect_logos
                      windmill
                      camera)

add_executable(test_classical_windmill_detect_objects
               test/test_classical_windmill_detect_objects.cpp)
target_link_libraries(test_classical_windmill_detect_objects
                      windmill
                      camera)

add_executable(test_classical_windmill_preprocess_image
               test/test_classical_windmill_preprocess_image.cpp)
target_link_libraries(test_classical_windmill_preprocess_image
                      windmill
                      camera)

add_executable(test_classical_windmill_recoginzer_param
               test/test_classical_windmill_recoginzer_param.cpp)
target_link_libraries(test_classical_windmill_recoginzer_param
                      windmill)

add_executable(test_classical_windmill_predictor_AngleRoll
        test/test_classical_windmill_predictor_AngleRoll.cpp)
target_link_libraries(test_classical_windmill_predictor_AngleRoll
        windmill
        solver
        camera
        ${CERES_LIBRARIES})


add_executable(test_imag
        test/test_imag.cpp)
target_link_libraries(test_imag
        windmill
        camera
        solver
        ${CERES_LIBRARIES})


add_executable(test_classical_windmill_predictor
               test/test_classical_windmill_predictor.cpp)
target_link_libraries(test_classical_windmill_predictor
                    windmill
                     camera)