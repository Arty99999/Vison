add_executable(cubot_armor_distance_compensator
               src/cubot_armor_distance_compensator.cpp)
target_link_libraries(cubot_armor_distance_compensator
                      brain)

add_executable(cubot_armor_image_extractor
               src/cubot_armor_image_extractor.cpp)
target_link_libraries(cubot_armor_image_extractor
                      armor
                      camera)

add_executable(cubot_armor_negative_generator
               src/cubot_armor_negative_generator.cpp)
target_link_libraries(cubot_armor_negative_generator
                      armor
                      camera)

add_executable(cubot_armor_svm_trainer
               src/cubot_armor_svm_trainer.cpp)
target_link_libraries(cubot_armor_svm_trainer
                      armor)

add_executable(cubot_armor_threshold_estimator
               src/cubot_armor_threshold_estimator.cpp)
target_link_libraries(cubot_armor_threshold_estimator
                      camera
                      armor)

add_executable(cubot_brain
               src/cubot_brain.cpp)
target_link_libraries(cubot_brain
                      brain )

add_executable(cubot_camera_calibrator
               src/cubot_camera_calibrator.cpp)
target_link_libraries(cubot_camera_calibrator
                      ${OpenCV_LIBS}
                      logger)

add_executable(cubot_distance_polynomial_estimator
               src/cubot_distance_polynomial_estimator.cpp)
target_link_libraries(cubot_distance_polynomial_estimator
                      solver)

add_executable(cubot_fan_distance_compensator
               src/cubot_fan_distance_compensator.cpp)
target_link_libraries(cubot_fan_distance_compensator
                      brain)

add_executable(cubot_fan_palstance_estimator
               src/cubot_fan_palstance_estimator.cpp)
target_link_libraries(cubot_fan_palstance_estimator
                      windmill
                      solver
                      camera)

add_executable(cubot_fan_template_extractor
               src/cubot_fan_template_extractor.cpp)
target_link_libraries(cubot_fan_template_extractor
                      windmill
                      camera)

add_executable(cubot_fan_threshold_estimator
               src/cubot_fan_threshold_estimator.cpp)
target_link_libraries(cubot_fan_threshold_estimator
                      camera
                      windmill)

add_executable(cubot_hsv_threshold_estimator
               src/cubot_hsv_threshold_estimator.cpp)
target_link_libraries(cubot_hsv_threshold_estimator
                      camera
                      armor)

add_executable(cubot_logo_image_extractor
               src/cubot_logo_image_extractor.cpp)
target_link_libraries(cubot_logo_image_extractor
                      windmill
                      camera)

add_executable(cubot_logo_negative_generator
               src/cubot_logo_negative_generator.cpp)
target_link_libraries(cubot_logo_negative_generator
                      windmill
                      camera)

add_executable(cubot_logo_svm_trainer
               src/cubot_logo_svm_trainer.cpp)
target_link_libraries(cubot_logo_svm_trainer
                      windmill)

add_executable(cubot_logo_threshold_estimator
               src/cubot_logo_threshold_estimator.cpp)
target_link_libraries(cubot_logo_threshold_estimator
                      camera
                      windmill)

add_executable(cubot_video_transcriber
               src/cubot_video_converter.cpp)
target_link_libraries(cubot_video_transcriber
                      ${OpenCV_LIBS})

add_executable(cubot_video_recorder
               src/cubot_video_recorder.cpp)
target_link_libraries(cubot_video_recorder
                      camera)

add_executable(cubot_windmill_contours_estimator
               src/cubot_windmill_contours_estimator.cpp)
target_link_libraries(cubot_windmill_contours_estimator
                      windmill
                      camera)

add_executable(cubot_windmill_hsv_threshold_estimator
               src/cubot_windmill_hsv_threshold_estimator.cpp)
target_link_libraries(cubot_windmill_hsv_threshold_estimator
                      windmill
                      camera)