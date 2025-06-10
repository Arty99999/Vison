
enable_language(CUDA)
add_library(armor SHARED
        src/classical_armor.cpp
        src/classical_armor_solver.cpp
        src/classical_armor_evaluation_weight.cpp
        src/classical_armor_recognizer.cpp
        src/classical_armor_recognizer_param.cpp
        src/calibrator.cpp
        src/postprocess.cu
        src/preprocess.cu
)
target_link_libraries(armor
        ${OpenCV_LIBS}
        ${TENSORRT_LIB}
        ${CUDA_LIB}
        ${GLOG_LIB}
        ${CERES_LIBRARIES}
        camera
)

add_executable(test_classical_armor_recognizer_param
        test/test_classical_armor_recognizer_param.cpp)
target_link_libraries(test_classical_armor_recognizer_param
        armor
        camera)


add_executable(test_classical_armors_yolo
        test/test_classical_armors_yolo.cpp)
target_link_libraries(test_classical_armors_yolo
        armor camera
)

add_executable(test_classical_evalute_polished_armors
        test/test_classical_evaluate_polished_armors.cpp)
target_link_libraries(test_classical_evalute_polished_armors
        armor
        camera)