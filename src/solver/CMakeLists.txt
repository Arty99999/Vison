add_library(solver SHARED
            src/armor_distance_offset.cpp
            src/armor_distance_param.cpp
            src/armor_velocity_param.cpp
            src/buff_distance_offset.cpp
            src/buff_distance_param.cpp
            src/large_buff_velocity_param.cpp
            src/small_buff_velocity_param.cpp
            src/solver.cpp
            src/solver_param.cpp
            src/angle_distance_offset_param.cpp
            src/predictor_KF.cpp
            src/predictor_EKF.cpp
            src/predictor_model.cpp
            src/predictor_tool.cpp
            src/predictor.cpp
            src/predictor_param.cpp
            src/predictor_command.cpp
            src/predictor_AntiTop.cpp)
add_library(predictor SHARED
        src/predictor_command.cpp
        src/predictor_EKF.cpp
)


target_link_libraries(solver
                      camera
                      armor
#                      common
                      logger
                      ${OpenCV_LIBS}
                      ${CERES_LIBRARIES}
                      uart)

add_executable(test_ceres_fit_curve
               test/test_ceres_fit_curve.cpp)
target_link_libraries(test_ceres_fit_curve
                      ${CERES_LIBRARIES})

add_executable(test_ceres_optimization
               test/test_ceres_optimization.cpp)
target_link_libraries(test_ceres_optimization
                      ${CERES_LIBRARIES})

add_executable(test_compute_vector_angle
               test/test_compute_vector_angle.cpp)

add_executable(test_eigen_rotate_vector
               test/test_eigen_rotate_vector.cpp)

add_executable(test_solver_compensate_distance
               test/test_solver_compensate_distance.cpp)
target_link_libraries(test_solver_compensate_distance
                      solver)

add_executable(test_solver_fit_polynomial
               test/test_solver_fit_polynomial.cpp)
target_link_libraries(test_solver_fit_polynomial
                      solver)

add_executable(test_solver_fit_sine
               test/test_solver_fit_sine.cpp)
target_link_libraries(test_solver_fit_sine
                      solver)

add_executable(test_solver_fit_sine_integral
               test/test_solver_fit_sine_integral.cpp)
target_link_libraries(test_solver_fit_sine_integral
                      solver)

add_executable(test_solver_param
               test/test_solver_param.cpp)
target_link_libraries(test_solver_param
                      solver)

add_executable(test_predictor_model
               test/test_predictor_model.cpp)
target_link_libraries(test_predictor_model
                      solver)

add_executable(test_predictor_param
               test/test_predictor_param.cpp)
target_link_libraries(test_predictor_param
                      solver)

#add_executable(test_predictor_autoaim_ekf
#               test/test_predictor_autoaim_ekf.cpp
#               test/test_predictor_tool.cpp
#               test/test_package.cpp
#               test/test_predictor_commond.cpp
#        test/test_predictor_antitop.cpp
#        test/test_predictor_yaw.cpp
#        test/test_ekf.cpp)
#target_link_libraries(test_predictor_autoaim_ekf
#                      solver)

add_executable(test_package
               test/test_package.cpp)
target_link_libraries(test_package
                      solver)

add_executable(test_predictor_tool
               test/test_predictor_tool.cpp)
target_link_libraries(test_predictor_tool
                      solver)

add_executable(test_predictor_commond
               test/test_predictor_commond.cpp)
target_link_libraries(test_predictor_commond
                      solver
                      brain)

add_executable(test_predictor_antitop
        test/test_predictor_antitop.cpp)
target_link_libraries(test_predictor_antitop
        solver)

add_executable(test_predictor_yaw
        test/test_predictor_yaw.cpp)
target_link_libraries(test_predictor_yaw
        solver)

add_executable(test_ekf
        test/test_ekf.cpp)
target_link_libraries(test_ekf
        solver)