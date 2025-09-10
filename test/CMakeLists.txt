add_executable(wheel_test wheel_test.c)
target_link_libraries(wheel_test PRIVATE mecanum)
target_compile_features(wheel_test PRIVATE c_std_11)

add_executable(encoder_test encoder_test.c)
target_link_libraries(encoder_test PRIVATE mecanum)
target_compile_features(encoder_test PRIVATE c_std_11)
