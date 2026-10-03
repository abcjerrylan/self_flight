add_executable(core_tests core_tests.cpp)
target_link_libraries(core_tests PRIVATE self_flight::core)
if(MSVC)
    target_compile_options(core_tests PRIVATE /W4 /WX /fp:precise)
else()
    target_compile_options(core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
foreach(test_case IN ITEMS
        vectors identity known_axes composition inverse_sign normalization
        normalization_failures axis_angle_failures rotation_failures
        time_intervals time_errors freshness data_contracts rotation_properties)
    add_test(NAME core.${test_case} COMMAND core_tests ${test_case})
endforeach()
