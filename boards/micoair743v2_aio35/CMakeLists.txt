set(GEN "${CMAKE_CURRENT_SOURCE_DIR}/generated/micoair743v2_aio35")
set(HAL "${GEN}/Drivers/STM32H7xx_HAL_Driver")
set(TX "${GEN}/Middlewares/ST/threadx")
set(UX "${GEN}/Middlewares/ST/usbx")

add_library(board_config INTERFACE)
target_compile_definitions(board_config INTERFACE USE_HAL_DRIVER STM32H743xx TX_INCLUDE_USER_DEFINE_FILE UX_INCLUDE_USER_DEFINE_FILE)
target_include_directories(board_config INTERFACE
    "${GEN}/Core/Inc"
    "${GEN}/AZURE_RTOS/App"
    "${HAL}/Inc"
    "${GEN}/Drivers/CMSIS/Include"
    "${GEN}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${TX}/common/inc"
    "${TX}/ports/cortex_m7/gnu/inc"
    "${GEN}/USBX/App"
    "${GEN}/USBX/Target"
    "${UX}/common/core/inc"
    "${UX}/common/usbx_device_classes/inc"
    "${UX}/common/usbx_stm32_device_controllers"
    "${UX}/ports/generic/inc"
)

add_library(stm32_hal STATIC
    "${HAL}/Src/stm32h7xx_hal.c"
    "${HAL}/Src/stm32h7xx_hal_cortex.c"
    "${HAL}/Src/stm32h7xx_hal_dma.c"
    "${HAL}/Src/stm32h7xx_hal_dma_ex.c"
    "${HAL}/Src/stm32h7xx_hal_exti.c"
    "${HAL}/Src/stm32h7xx_hal_flash.c"
    "${HAL}/Src/stm32h7xx_hal_flash_ex.c"
    "${HAL}/Src/stm32h7xx_hal_gpio.c"
    "${HAL}/Src/stm32h7xx_hal_hsem.c"
    "${HAL}/Src/stm32h7xx_hal_i2c.c"
    "${HAL}/Src/stm32h7xx_hal_i2c_ex.c"
    "${HAL}/Src/stm32h7xx_hal_mdma.c"
    "${HAL}/Src/stm32h7xx_hal_pwr.c"
    "${HAL}/Src/stm32h7xx_hal_pwr_ex.c"
    "${HAL}/Src/stm32h7xx_hal_rcc.c"
    "${HAL}/Src/stm32h7xx_hal_rcc_ex.c"
    "${HAL}/Src/stm32h7xx_hal_tim.c"
    "${HAL}/Src/stm32h7xx_hal_tim_ex.c"
    "${HAL}/Src/stm32h7xx_hal_spi.c"
    "${HAL}/Src/stm32h7xx_hal_spi_ex.c"
    "${HAL}/Src/stm32h7xx_hal_uart.c"
    "${HAL}/Src/stm32h7xx_hal_uart_ex.c"
    "${HAL}/Src/stm32h7xx_hal_pcd.c"
    "${HAL}/Src/stm32h7xx_hal_pcd_ex.c"
    "${HAL}/Src/stm32h7xx_ll_usb.c"
)
target_link_libraries(stm32_hal PUBLIC board_config)

include(threadx-sources.cmake)
add_library(threadx STATIC ${THREADX_SOURCES})
target_link_libraries(threadx PUBLIC board_config)

include(usbx-sources.cmake)
# Include USBX's HAL callbacks directly so HAL's weak defaults cannot hide them.
add_library(usbx OBJECT ${USBX_SOURCES})
target_link_libraries(usbx PUBLIC board_config)

add_executable(self_flight
    "${GEN}/Core/Startup/startup_stm32h743vihx.s"
    "${GEN}/Core/Src/main.c"
    "${GEN}/Core/Src/system_stm32h7xx.c"
    "${GEN}/Core/Src/stm32h7xx_hal_msp.c"
    "${GEN}/Core/Src/stm32h7xx_hal_timebase_tim.c"
    "${GEN}/Core/Src/stm32h7xx_it.c"
    "${GEN}/Core/Src/syscalls.c"
    "${GEN}/Core/Src/sysmem.c"
    "${GEN}/Core/Src/app_threadx.c"
    "${GEN}/Core/Src/tx_initialize_low_level.S"
    "${GEN}/AZURE_RTOS/App/app_azure_rtos.c"
    "${GEN}/USBX/App/app_usbx_device.c"
    "${GEN}/USBX/App/ux_device_cdc_acm.c"
    "${GEN}/USBX/App/ux_device_descriptors.c"
    "${PROJECT_SOURCE_DIR}/app/app.cpp"
    "${PROJECT_SOURCE_DIR}/platform/stm32h7/platform.cpp"
    "${PROJECT_SOURCE_DIR}/flight/services/imu.cpp"
    "${PROJECT_SOURCE_DIR}/flight/services/attitude.cpp"
    "${PROJECT_SOURCE_DIR}/flight/services/flight.cpp"
    "${PROJECT_SOURCE_DIR}/flight/services/rc.cpp"
)
set_target_properties(self_flight PROPERTIES SUFFIX ".elf" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_include_directories(self_flight PRIVATE "${PROJECT_SOURCE_DIR}/app" "${PROJECT_SOURCE_DIR}/platform/stm32h7"
    "${PROJECT_SOURCE_DIR}/flight/services" "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(self_flight PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Werror>)
target_link_libraries(self_flight PRIVATE self_flight::core bmi088 dshot sbus usbx stm32_hal threadx m)
set(LINKER_SCRIPT "${GEN}/STM32CubeIDE/STM32H743VIHX_FLASH.ld")
target_link_options(self_flight PRIVATE
    "-T${LINKER_SCRIPT}"
    "-Wl,--defsym=__Vectors=g_pfnVectors"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/self_flight.map"
    "-Wl,--print-memory-usage"
)
set_property(TARGET self_flight APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
add_custom_command(TARGET self_flight POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:self_flight> "${CMAKE_BINARY_DIR}/self_flight.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:self_flight> "${CMAKE_BINARY_DIR}/self_flight.bin"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:self_flight>
    VERBATIM
)
