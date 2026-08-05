cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32WB55xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../STM32_WPAN/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities/lpm/tiny_lpm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/template
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/svc/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/svc/Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32WBxx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities/sequencer
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../STM32_WPAN/Target/hw_ipcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../STM32_WPAN/App/app_ble.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../STM32_WPAN/App/p2p_server_app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/app_entry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/app_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/hw_timerserver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32_lpm_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32wbxx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32wbxx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32wb55xx_cm4.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32wbxx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_ipcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_spi_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_usb.c
)

# Drivers Midllewares


set(Utilities_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities/lpm/tiny_lpm/stm32_lpm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities/sequencer/stm32_seq.c
)
set(STM32_WPAN_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/utilities/otp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/utilities/stm_list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/utilities/stm_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/template/osal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/svc/Src/p2p_stm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/hci_tl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/hci_tl_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_WPAN/ble/svc/Src/svc_ctl.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    Utilities	STM32_WPAN	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create Utilities static library
add_library(Utilities OBJECT)
target_sources(Utilities PRIVATE ${Utilities_Src})
target_link_libraries(Utilities PUBLIC stm32cubemx)

# Create STM32_WPAN static library
add_library(STM32_WPAN OBJECT)
target_sources(STM32_WPAN PRIVATE ${STM32_WPAN_Src})
target_link_libraries(STM32_WPAN PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
