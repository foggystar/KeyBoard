# Teaching notes

- The user prefers STM32CubeMX plus the official STM32 VS Code extension for normal CPU1 development and does not want to use the CubeProgrammer GUI for ordinary application flashing.
- The current board has HSE enabled and has reached the line after `SystemClock_Config()`, proving that HSE can reach ready state with the installed 12 pF crystal.
- Treat LSE as unavailable for the current board unless the user later confirms that a 32.768 kHz source is populated.
- Teach bring-up in this order: CPU2 stack -> advertising -> connection -> GATT write/notify -> BLE HID -> low power.
- The user explicitly reports no prior STM32 Bluetooth experience. Start from Peripheral/Central, advertising, connection, GATT service/characteristic, then map those concepts to generated STM32WB files and functions.
- Prefer one observable result per lesson and use breakpoints as the feedback mechanism before introducing custom BLE APIs.
- CubeMX 6.18.1 generated `HAL_RTC_MspInit()` with HSE/32 but without an unconditional `HAL_PWR_EnableBkUpAccess()`. On a system reset that retains RTCSEL but re-protects the backup domain, `RTC_EnterInitMode()` times out. Keep the manual call in the RTC MSP user-code block.
