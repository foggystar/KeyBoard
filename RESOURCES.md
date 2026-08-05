# STM32WB55 Bluetooth LE Resources

## Knowledge

- [Bluetooth SIG: The Bluetooth Low Energy Primer](https://www.bluetooth.com/bluetooth-le-primer/)
  Bluetooth SIG's official orientation guide. Use for: Peripheral/Central roles, advertising, connections, GAP, GATT, services, and characteristics before studying STM32-specific APIs.
- [AN5289: How to build wireless applications with STM32WB MCUs](https://www.st.com/resource/en/application_note/an5289-how-to-build-wireless-applications-with-stm32wb-mcus-stmicroelectronics.pdf)
  ST's primary architecture and application guide. Use for: CPU1/CPU2 division, transport layer, BLE initialization, sequencer, timer server, and low-power integration.
- [RM0434: STM32WB55/35 reference manual](https://www.st.com/resource/en/reference_manual/dm00318631.pdf)
  ST's register-level authority. Use for: PWR backup-domain access, RCC_BDCR RTC clock selection, RTC initialization/status registers, and reset behavior.
- [STM32WB BLE - STM32CubeMX application conception](https://wiki.st.com/stm32mcu/wiki/Connectivity%3ASTM32WB_BLE_STM32CubeMX)
  Official CubeMX walkthrough. Use for: HSEM, IPCC, RTC, RF, STM32_WPAN, P2P Server, and generated project structure.
- [STM32WB BLE hardware setup](https://wiki.st.com/stm32mcu/wiki/Connectivity%3ASTM32WB_BLE_Hardware_Setup)
  Official dual-core and CPU2 wireless-stack overview. Use for: understanding why the encrypted wireless binary is separate from the CPU1 application.
- Local official example: `C:\Users\FoggyStar\STM32Cube\Repository\STM32Cube_FW_WB_V1.24.0\Projects\P-NUCLEO-WB55.Nucleo\Applications\BLE\BLE_p2pServer`
  Version-matched source of truth for generated `main.c`, `app_entry.c`, `app_ble.c`, and P2P service behavior.
- Local official HID example: `C:\Users\FoggyStar\STM32Cube\Repository\STM32Cube_FW_WB_V1.24.0\Projects\P-NUCLEO-WB55.Nucleo\Applications\BLE\BLE_Hid`
  Version-matched source of truth for STM32WB55 HIDS, BAS, DIS, bonding, advertising, and report-map integration. Its application example is mouse-oriented and has no `.ioc`, so reuse its service integration rather than its board-specific clock and GPIO code.
- Local CPU2 release notes: `C:\Users\FoggyStar\STM32Cube\Repository\STM32Cube_FW_WB_V1.24.0\Projects\STM32WB_Copro_Wireless_Binaries\STM32WB5x\Release_Notes.html`
  Version-matched authority for FUS checks, wireless binary selection, install addresses, and CLI procedures. Follow it exactly if the CPU2 BLE stack is absent.
- [Bluetooth SIG: HID Over GATT Profile](https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-hogp/)
  Primary profile specification. Use for: mandatory HID/Battery/Device Information services, HID Device and Report Host roles, bonding, and interoperable behavior.
- [USB-IF: Human Interface Devices specifications and tools](https://www.usb.org/hid)
  Primary source for HID report descriptors and keyboard/keypad usage codes. BLE HOGP reuses this HID report language.

## Wisdom (Communities)

- [ST: STM32WB RTC initialization timeout caused by backup-domain protection](https://community.st.com/stm32-mcus-embedded-software-32/init-rtc-failed-timeout-error-stm32wb10cc-123993)
  A matching failure diagnosed by ST staff. Use for: `RTC_EnterInitMode()` timeout where `HAL_PWR_EnableBkUpAccess()` is missing.
- [STMicroelectronics STM32 MCUs Wireless Community](https://community.st.com/t5/stm32-mcus-wireless/bd-p/mcu-wireless-forum)
  Use for: custom-board bring-up reports, no-LSE limitations, RF wake-up clock issues, and stack/FUS failures after official documentation has been checked.
