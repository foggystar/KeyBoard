# Wireless stack and phone connection confirmed

The STM32WB55CGU6 was recovered from FUS-only operation, upgraded from FUS v1.2.0 to FUS v2.2.0, and programmed with `stm32wb5x_BLE_Stack_full_fw.bin` v1.24.0 at the official 1 MB-device address `0x080D0000`. The CPU1 application was then reflashed and a phone connected successfully to its P2P peripheral.

This confirms the CPU2 wireless firmware, CPU1/CPU2 transport, advertising, connection, RF path, and current HSE configuration work under the tested conditions. P2P characteristic read/write instructions were provided, but a completed read-back result has not yet been reported.

The next learning target is a minimal BLE HID keyboard: bond from the operating system Bluetooth settings, subscribe to one input report, and emit exactly one `A` press followed by an all-zero release report.
