# Mission: Bring up Bluetooth LE on the STM32WB55 keyboard

## Why
Prove that the custom STM32WB55CGU6 keyboard board can advertise, connect to a phone, and exchange GATT data before building the final Bluetooth HID keyboard firmware.

## Success looks like
- Confirm that the 32 MHz HSE and the CPU2 wireless stack both initialize successfully.
- Discover the board from a phone as a BLE peripheral and keep a stable connection.
- Write and receive one test value through a generated P2P GATT service.
- Progress from the P2P smoke test to the BLE HID keyboard example.

## Constraints
- Custom UFQFPN48 board using a 32 MHz crystal marked with 12 pF load capacitance.
- Current hardware/configuration has no active LSE, so the first BLE test must use HSE/1024 and avoid deep low-power modes.
- CPU1 application development uses STM32CubeMX, CMake, VS Code, ST-LINK, and SWD.
- Keep SWD and NRST recoverable throughout clock and wireless testing.

## Out of scope
- Bluetooth qualification, RF certification, antenna matching certification, and final power optimization.
- Final keyboard HID report design until advertising, connection, and P2P data transfer work reliably.
