# STM32 serial Myo firmware

Open `stm32/new-stm-code` as an existing STM32CubeIDE project and flash it to
the NUCLEO-F446RE.

Protocol on the ST-LINK virtual COM port (`/dev/ttyACM*`) is `115200 8N1`,
newline-terminated:

- `STOP` or `0`
- `RUN` or `1`
- `P 0..100`
- `STATUS` or `S`

Boot, malformed input, UART error, and a 3 s command timeout all force STOP:
`PA4 = 0 V`, `PC0 = LOW`, `LD2 = OFF`.
