# stopwatch

## Project Features

Our stopwatch is designed to provide precise time measurement with the following functionalities:

- **Lap Recording**: Capture individual lap or segment times for detailed analysis.
- **User Interface**: Simple button-based control allowing actions like start, stop, and saving time measurements.
- **LCD Display**: The current time and lap information are displayed on a clear and readable LCD.
- **Optimized Performance**: Focused on precise timing and efficient use of the STM32F429 microcontroller's resources.

## Project Structure

### Includes Section:
- Imports various libraries including standard C libraries (`stdio.h`, `stdlib.h`) and STM32F429 specific libraries (`stm32f429i_discovery_lcd.h`, `main.h`, etc.).

### Private Variables Section:
- Defines variables used in the program, such as:
  - Time values: `seconds`, `minutes`
  - Auxiliary variables: `privateTick`, `globalTick`
  - Text buffers for the LCD display: `str_curTime`, `str_lapTime1`, `str_lapTime2`, `str_lapTime3`

### Main Function (`main()`):
- Initializes microcontroller peripherals (communication interfaces, timers, LCD display, etc.).
- Configures the system clock.
- Contains the core logic of the stopwatch within an infinite `while(1)` loop:
  - Handles timing conditions (`globalSpace`, `globalClock`).
  - Updates time values (`curTime`, `lapTime1`, `lapTime2`, `lapTime3`).
  - Utilizes LCD display functions such as:
    - Drawing shapes: `BSP_LCD_DrawCircle`
    - Displaying text: `BSP_LCD_DisplayStringAtLine`
    - Drawing clock hands: `BSP_LCD_DrawLine`
  - Depending on `globalSpace` and `globalClock`, it shows various time and stopwatch information on the LCD.

### Helper Functions:
- **`SystemClock_Config()`**: Configures the system clock.
- **`MX_DMA2D_Init()`**: Initializes the DMA2D controller.
- **`MX_FMC_Init()`**: Initializes the external memory controller.
- **`MX_I2C3_Init()`**: Sets up the I2C interface.

These helper functions initialize various peripherals like DMA controllers, communication interfaces, RAM, clocks, and more.

## Summary
This code runs in a continuous loop, updating the display based on the current time and stopwatch status. It ensures smooth performance and accurate timing through the use of peripherals on the STM32F429 platform.
