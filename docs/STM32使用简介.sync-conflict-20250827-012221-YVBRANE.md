好的，根据您提供的STM32F103x8/xB数据手册，我将为您详细解释如何烧录程序，包括最小系统板设计、不同烧录方式以及软件开发。

STM32F103x8/xB是基于ARM Cortex-M3核的微控制器，具有丰富的片上资源。

---

### 一、最小系统板设计

一个STM32F103x8/xB的最小系统板至少需要满足以下几点，以保证MCU能够正常启动和运行：

1.  **电源部分：**
    *   **主电源 (VDD/VSS)：** 数据手册2.3.9节和图14显示，VDD电压范围为2.0V至3.6V。需要连接VDD引脚（通常有多个）到电源正极，VSS引脚（通常有多个）到地。
    *   **模拟电源 (VDDA/VSSA)：** 数据手册2.3.9节提到，VDDA和VSSA是ADC、复位块、RCs和PLL的模拟电源，范围为2.0V至3.6V（当使用ADC时，VDDA最低2.4V）。通常，VDDA应与VDD连接，VSSA与VSS连接，并使用高质量的去耦电容（如图39所示，1µF并联10nF）进行滤波，放置在靠近芯片引脚的位置。
    *   **RTC及备份域电源 (VBAT)：** 数据手册2.3.9节提到，VBAT电压范围为1.8V至3.6V，用于RTC和备份寄存器供电。可以连接一个纽扣电池（如CR2032）或直接连接到VDD，通过一个二极管防止VDD断电时VBAT回灌。
    *   **去耦电容：** 所有VDD和VDDA引脚附近都需要放置100nF和4.7µF（或1µF）的去耦电容，靠近芯片引脚。数据手册图14、图39、图40有详细说明。

2.  **时钟部分：**
    *   **高速外部时钟 (HSE)：** 数据手册2.3.7节和图24建议使用4-16MHz的晶体振荡器（常用8MHz）连接到OSC_IN和OSC_OUT引脚。需要匹配的负载电容（CL1和CL2，通常为10-30pF，根据晶振规格选择）和串联电阻（REXT，通常不使用或小阻值）。这是CPU和USB等高速外设的首选时钟源。
    *   **低速外部时钟 (LSE，可选)：** 数据手册2.3.7节和图25建议使用32.768kHz的晶体振荡器连接到OSC32_IN和OSC32_OUT引脚，用于RTC。同样需要负载电容。如果不需要RTC功能，可以不使用。
    *   **内部时钟：** 芯片内部有8MHz高速RC振荡器(HSI)和40kHz低速RC振荡器(LSI)，在没有外部晶振时可作为备用时钟源。但为了性能和精度，通常推荐使用外部晶振。

3.  **复位部分 (NRST)：**
    *   NRST引脚是外部复位引脚。数据手册图31显示了推荐的复位电路：一个100nF的电容连接到地，一个10kΩ的电阻连接到VDD。可以再串联一个按钮到地，用于手动复位。

4.  **启动模式选择 (BOOT0/BOOT1)：**
    *   数据手册2.3.8节提到，MCU有三种启动模式，通过BOOT0和BOOT1引脚的电平状态选择。
        *   **主闪存启动 (User Flash Memory)：** BOOT0 = 0 (下拉)，BOOT1 = 0 (下拉)。这是最常用的模式，MCU从用户编写的Flash程序启动。
        *   **系统存储器启动 (System Memory)：** BOOT0 = 1 (上拉)，BOOT1 = 0 (下拉)。MCU从内置的Bootloader程序启动，用于通过串行接口（如USART）进行程序烧录。
        *   **SRAM启动：** BOOT0 = 1 (上拉)，BOOT1 = 1 (上拉)。MCU从SRAM启动，通常用于调试。
    *   建议在BOOT0引脚连接一个下拉电阻（如10kΩ）和一个上拉按钮，以便在需要进入Bootloader模式时切换。BOOT1引脚通常直接接地（下拉）。

5.  **调试/烧录接口：**
    *   STM32F103x8/xB支持串行线调试(SWD)和JTAG接口（数据手册2.3.24节）。SWD接口更简洁，通常只需要SWDIO、SWCLK、NRST、VDD和GND五个引脚。建议预留一个标准的2x5或2x10针排针接口，用于连接ST-Link或其他调试器。

6.  **USB接口 (可选，但推荐用于烧录)：**
    *   数据手册2.3.20节提到支持USB 2.0全速接口。需要连接USB_DP和USB_DM引脚。
    *   **Type-C接口考虑：** Type-C只是物理接口。对于STM32F103，它本身是USB 2.0全速设备。要使用Type-C连接器，除了D+/D-和VBUS/GND外，还需要在Type-C连接器的CC1和CC2引脚上连接5.1kΩ的下拉电阻到地，以指示它是一个DFP（下行端口，即设备）。VBUS引脚上通常需要一个TVS二极管进行ESD保护。

---

### 二、烧录程序的方式

STM32F103x8/xB有多种烧录方式：

#### 1. 通过调试器 (ST-Link/J-Link) 烧录

这是最常用和推荐的烧录方式，尤其在开发阶段。

*   **所需硬件：**
    *   **ST-Link/V2/V3 (或兼容调试器)：** ST官方的调试器，性价比高。
    *   **J-Link (或兼容调试器)：** Segger公司的调试器，功能强大，支持范围广。
    *   **连接线：** 将调试器与最小系统板的SWD接口连接（SWDIO, SWCLK, NRST, VDD, GND）。

*   **所需软件：**
    *   **STM32CubeIDE (推荐)：** ST官方的集成开发环境，内置调试器支持。
    *   **Keil MDK-ARM：** 业界常用的ARM开发工具，集成度高，调试功能强大。
    *   **IAR Embedded Workbench for ARM：** 另一款流行的ARM开发工具，编译优化出色。
    *   **STM32CubeProgrammer：** ST官方的通用烧录工具，支持多种接口（包括ST-Link）。
    *   **USB驱动：** 确保您的电脑安装了ST-Link或J-Link的USB驱动。

*   **烧录流程：**
    1.  将调试器通过USB连接到电脑。
    2.  将调试器通过SWD接口连接到STM32最小系统板。
    3.  确保最小系统板已上电。
    4.  在STM32CubeIDE、Keil MDK或IAR中，完成代码编译，生成`.hex`或`.bin`文件。
    5.  选择“Download”或“Debug”功能，IDE会自动调用内置的烧录工具将程序下载到STM32的Flash中。
    6.  如果使用STM32CubeProgrammer，打开软件，选择ST-Link/J-Link接口，点击“Connect”，然后选择要烧录的`.hex`或`.bin`文件，点击“Download”即可。

#### 2. 通过内置Bootloader (USB DFU 或 UART) 烧录

STM32F103x8/xB芯片内部固化了一个Bootloader程序在System Memory中。通过设置BOOT0/BOOT1引脚，可以进入这个Bootloader模式，然后通过USB或UART接口进行烧录。

*   **通过USB DFU (Device Firmware Upgrade) 烧录：**
    *   **所需硬件：**
        *   **最小系统板：** 带有USB_DP/USB_DM引脚连接到USB Type-C接口。
        *   **USB Type-C数据线：** 连接电脑和最小系统板。
    *   **所需软件：**
        *   **STM32CubeProgrammer (推荐)：** ST官方的烧录工具。
        *   **Zadig (Windows下，如果需要)：** 用于安装DFU设备的WinUSB驱动。
        *   **dfu-util (Linux/macOS下)：** 开源的DFU工具。
    *   **烧录流程：**
        1.  将BOOT0引脚拉高（上拉到VDD），BOOT1引脚拉低（下拉到GND）。
        2.  给最小系统板上电或复位，使其进入USB DFU模式。
        3.  通过USB Type-C数据线将最小系统板连接到电脑。
        4.  电脑应识别出一个DFU设备（可能需要手动安装驱动，使用Zadig工具将DFU设备驱动改为WinUSB）。
        5.  打开STM32CubeProgrammer，选择“USB”接口，点击“Connect”。
        6.  连接成功后，选择要烧录的`.hex`或`.bin`文件，点击“Download”。
        7.  烧录完成后，将BOOT0引脚拉低，复位芯片，程序将从Flash启动。

*   **通过UART (串行) 烧录：**
    *   **所需硬件：**
        *   **最小系统板：** 确保USART1的TX (PA9) 和RX (PA10) 引脚可用。
        *   **USB转TTL模块 (USB-to-UART)：** 如FT232RL、CH340G、CP2102等。
        *   **连接线：** USB转TTL模块的TX连接STM32的PA10 (RX)，RX连接STM32的PA9 (TX)，GND连接GND。
    *   **所需软件：**
        *   **STM32CubeProgrammer (推荐)：** ST官方的烧录工具。
        *   **Flash Loader Demonstrator (旧版ST工具)：** 也可以用于UART烧录。
    *   **烧录流程：**
        1.  将BOOT0引脚拉高（上拉到VDD），BOOT1引脚拉低（下拉到GND）。
        2.  给最小系统板上电或复位，使其进入串行Bootloader模式。
        3.  将USB转TTL模块连接到电脑，并连接到最小系统板的USART1引脚。
        4.  打开STM32CubeProgrammer，选择“UART”接口，选择正确的COM端口和波特率（通常是115200或更高），点击“Connect”。
        5.  连接成功后，选择要烧录的`.hex`或`.bin`文件，点击“Download”。
        6.  烧录完成后，将BOOT0引脚拉低，复位芯片，程序将从Flash启动。

---

### 三、如何编写程序

编写STM32F103x8/xB的程序通常遵循嵌入式C/C++开发流程。

*   **1. 开发环境选择：**
    *   **STM32CubeIDE (推荐，免费)：** 这是ST官方推荐的集成开发环境，基于Eclipse，集成了GNU GCC工具链、STM32CubeMX配置工具和ST-Link调试器驱动。它提供了一站式的开发体验，从项目创建、代码生成、编译到调试烧录。
    *   **Keil MDK-ARM (付费，有免费评估版限制)：** 经典的ARM开发工具，拥有强大的编译器和调试器。
    *   **IAR Embedded Workbench for ARM (付费，有免费评估版限制)：** 另一款高性能的开发工具，以其优秀的编译优化而闻名。
    *   **VS Code + PlatformIO/Cortex-Debug (免费，灵活)：** 如果您喜欢轻量级和可定制的环境，可以使用VS Code配合PlatformIO或Cortex-Debug扩展，集成GCC工具链。

*   **2. 代码生成与配置 (STM32CubeMX)：**
    *   无论您选择哪个IDE，强烈建议使用ST的图形化配置工具 **STM32CubeMX** (已集成到STM32CubeIDE中，也可独立下载)。
    *   **流程：**
        1.  在STM32CubeMX中选择您的具体芯片型号（如STM32F103C8Tx）。
        2.  通过图形界面配置引脚功能（GPIO、USART、SPI、USB等），时钟树（选择外部晶振、PLL倍频等），以及外设参数。
        3.  STM32CubeMX会根据您的配置生成初始化代码（C语言），包括HAL（硬件抽象层）或LL（底层库）库的调用。这些库极大地简化了外设的配置和使用。
        4.  选择您使用的IDE（如STM32CubeIDE、MDK-ARM、IAR）生成项目文件。

*   **3. 编写应用程序：**
    *   在生成的项目中，您可以在`main.c`文件或创建的其他源文件中编写您的应用程序逻辑。
    *   **例如，一个简单的LED闪烁程序：**
        ```c
        /* Private includes ----------------------------------------------------------*/
        #include "main.h" // 包含CubeMX生成的头文件
        #include "stm32f1xx_hal.h" // 包含HAL库头文件
        
        /* Private variables ---------------------------------------------------------*/
        // HAL库句柄或其他变量定义
        
        /* Private function prototypes -----------------------------------------------*/
        void SystemClock_Config(void); // 系统时钟配置函数，由CubeMX生成
        
        /* USER CODE BEGIN PFP */
        // 用户自定义函数原型
        /* USER CODE END PFP */
        
        /**
          * @brief  The application entry point.
          * @retval int
          */
        int main(void)
        {
          /* HAL_Init() must be called to initialize the HAL Library */
          HAL_Init();
        
          /* Configure the system clock */
          SystemClock_Config();
        
          /* USER CODE BEGIN SysInit */
          // 用户自定义系统初始化
          /* USER CODE END SysInit */
        
          /* Initialize all configured peripherals */
          // GPIO、USART等外设的初始化函数，由CubeMX生成
          MX_GPIO_Init(); 
          // ... 其他外设初始化
        
          /* USER CODE BEGIN 2 */
          // 用户自定义初始化代码
          /* USER CODE END 2 */
        
          /* Infinite loop */
          while (1)
          {
            /* USER CODE BEGIN WHILE */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 假设LED连接在GPIOC的PIN13
            HAL_Delay(500); // 延时500毫秒
            /* USER CODE END WHILE */
          }
        }
        
        /* USER CODE BEGIN 4 */
        // 用户自定义函数实现
        /* USER CODE END 4 */
        ```
    *   **编程语言：** 主要是C语言，也可以结合C++。
    *   **库函数：** 充分利用STM32CubeF1提供的HAL或LL库函数来操作外设，避免直接操作寄存器，提高开发效率和代码可读性。

*   **4. 编译与调试：**
    *   在IDE中点击“Build”按钮编译项目。
    *   连接调试器，点击“Debug”按钮，程序会下载到芯片中并开始调试。您可以在代码中设置断点、单步执行、查看变量值等。

---

**总结：**

对于STM32F103x8/xB的开发，推荐的路径是：
1.  **硬件：** 设计一个包含电源、时钟、复位、启动模式选择和SWD/USB接口的最小系统板。
2.  **软件配置：** 使用STM32CubeMX图形化配置芯片，生成初始化代码。
3.  **代码编写：** 在STM32CubeIDE (或Keil/IAR) 中基于HAL/LL库编写应用程序逻辑。
4.  **烧录与调试：** 使用ST-Link调试器通过SWD接口进行烧录和在线调试，或者在生产阶段使用USB DFU或UART Bootloader进行批量烧录。

通过遵循这些步骤，您可以有效地开发和烧录STM32F103x8/xB微控制器上的应用程序。









对于STM32F103x8/xB，当使用8MHz晶振时，**无源晶振 (Passive Crystal)** 是更常用、更推荐的选择。

下面我将详细解释两者的区别、优缺点，以及为何无源晶振更适合STM32F103：

### 1. 无源晶振 (Passive Crystal)

*   **特点：** 只有两个引脚。它本身不产生振荡，需要外部的振荡电路来驱动它产生稳定的频率。STM32F103芯片内部集成了这样的振荡电路（HSE振荡器）。
*   **连接方式：**
    *   8MHz无源晶振连接到STM32的 **OSC_IN** 和 **OSC_OUT** 引脚。
    *   还需要两个 **匹配的负载电容 (Load Capacitors)**（通常为10pF到30pF，具体值根据晶振规格书和PCB寄生电容确定）连接在晶振引脚和地之间。数据手册图24和ST的应用笔记AN2867对此有详细说明。
    *   可能还需要一个串联电阻（REXT），但通常对于8MHz晶振，这个电阻很小甚至可以省略。
*   **优点：**
    *   **成本较低：** 通常比有源晶振便宜。
    *   **功耗较低：** 由于振荡电路集成在MCU内部，ST对这个电路进行了优化，通常功耗更低。
    *   **引脚数量少：** 晶振本身只有两个引脚，PCB布局相对简洁。
    *   **符合设计惯例：** STM32F103的HSE模块就是为无源晶振设计的。
*   **缺点：**
    *   **对PCB布局敏感：** 晶振和负载电容的走线必须短而直，并远离噪声源，以避免引入寄生电容和噪声，影响振荡稳定性。
    *   **需要精确匹配负载电容：** 负载电容的选择至关重要，不匹配会导致频率偏差或振荡不稳定。ST的AN2867应用笔记提供了详细的计算和布局指南。
    *   **启动时间可能稍长：** 芯片内部振荡电路需要一定时间才能稳定。

### 2. 有源晶振 (Active Oscillator / Crystal Oscillator - XO)

*   **特点：** 通常有四个引脚（电源VCC、地GND、输出OUT、使能OE/NC）。它内部已经集成了晶体和振荡电路，直接输出一个稳定的方波时钟信号。
*   **连接方式：**
    *   8MHz有源晶振的输出引脚连接到STM32的 **OSC_IN** 引脚（或任何可以配置为外部时钟输入功能的GPIO引脚）。
    *   **OSC_OUT** 引脚保持浮空或根据芯片配置进行处理。
    *   在STM32CubeMX中，这对应于HSE时钟源选择“Bypass Clock Source”（旁路时钟源）。
    *   需要为有源晶振提供独立的电源和地。
*   **优点：**
    *   **设计简单：** 无需外部负载电容，只需连接电源、地和输出信号。
    *   **频率精度和稳定性高：** 内部振荡电路与晶体完美匹配，受外部环境影响小。
    *   **抗干扰能力强：** 输出的是已稳定的时钟信号，对PCB走线的要求相对较低（但仍需注意信号完整性）。
    *   **启动速度快：** 通常比无源晶振启动更快。
*   **缺点：**
    *   **成本较高：** 内部集成度更高，所以价格更贵。
    *   **功耗可能较高：** 内部的振荡电路会持续消耗电能。
    *   **占用更多引脚：** 通常需要电源、地和输出共3个引脚，如果带使能功能则是4个引脚。

### 结论与建议

对于STM32F103x8/xB，如果使用8MHz晶振，**强烈推荐使用无源晶振**：

1.  **官方设计意图：** STM32F103的HSE模块就是为无源晶振设计的，使用无源晶振是充分利用芯片内部硬件资源、实现最佳性能和最低功耗的途径。
2.  **USB要求：** 如果您的设计需要使用USB功能，USB模块需要一个精确的48MHz时钟，这个时钟通常是通过PLL从8MHz HSE晶振倍频而来。使用无源晶振配合内部PLL是官方推荐且最可靠的方案。
3.  **成本考量：** 无源晶振更经济实惠。

**注意事项：**

*   **PCB布局：** 无论选择哪种晶振，时钟信号都是敏感信号。对于无源晶振，务必遵循ST的AN2867应用笔记中的晶振布局指南，确保晶振和负载电容紧邻芯片，走线短而粗，并使用地平面进行屏蔽。
*   **负载电容：** 负载电容的精确选择和匹配是无源晶振稳定工作的关键。请务必参考您所选晶振的数据手册，并根据AN2867进行调整。

综上所述，除非有特殊需求（例如极高的频率精度要求，且愿意承担更高的成本和功耗），否则对于STM32F103x8/xB，选择 **8MHz无源晶振** 是最普遍和稳妥的方案。