# iEdge4.0 Driver&SDK Library
This is the iEdge4.0 IOT system's driver layer repository. All the codes in this repository can be used for Polysense iEdge4.0 platform. It supports: 
* LoRa module
* NBIOT module
* WIFI module
* Various I2C/RS232/ADC/RS485(MODBUS) sensor drivers in almost all IOT areas

Users can also develop their private sensor drivers based on iEdge4.0 platform. 

## Installation
The code architecture is C for ARM Cortex M0+. All drivers are maintained as a stand-alone Eclipse project format. For example, lora/bg95/sht30/.... are all a Eclipse project. If you're familiar with Eclipse IDE, please ignore this step.

SystemWorkbench for STM32 or STM32CubeIDE is preferred to be used. In this guide, we'll use STM32 CubeIDE. See below for detailed:

1. **Download and install STM32CubeIDE**
  https://www.st.com/content/st_com/en/stm32cubeide.html

2. **Start STM32 CubeIDE and create workspace**
  In this guide, we'll use /mnt/c/workspace 

3. **Git clone this library within this workspace**
```
  cd /mnt/c/workspace
  git clone https://github.com/PolysenseTech/iEdge4.0
```

## Projects Introduction
This driver library contains a series of offical Polysense sensor drivers which have been maintined by Polysense. Also, it contais several example sensor drivers for study or new sensor development. All these drivers are stored in sub folder - 4.0/src/drv, which will look like:
```
/mnt/c/workspace/iEdge4.0/4.0/src/drv$ ls
LinkerScript.ld  cnsl    co2z19   fastcnt  im1281  jc     mtepd   myuart  oiw    pm      sga457  src    vbat
a780             cntr12  coil     fs217    im948   lora   mtlcd   n306    oven   pt100   sht30   swr    xdio
bc28             co2     ds18b20  fs801    inc     mfm    mts4    n58     pack   puf     sm9569  tb600
bg95             co2ns   epd      fsxcs    ipos    mix    myadc   o2      ph     rfid    soil2   uart
bmp390           co2ss   esp      gps      ips     mn316  mymbus  o2ze03  phorp  rscoil  soil3   ultra
```

Among all of these drivers, following example projects are preferred to be used:
1. Project: **myadc**
    ADC sampling for calcuations. It collects the VBAT power, translate from ADC value to real VBAT voltage and evaluate the power capacity. It could be easily modified as any other ADC sensor case.

2. Project: **mymbus**
    RS485/Modbus sensor driver example. It uses the main library function **drv->data->modbus_read()** for Modbus READ and get the GHI sensor value. 
    ```
    v = drv->data->modbus_read(io, buf, addr, 3, 0, 1); /* read(03) 1 modbus register */
    if( v != 7 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    ```
    Traditional Modbus operations could use the main libary directly. Complicated operations should write the codes by raw IO read/write (refer to project: **im1281**)

3. Project: **myuart**
    RS232 sensor driver example. It reads a RS232 O2 sensor for multiple times and report the avarage O2 value. All IO operation is using IO READ directly and sensor response CRC is also verified. It could be used as a basic driver and changed for any other kind of UART based sensors.

## Start Your First Project
Below steps will start/import myuart project in STM32CubeIDE.
1. Start STM32CubeIDE
2. Click on:
   **File -> Import -> Existing Projects into Workspace -> Next**
3. In the Import Projects page, 
   - Use "Browse" to select root direcory to "C:\workspace\iEdge4.0\4.0\src\drv\myuart"
   - Select the "drv_myuart" in Projects list
   - Click "Finish
4. Right click mouse on the new imported project - "drv_myuart" in the Project Explorer panel
5. Choose "Build Project" and wait for compile done (Refer to below)
```
22:49:10 **** Build of configuration Debug for project drv_myuart ****
make all 
arm-none-eabi-gcc "../drv_myuart.c" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -c -I../../inc -I../../../sdk/inc -Os -ffunction-sections -Wall -Werror -fPIC -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"drv_myuart.d" -MT"drv_myuart.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "drv_myuart.o"
arm-none-eabi-gcc -o "drv_myuart.elf" @"objects.list"   -mcpu=cortex-m0plus -T"../../LinkerScript.ld" --specs=nosys.specs -Wl,-Map="drv_myuart.map" -Wl,--gc-sections -nostartfiles -nodefaultlibs -nostdlib -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
Finished building target: drv_myuart.elf
 
arm-none-eabi-size  drv_myuart.elf 
   text	   data	    bss	    dec	    hex	filename
    456	     56	      0	    512	    200	drv_myuart.elf
Finished building: default.size.stdout
 
arm-none-eabi-objdump -h -S drv_myuart.elf  > "drv_myuart.list"
Finished building: drv_myuart.list
 
arm-none-eabi-objcopy -O binary "drv_myuart.elf" "../../../Listings/drv00a2_myuart.bin"
arm-none-eabi-size "drv_myuart.elf"
   text	   data	    bss	    dec	    hex	filename
    456	     56	      0	    512	    200	drv_myuart.elf

```

## Driver Code Structure
Choose the C file, for example "drv_myuart.c" and double click it to show its content:
```
/**
 * @file  drv_myuart.c
 * @brief SENSOR driver for generic UART example
 * @author Runby F.
 * @date 2025-4-8
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-04-08
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_MYUART_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_MYUART_NAME  "MYUART"

/**
 * @brief Operating timeout in unit ms
 */
#define DRV_UART_IO_TIMEOUT  100

/**
 * @brief Operating timeout retry times
 */
#define DRV_UART_RETRY_NUM  10

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_myuart_set(pos_u32_t on) {
  ma_board_t *b = g_drv->board;
  b->pwr_set(MB_PWR_ID_VCCN_PWR3, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_myuart_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt, max, min;
  pos_u16_t t;
  drv_api_t *drv = g_drv;
  pos_u8_t response[12];

  /* specify the UART ID */
  t = POS_IO_UART2;

  drv->log->data("myuart io", t);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* init io */
  io = drv->os->io->init(t, 9600, 0);

  /* read and report the average */
  sum = 0;
  cnt = 0;
  max = 0;
  min = 0xffffffff;
  for(t=0; t < DRV_UART_RETRY_NUM; t++ ) {
	const pos_u8_t cmd_read[9] = {0xff, 0x20, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a};
	/* flush before read */
	while( io->read(io, POS_NULL, 1024, 50) >= 1024 );

	/* pull command */
	io->write(io, (pos_u8_t*)cmd_read, 9, DRV_UART_IO_TIMEOUT);

	/* read response */
	v = io->read(io, response, 9, DRV_UART_IO_TIMEOUT);
	drv->log->buf("RX", response, v);
	if( v != 9 )
	  continue;

	if( response[0] != 0xff && response[1] != 0x86 )
	  continue; /* wrong format */

	/* crc */
	{
	  pos_u32_t i;
	  pos_u8_t c;
	  c = response[0];
	  for( i = 1; i < v; i++ )
		c += response[i];
	  if( c != 0xff ) /* crc check */
	   continue; /* wrong crc */
	}

	v = (response[2]<<8) + response[3];
	cnt++;
	sum += v;

	if( cnt >= 1 )
	  break;

	drv->os->tick->sleep(1000);

  }

  if( !cnt ) {
	/* no valid data got */
	drv->data->put("\"%s\":\"no sensor data\",", "err", 0, 0, 0, 0);
	/* set error flag */
	drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  } else {
	if( cnt > 2 ) {
	  sum = sum - min - max;
	  cnt -= 2;
	}
	v = sum / cnt;
	drv->data->plss_put_u16(61, v); /* PLSS data format put, 61 for PLSS_PL_EXT_O2: U16 in  unit of 0.1% */

	drv->data->put_raw("\"o2\":", 5);
    drv->data->put_ufloat(v, 10);
  }

  if( io ) {
	/* shutdown IO after read */
	io->release(io);
  }

  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(162, DRV_MYUART_VERSION), /* 162 is registered sensor type */
  .name = DRV_MYUART_NAME,
  .u.sensor={
    .power_set = sensor_myuart_set,
    .collect = sensor_myuart_collect,
  },
};
```
The above code is a running RS232 based driver for O2 sensor. It includes following major elements:
- Global **"g_export"** variable to export its power control function and sensor data collecting function to the iEdge4.0 main program
  * The main program will get the function entry through this "g_export" variable and thus control the sensor by provided functions
  * The calling to these function is address in-dependent (-fPIC as C flags)
  * Stack usage should be within 700 bytes
  * No other static or global variables are allowed and only "g_export" is defined in DATA section:
    - BSS section size MUST be ZERO
    - DATA section size must be 56 (exactly length of sizeof(g_export))
- Function body for **"sensor_myuart_set"**
  * When the main program start to collecting data, it will call this function with parameter on=1 to perform sensor power on actions;
  * After sensor collecting is done, the main program will perform sensor power off through this function with parameter on=0
- Function body for **"sensor_myuart_collect"**
- A sensor init function is optional
  * For example: .init = sensor_myuart_init (refer to other projects like drv_co2 for reference)
  * The main program will call this function with init flag = 1 (to-be-init) for the first time of running.  

## Compile and Project Binary files
In STM32CubeIDE, right click the name of the project and choose "Build Project" menu. Wait for new project binary files to be generated. Or if there's code error, it will stop compiling and prompts the errors in IDE's console panel. After the project binary is successfully generated, folloiwng folder will contain the final compiled binary:
```
/mnt/c/workspace/iEdge4.0/4.0/src/Listings$ ls -al
total 4
drwxrwxrwx 1 tliu tliu 4096 Apr 10 22:49 .
drwxrwxrwx 1 tliu tliu 4096 Apr  7 23:30 ..
-rwxrwxrwx 1 tliu tliu   20 Apr  7 23:30 .gitignore
-rwxrwxrwx 1 tliu tliu  680 Apr  8 20:57 drv00a0_myadc.bin
-rwxrwxrwx 1 tliu tliu  392 Apr  8 20:57 drv00a1_mymbus.bin
-rwxrwxrwx 1 tliu tliu  512 Apr 10 22:49 drv00a2_myuart.bin
```
The binary file (dvr00a2_myuart.bin) is a special C program. iEdge 4.0 platform will recognize its format and calling its related functions for Power control and sensor collecting by "g_export" definitions. The generation contains two step:
1. Compile and ELF file generating
2. Objcopy from ELF to final Binary
Please note only the binary file (*.bin) could be used in iEdge 4.0 target downloading & running.

## Download to Target Board
Below is the guide about how to download a new compiled binary onto the iEdge4.0 target board:
1. Connect the USB port to an iEdge4.0 device
2. Use a Serial Console utility to connect to iEdge4.0's serial port with 8N1 9600 baudrate 
3. Turn on the serial console's chat window (or command window) which supports interact (or chat) window and finally write the full command towards target board. For example, SecureCRT's chat window
4. In the chat window, type command "1" and ENTER to check the board response. If connection is correct, following would be seen:
```
Press following KEY for:
    0 ... System reset
    1 ... Show this help
    2 ... Show running info
    3 ... Erase all
    4 ... XMODEM Program POS firmware
    5 ... XMODEM Program MA firmware
    6 ... XMODEM Program drivers
    7 ... XMODEM Program User#2
    8 ... XMODEM Program EEPROM CFG
    9 ... Switch Main App Debug
    + ... Change console 9600/115200 baudrate
Note:
  1) Console is always 9600 baudrate after reset
```
5. Once the connection is good, type command "stop" and ENTER to stop the device running. If the device happens to collecting data or reporting now, please be patient to wait for its done and "stop" command is process. Once "stop" command is processed by the device, following messages would be seen:
```
!!!OK!!! Please wait and check FSM is STOP.

[49697364] FSM is STOP (arg=0x80000)
```
6. Sometime, system log or debug messages would be very frequent and hard to check the above response. Type command "info" and ENTER to check if the current device is is STOP state. Following is an example indicating the device is stopped now:
```
info
[Running Info]
            FSM: 0x80000 (STOP state) <=== This indicates it's STOP now and we can download new binaries now
         Module: 0x4384/0x1004f (type/state)
           Loop: 495 (main loop counter)
           Duty: 276 (duty counter)
       Main App: v4.0.4.5 (MA version)
            POS: v1.0.2.13 (POS version)
       Free Mem: 19616 (free heap size)
    Mininal Mem: 18584 (history min free size)
  Running Ticks: 49713852 (accumulated, unit ms)
 Sleeping Ticks: 48588793 (accumulated, unit ms)
       OS Stack: 876
      APP Stack: 696
     IDLE Stack: 64
  Startup Stack: 0x4078
       Poll Num: 0
  NACK INIT/SYS: 0/0 (module/sys reset timeout)
       Reserved: 01000000ef010000140100000000080002000000

[Sensor Info]
SLOT CYCLE(/P.) PWRms IO/ADD  THW0  THW1 TYPE VER/NAME(STAT/DUTY/CTRL/SRSV)
 [0]   180/   0     1 0x0000     0     0    1 v1.21 VBAT (OK/49706150/139/0x5)
 [1]   180/   0   750 0x0001     0     0    9 v1.11 CO2NS (OK/49706150/137/0x0)
 [2]   180/   0     1 0x0700     0     0   11 v1.23 SHT30 (OK/49706150/137/0x3)
```
7. Type command "xupg" and ENTER. And then choose the serial console utility's Transfer XMODEM function to send the binary (drv00a2_myuart.bin) to the device. The downloading will take sometime. Please wait until it's done like below:
```
Please send file through XMODEM ...

Start xmodem transferring. Press Ctrl+C to cancel.
  100%     512 bytes  512 bytes/s 00:00:01       0 Errors


Total Receive Bytes: 512

Upgrade driver - MYUART, v1.0, 512 bytes.
Unmount and wait ......

!!!OK!!! Upgrade done.
```

## Activate a Sensor and Execute It Imediately
The system has about 56KB flash to install any sensor or communication modules. Refer to below steps about how to activate a sensor (drv_myuart):
1. Type command "list" and ENTERY to check what we have downloaded in this device. It could see following response with this command:
```
[DRIVER DIRECTORY]
      Type         Filename            Class       Size       Disk Version
    0x4000             LORA           Module       1352       1536 1.12
    0x4181              ESP           Module       1256       1536 1.2
    0x4183             A780           Module       1608       2048 1.5
    0x4184             N306           Module       4028       4096 1.11
    0x4185             BG95           Module       2876       3072 1.5
    0x4186            MN316           Module       4052       4096 1.1
    0x42FF          CONSOLE           Module        200        512 1.1
    0x0009            CO2NS           Sensor       3904       4096 1.11
    0x000A              EPD           Sensor      14060      14336 1.20
    0x000B            SHT30           Sensor       4148       4608 1.23
    0x0001             VBAT           Sensor       1176       1536 1.21
    0x00A2           MYUART           Sensor        512        512 1.0
                      Total                       39172      41984
Total Remaining: 15360 Bytes
```
2. Assign the 0xa2 (or 162 in decimal) sensor type in sensor slot#3. In this example, slot#0/#1/#2 have been enabled with other sensors. We use #3 for the "MYUART". Type command "set type[3] 160" to assign this sensor into slot#3. Type "set cycle[3] 60" to choose a 60s duty cycle to call this sensor. Each command will turn a "!!!OK!!! xxxxx" messages. The response would be like:
```
set type[3] 162
!!!OK!!! Type @0x114 is modified.)
set cycle[3] 60
!!!OK!!! Cycle @0x110 is modified.)
```
3. Type command "set ctrl 0x4005" to dump sensor reporting and debug messages. 
4. Type command "dbg 5" to resume device running and immediately start a duty now. Since the myuart does not find a valie sensor, it probably works as below:
```
[50783706] myuart io (arg=0x2) <=== This is the "myuart"'s collecting function log

[50783866] RX (len=0) 

[50784025] RX (len=0) 

[50784184] RX (len=0) 

[50784343] RX (len=0) 

[50784502] RX (len=0) 

[50784661] RX (len=0) 

[50784820] RX (len=0) 

[50784979] RX (len=0) 

[50785138] RX (len=0) 

[50785297] RX (len=0) 

[50785297] {"per":100,"vbat":4998,"rf":50,"co2":411,"tmp":20.6,"hum":20.7,"err":"no sensor data"} <=== When HW sensor is not ready, "err" will be prompted

[50785297] tx (len=31) 2514202311284000005a03d77e6f640713864d0000003228019b0100ce0215

[50789426] rx (len=12) 2514202311284000005adfff
```
5. Wait for 60s. And another myuart duty will collect and report again. This is due to the above "set cycle[3] 60" is using 60s duty cycle. 

## De-activate a Sensor or Adjust a Sensor's Cycle/Duty Time
Since sensor is controlled by any of eight duty slots, to choose a slot's cycle with ZERO will de-activate this slot (sensor). 
1. De-activate a given slot
   Below example commands, will deactivate slot #0/#1/#2
  ```
   set cycle[0] 0
   set cycle[1] 0
   set cycle[2] 0
   ```
2. De-activate all slots
   ```
   set cycle 0 
   ```
3. Adjust all slots with cycle (duty) time 0 (disable them all) and only set slot#3 as 60s
   ```
   set cycle 0
   set cycle[3] 60
   ```
**Note: the duty/cycle change will happens on next duty. The device will check these change after current duty sleep/execution is done and use the new duty times for the next duty.**

## Debug and Console Command Help
Type "help" will list all console commands. It would be like:
```
help
[APP v4.1.3.2 Help]
Type following commands in chat window:
 ADC PXn       : Show PIN (pa0~pf15) ADC Input
 CFGO          : Show OneNET/CFG file
 CFGP          : Show PIN/CFG file
 CFGR          : Show CFG file
 DBG XXXX      : Debug commands or help info
 DEL XXXX or * : Delete Driver Type/Name (* for all)
 DUMP          : Show all EEPROM CFG (including all zero fields)
 HELP          : Show this help information
 HELP DBG/PIN/SET : Show DBG/PIN/SET help information
 INFO          : Show running information
 INP PXn       : Show PIN (pa0~pf15) Input level
 LIST          : List driver directory
 MOD PXn xxx   : Set PIN (pa0~pf15) mode
 ONE AAA BBB   : Set OneNET Field(AAA) to BBB
 OTA X.X.X.X   : OTA image by version (Not supported)
 OUT PXn 0/1   : Ouput PIN (pa0~pf15) 0/1
 PIN AAA[n] BBB: Set PIN Field(AAA[n]) to BBB
 PWD PXn xxx   : PWM ouput PIN (pa0~pf15) Duty (1~10000)
 PWM PXn xxx   : PWM ouput PIN (pa0~pf15) Hz
 PINS          : Show all PINs information
 RTC YYMMDDHHMMSSWW : Set RTC BCD time
 SET AAA BBB   : Set Field(AAA) to BBB
 SHOW          : Show none-zero EEPROM CFG
 SRUN          : Set FSM State RUN
 STOP          : Set FSM State STOP
 XUPG          : Xmodem upgrade (can be any firmware, configurations and drivers)

Note:
  1) BBB can be number or string according to each field
  2) BBB can be * to clear this field to all ZERO/EMPTY
  3) n is ZERO based
```
Or type "help dbg" to list all dbg commands:
```
 dbg 0         : Show this debug help
 dbg 1         : Control Module Uart until ESCx2
 dbg 1 1       : Quit TRANS mode and control Module Uart
 dbg 2 XXXX    : Set module uart baudrate
 dbg 3         : Backup current CFG to factory default
 dbg 4         : 15s Auto Reset and Control Module Uart until ESCx2
 dbg 5 XXXX    : Re-Schedule duty time
 dbg 6 XXXX    : Set 0-allow sleeping, 1-never sleep
 dbg 7 XXXX    : Set 0-require for confirm, 1-not require
 dbg 8~23 XXX  : Uart0~15 debug under XXX baudrate
 dbg 32~47 XXX : I2C0~15 debug under XXX baudrate
 dbg AAA       : Get U32 memory [AAA], AAA should be >= 0x20000000
 dbg AAA XXX   : Set U32 memory [AAA] to value XXX

```
Or type "help set" to list all SCFG set commands:
```
set Magic XXXXXXXX
set Report XXXX
set CTRL XXXX
set EUI XXXXXXXXXXXXXXXX
set LoRaEUI XXXXXXXXXXXXXXXX
set LoRaAPP XXXXXXXXXXXXXXXX
set LoRaKey XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set LoRaASKey XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set LoRaNSKey XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set LoRaAddr XXXXXXXX
set LoRaNonce XXXX
set LoRaModel XX
set LoRaAckLmt XX
set LoRaAckDly XX
set LoRaNBTrans XX
set LoRaCtrl XX
set LoRaPower XX
set LoRaGroup XX
set LoRaTX XX
set LoRaRX XX
set LoRaFlags XX
set ResetModule XXXX
set ResetSystem XXXX
set MBUS XXXX
set TCTRL XXXXXXXXXX
set GRSV XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set CALIB XX
set DR XX
set MIO XX
set Console XX
set Fixed XX
set Cycle[0..7] XXXXXXXX
set Type[0..7] XXXX
set Power[0..7] XXXX
set Periodic[0..7] XXXX
set IO[0..7] XXXX
set SRSV[0..7] XXXXXXXX
set ARG[0..7] XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set D0[0..7] XXXXXXXX
set D1[0..7] XXXXXXXX
set D2[0..7] XXXXXXXX
set PID[0..7] XXXX
set THW0[0..7] XXXX
set THW1[0..7] XXXX
set THW2[0..7] XXXX
set THW3[0..7] XXXX
set THW4[0..7] XXXX
set THW5[0..7] XXXX
set THW6[0..7] XXXX
set THW7[0..7] XXXX
set THLD[0..7] XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set ROM[0..15] XXXXXXXXXXXXXXXX
set Position XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set Publish STRING[63]
set Subscribe STRING[63]
set MUSR STRING[15]
set MPWD STRING[15]
set MCID STRING[31]
set DNS X.X.X.X
set QOSP XX
set QOSS XX
set NRSV XXXXXXXXXXXX
set Retry XX
set Delay XX
set Port XXXX
set URL STRING[31]
set BAND STRING[31]
set APN STRING[31]
set SSID STRING[31]
set WRSVD XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
set WPWD STRING[15]
```
Or type "help pin" to list all GPIO PIN setup commands:
```
pin Magic 0xXX or XXXX (hex or decimal integer)
pin Version 0xXX or XXXX (hex or decimal integer)
pin LEDR PXXX
pin LEDG PXXX
pin LEDB PXXX
pin LEDY PXXX
pin PullUp PXXX
pin PullDn PXXX
pin PullUp2 PXXX
pin PullDn2 PXXX
pin SetH[0..7] PXXX
pin SetL[0..7] PXXX
pin PWR[0..31] PXXX
pin Wire1[0..7] PXXX
pin AIN[0..7] PXXX
pin DIO[0..7] PXXX
pin PWM[0..3] PXXX
pin PMODE[0..3] 0xXX or XXXX (hex or decimal integer)
pin CTRL[0..31] PXXX
pin RSVD[0..95] PXXX
pin UART0..15[0..3] PXXX
pin LPUART0..15[0..3] PXXX
pin I2C0..15[0..3] PXXX
pin SPI0..15[0..3] PXXX
```
