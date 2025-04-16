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

### Debug&Operation Examples
#### Console Basic 
The device debug console is default working at 9600 Baudrate, No FlowControl, Data 8, Stop 1. As a lower power UART console, it's preferred to work only with 9600 Baudrate. All input must be a full command. For example, "help\r". It does not support one by one character input and inline editting. So the best style is to use a Terminal tool like SecureCRT and type/edit commands in its interactive window. 
1. System Information Check - "info"
   This is a very useful command to check the current system running status, including: FSM state, module type, duty/loop counters, running/speeping time, stack/heap usage and sensor/slot configurations. Refer to below example:

   ```
    info
    [Running Info]
              FSM: 0x32000 (SLEEP state)
          Module: 0x4006/0x1000f/0% (type/state/signal)
            Loop: 531 (main loop counter)
            Duty: 506 (duty counter)
        Main App: v4.1.3.1 (MA version)
              POS: v1.1.0.2 (POS version)
        Free Mem: 18600 (free heap size)
      Mininal Mem: 17048 (history min free size)
    Running Ticks: 30478504 (accumulated, unit ms)
    Sleeping Ticks: 29106846 (accumulated, unit ms)
          OS Stack: 880
          APP Stack: 704
        IDLE Stack: 64
      Startup Stack: 0x4078
          Poll Num: 0
      NACK INIT/SYS: 0/0 (module/sys reset timeout)
          Reserved: 0100000013020000fa0100000020030000000000
        Console/Ext: 0x11/0x10
                RTC: 00000008275800 (2000-00-00 08:27:58 W0 X)

    [Sensor Info]
    SLOT CYCLE(/P.) PWRms IO/ADD  THW0  THW1 TYPE VER/NAME(STAT/DUTY/CTRL/SRSV)
    [0]    60/   0     5 0x0000     0     0  160 v1.0 MYADC (OK/30505905/137/0x0)
   ```

2. Configuration Dump - "cfgr"
   This command will dump all configurations. See below example:
   ```
   cfgr
   [EEPROM CFG FILE]
    set Magic 0x92747613
    set Report 0x4006
    set CTRL 0x400f
    set EUI 3333333333333333
    set LoRaEUI 3333333333333333
    set LoRaAPP 2017060000000000
    set LoRaKey 11223344556677889900aabbccddeeff
    set LoRaNonce 0x360
    set LoRaAckLmt 0x40
    set LoRaAckDly 0x20
    set LoRaNBTrans 0x1
    set LoRaCtrl 0x2
    set LoRaTX 2
    set LoRaRX 222
    set Cycle[0] 60
    set Type[0] 0xa0
    set Power[0] 5
    set ARG[0] 000000000000000000000000000061ea
    set PID[0] 60001
    set Type[1] 0xb
    set IO[1] 0x700
    set SRSV[1] 0x3
    set ARG[1] 000000000000000000000000000065ea
    set PID[1] 60005
    set Type[2] 0xa1
    set Type[3] 0xa2
    set Publish /iot/00000000/up/3333333333333333
    set Subscribe /iot/00000000/dn/3333333333333333
    set MUSR RDdemo
    set MPWD polysense_0379
    set MCID plss_3333333333333333
    set DNS 8.8.8.8
    set Port 1560
    set URL ota.polysense.online
    set BAND 5,8
    set APN cmnbiot
    set SSID plss_wifi
    set WPWD polysense
   ```

3. Flash Directory Show - "list"
   This command will list all the drivers within flash area and remaining flash size. See below example:
   ```
   list
   [DRIVER DIRECTORY]
      Type         Filename            Class       Size       Disk Version
    0x42FF          CONSOLE           Module        260        512 1.3
    0x0001             VBAT           Sensor       1176       1536 1.21
    0x000A              EPD           Sensor      19244      19456 1.41
    0x000B            SHT30           Sensor       4824       5120 1.30
    0x00A0            MYADC           Sensor        680       1024 1.0
    0x00A1           MYMBUS           Sensor        392        512 1.0
    0x00A2           MYUART           Sensor        512        512 1.0
    0x4000             LORA           Module       4396       4608 1.21
                      Total                       31484      33280
   Total Remaining: 24064 Bytes
   ```

4. Flash Driver Delete - "del XXXX(driver type value)"
   This command will delete a certain driver by given XXXX type value. For example, "del 1" will delete VBAT driver. See below example:
   ```
   del 1
   Unmount and wait ......
   !!!OK!!! Driver has been erased. 
   list
   [DRIVER DIRECTORY]
      Type         Filename            Class       Size       Disk Version
    0x42FF          CONSOLE           Module        260        512 1.3
    0x000A              EPD           Sensor      19244      19456 1.41
    0x000B            SHT30           Sensor       4824       5120 1.30
    0x00A0            MYADC           Sensor        680       1024 1.0
    0x00A1           MYMBUS           Sensor        392        512 1.0
    0x00A2           MYUART           Sensor        512        512 1.0
    0x4000             LORA           Module       4396       4608 1.21
                      Total                       30308      31744
   Total Remaining: 25600 Bytes
   [DRIVER DIRECTORY]
      Type         Filename            Class       Size       Disk Version
    0x42FF          CONSOLE           Module        260        512 1.3
    0x0001             VBAT           Sensor       1176       1536 1.21
    0x000A              EPD           Sensor      19244      19456 1.41
    0x000B            SHT30           Sensor       4824       5120 1.30
    0x00A0            MYADC           Sensor        680       1024 1.0
    0x00A1           MYMBUS           Sensor        392        512 1.0
    0x00A2           MYUART           Sensor        512        512 1.0
    0x4000             LORA           Module       4396       4608 1.21
                      Total                       31484      33280
   Total Remaining: 24064 Bytes
   ```
   **Note: sometime if driver is being used and unable to get it done in short term, the delete will trigger a system reboot.**

#### POS/MAIN/Driver/... Image Upgrading
Console command "xupg" support to XMODEM upgrade a new file. In order to make sure the system healthy, it's always required that stop the system running by command - "stop", perform upgrading and finally resume the system running by command - "srun". See below example sequence:
1. Stop FSM by command "stop"
Type "stop" to stop the system running and wait for printing of "FSM is STOP". Do NOT upgrade an image during system running. It might broken the system sometime. See below example:
```
stop
!!!OK!!! Please wait and check FSM is STOP.

[31375264] sleep done (arg=0x2ef)

[31375264] FSM is STOP (arg=0x80000)
```
2. XModerm Upgrade an image file by command "xupg"
After type command "xupg", folloiwng prompts will be printed:
```
xupg
Please send file through XMODEM ...
............
```
Choose the Terminal tools' XMODEM transferring function, browse and select the file for uploading and wait for it's done. Below example upload "drv_vbat.bin" onto the target board:
```
Start xmodem Transferring. Press Ctrl+C to cancel.
  100%       1 KB    1 KB/s 00:00:01       0 Errors


Total Receive Bytes: 1176

Upgrade driver - VBAT, v1.21, 1176 bytes.
Unmount and wait ......

[31667779] program driver (arg=0x8098)

!!!OK!!! Upgrade done.
```
If the file is POS kernel image (pos.bin) or MAIN image (main.bin), it requires a double confirm. Type "y" to confirm the upgrading, or else it will be cancelled. Once it's confirmed, it will be really programmed onto the flash and then reboot system automatically with new upgraded files. See below of main.bin upgrading:
```

!!!WARNING!!! MA area will be erased. 
Press Y to confirm ... Confirmed

[Startup Mode]
            POS: v1.1.0.2
       Free Mem: 22544
    Mininal Mem: 22544
  Running Ticks: 2
 Sleeping Ticks: 0
       OS Stack: 1088
      APP Stack: 1280
     IDLE Stack: 64
  Startup Stack: 0x4078
       Reserved: 0000000000000000000000000000000000000000
      Exp Sleep: 0
Waiting for remote access ...
Starting MA application ...

Main  App: v4.1.3.2
OS Kernel: v1.1.0.2

Main App Normal mode.

......
```

3. Resume System Ruunning by command "srun"
This step is used only for driver image upgrading. Since POS kernel and MAIN image upgrading is always require a reboot, the system resuming operation is only required for sensor/module drivers. Or else, the system is always staying at STOP state. See below example:
```
!!!OK!!! Switch to RUN.

[175555] FSM is RUN (arg=0x0)

.....
```

#### POS/MAIN/Driver/... Image Upgrading without Manual Confirmation
If the manual confirmation "y" is not preferred, type command "dbg 7 1" before the above Image Upgrading procedure. In this case, even if the pos.bin or main.bin is upgraded, the system will operate the final upgrading immediately without the wait for user confirmation. Below example is the command to disable such manual confirmation:
```
dbg 7 1
Not require to confirm!
```
Or if you want to resume the double confirm, type command "dbg 7 0". For example:
```
dbg 7 0
Require to confirm!
```
By each reboot default, it's always required to double confirm.

#### Execute a Duty Report Immediately 
Sometime, we want a duty report immediately withtout waiting for the next cycle time. It could be achieved by command "dbg 5". See below example:
```
dbg 5
[645935] sleep done (arg=0x6b29)

[645935] collect loop (arg=0xf)

[645935] sensor age bmp (arg=0x10001)

[645935] pwr set (arg=0x10a07)

[645935] pwr set (arg=0x105ff)

[645935] power on sleep ms (arg=0x5)

[646209] collect (arg=0xa0)

[646924] put (arg=0x1e)

[646924] per (arg=0x16f)

[646924] put (arg=0xd3e)

[646924] mv (arg=0x207)

[646924] power off (arg=0x0)

[646924] pwr set (arg=0xa07)

[646924] pwr set (arg=0x5ff)

[646924] collect done len (arg=0x14)

[647209] report start (arg=0x14)

[647209] {"per":30,"mv":3390}

[647209] tx (len=18) 2500333333333333333303d77e6f1e070d3e

[647209] ext run (arg=0x10002)

[647209] net rdy chk (arg=0x0)

==>AT+RSSI

<==+RSSI:1:-27:9

OK


==>AT+SEND=2:0:D77E6F1E070D3E

<==
OK


<==+EVT:222:01:FF
+EVT:RX_!, PORT 222, DB 5, RSSI -27, SNR 9


==>AT+RECV

<==+RECV:2:DEFF

OK


[649410] slora rx (len=1) ff

[649410] ma tx status (arg=0x0)

[649410] rx raw (len=12) 25003333333333333333dfff

[649410] plss udp cmd (len=1) ff

[649410] report done (arg=0x0)

[649630] ext run (arg=0x20001)

[649630] sleep start (arg=0xdbf1)

``` 


#### Debug Print Control
Following commands will control the printing of debug by different levels:
1. "set ctrl 0x4000" - Normal PSM working mode and no debug printing

2. "set ctrl 0x4001" - Normal PSM working mode and prints only TX/RX. See below:
```
set ctrl 0x4001
!!!OK!!! CTRL @0x6 is modified.

dbg 5

[879779] {"per":31,"mv":3392}

[879779] tx (len=18) 2500333333333333333303d77e6f1f070d40

[881619] rx raw (len=12) 25003333333333333333dfff

```
2. "set ctrl 0x4005" - Normal PSM working mode and prints TX/RX and drivers' log. See below:
```
set ctrl 0x4005
!!!OK!!! CTRL @0x6 is modified.

dbg 5

[952353] {"per":30,"mv":3390}

[952353] tx (len=18) 2500333333333333333303d77e6f1e070d3e

[956531] slora rx (len=1) ff

[956531] rx raw (len=12) 25003333333333333333dfff

```
3. "set ctrl 0x400f" - Normal PSM working mode and prints all debug log. See below:
```
set ctrl 0x400f
!!!OK!!! CTRL @0x6 is modified.

dbg 5


[1004837] sleep done (arg=0xbcb2)

[1004837] collect loop (arg=0x24)

[1004837] sensor age bmp (arg=0x10001)

[1004837] pwr set (arg=0x10a07)

[1004837] pwr set (arg=0x105ff)

[1004837] power on sleep ms (arg=0x5)

[1005119] collect (arg=0xa0)

[1005836] put (arg=0x1e)

[1005836] per (arg=0x16f)

[1005836] put (arg=0xd3e)

[1005836] mv (arg=0x207)

[1005836] power off (arg=0x0)

[1005836] pwr set (arg=0xa07)

[1005836] pwr set (arg=0x5ff)

[1005836] collect done len (arg=0x14)

[1006130] report start (arg=0x14)

[1006130] {"per":30,"mv":3390}

[1006131] tx (len=18) 2500333333333333333303d77e6f1e070d3e

[1006131] ext run (arg=0x10002)

[1006131] net rdy chk (arg=0x0)

==>AT+RSSI

<==+RSSI:1:-27:10

OK


==>AT+SEND=2:0:D77E6F1E070D3E

<==
OK


<==+EVT:222:01:FF
+EVT:RX_1, PORT 222, DR 5, RSCI -27, SNR 9


==>AT+RECV

<==+RECV:2:DEFF

OK


[1008338] slora rx (len=1) ff

[1008338] ma tx status (arg=0x0)

[1008338] rx raw (len=12) 25003333333333333333dfff

[1008338] plss udp cmd (len=1) ff

[1008338] report done (arg=0x0)

[1008564] ext run (arg=0x20001)

[1008564] sleep start (arg=0xdbd1)

```
**Note: more debug prints means much more power cost. For better battery life, please "set ctrl 0x4000" during normal working.**
