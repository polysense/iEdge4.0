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

## Download to Target Board and Verify its Running
TBD



