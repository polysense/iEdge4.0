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

## Start Your First Driver Project
This driver library contains a series of offical Polysense sensor drivers which have been maintined by Polysense. Also, it contais several example sensor drivers for illustration. All these drivers are stored in sub folder - 4.0/src/drv, which will look like:
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
    Tradional Modbus operations could use the main libary directly. Complicated operations should write the codes by raw IO read/write (refer to project: **im1281**)

3. Project: **myuart**
    RS232 sensor driver example. It reads a RS232 O2 sensor for multiple times and report the avarage O2 value. All IO operation is using IO READ directly and sensor response CRC is also verified. It could be used as a basic driver and changed for any other kind of UART based sensors.

Below steps will start/import myuart project in STM32CubeIDE.







