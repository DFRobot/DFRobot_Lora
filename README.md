# Lora Module
<img width="400" height="400" src="https://raw.githubusercontent.com/DFRobot/binaryfiles/master/TEL0122/TEL0122svg1.png"/>

# DFRobot_Lora Library for Arduino

Provides an Arduino library for FireBeetle Covers-LoRa Radio
## Table of Contents

* [Summary](#summary)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)
<snippet>
<content>

## Summary
The library is used to Send and receive GPRS or NB-IOT data (TCP,UDP)

## Methods
### Initialize the function

```C++
    bool init(uint8_t NSSPin = NSS_PIN, uint8_t NRESETPin = RESET_PIN);
```

### Initialise rx mode

```C++
    bool rxInit();
```

### Sending packet through RF
```C++
    bool sendPackage(uint8_t* sendbuf,uint8_t sendLen); // length of data to send,less than 64 bytes
```

### Receive packet from RF

```C++
    uint8_t receivePackage(uint8_t* recvbuf);
```

### Inquire interrupt

```C++
    bool waitIrq(uint8_t irqMask = LR_RXDONE_MASK); // LR_RXDONE_MASK and LR_TXDONE_MASK
```

### Clear interrupt

```C++
    void clearIRQFlags();
```

### Enter low power consumption

```C++
    void sleep();
```

### Enter standby mode

```C++
    void idle();
```

### Read rssi

```C++
    uint8_t readRSSI(uint8_t mode = 0);
```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
FireBeetle-BLE4.1 |      √       |             |            | 

## History

- data 2017-8-30
- version V1.0

## Credits

- author [wuxiao  <zed.wu@dfrobot.com>]