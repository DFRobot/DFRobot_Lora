# Lora Module
TEL0121 is a Lora 433M communication module based on SX1278. Users can communicate with the module with SPI. <br>
TEL0122 is a Lora 915M communication module based on SX1278. Users can communicate with the module with SPI.
<img width="400" height="400" src="https://raw.githubusercontent.com/DFRobot/binaryfiles/master/TEL0122/TEL0122svg1.png"/>

# DFRobot_Lora Library for Arduino

Provides an Arduino library for FireBeetle Covers-LoRa Radio. <br>
test environment: <br>
In city, a straight road with less obstacles. <br>
The test available distance is 1km.

## Table of Contents

* [Summary](#summary)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)
<snippet>
<content>

## Summary
The library is used to Send and receive Long Range Radio data.

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
FireBeetle-328P |      √       |             |            | 

## History

- date 2017-8-30
- version V1.0

- date 2018-12-5
- reviser: guojiehan jiehan.guo@dfrobot.com
- version v1.1
- optimization config to make the distance farther

## Credits

- author [wuxiao  <zed.wu@dfrobot.com>]
