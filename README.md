# DFRobot_LoRa

- [中文版](./README_CN.md)

TEL0121 is a Lora 433M communication module based on SX1278. Users can communicate with the module with SPI. <br>
TEL0122 is a Lora 915M communication module based on SX1278. Users can communicate with the module with SPI.<br>
Provides an Arduino library for FireBeetle Covers-LoRa Radio. <br>
Better power supply provides farther distance. <br>
test environment: <br>
In city, a straight road with less obstacles. <br>
The test available distance is 1km.

![](./resources/images/TEL0122.png)

## Product Link(https://www.dfrobot.com/product-1665.html)

    SKU：TEL0122

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary
The library is used to Send and receive Long Range Radio data.

## Installation

There are two ways to use the library:
1. Open the Arduino IDE, search for "DFRobot_LoRa" in Tools --> Manager Libraries on the status bar, and install the library.
2. First download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in that folder.

## Methods

```C++
  /**
   * @fn init
   * @brief  Initialise lora module
   * @param NSSPin		output pin,slave select pin
   * @param NRESETPin 	output pin,enter shutdown mode when driving low
   * @return  1 if ok, 0 otherwise
   */
  bool init(uint8_t NSSPin = NSS_PIN, uint8_t NRESETPin = RESET_PIN);
  
  /**
   * @fn rxInit
   * @brief Initialise rx mode.
   * @return  1 if ok, 0 otherwise
   */
  bool rxInit();
  
  /**
   * @fn sendPackage
   * @brief Sending packet through RF
   * @param sendbuf		buf of data to send
   * @param	sendLen		length of data to send,less than 64 bytes
   * @return  1 if tx ok, 0 otherwise
   */
  bool sendPackage(uint8_t* sendbuf,uint8_t sendLen);
  
  /**
   * @fn receivePackage
   * @brief Receive packet from RF
   * @param recvbuf		buf to save the rx data
   * @return length of rx data
   */
  uint8_t receivePackage(uint8_t* recvbuf);
  
  /**
   * @fn waitIrq
   * @brief Inquire interrupt.
   * @param irqMask		interrupt flag
   * @return  1 if interrupt occur, 0 if no interrupt
   */
  bool waitIrq(uint8_t irqMask = LR_RXDONE_MASK);

  /**
   * @fn idle
   * @brief Enter standby mode.
   */
  void idle();

  /**
   * @fn sleep
   * @brief Enter sleep mode.
   */
  void sleep();

  /**
   * @fn clearIRQFlags
   * @brief Clear interrupt
   */
  void clearIRQFlags();
  
  /**
   * @fn setFrequency
   * @brief Set RF frequency.
   * @param	freq    value of frequency
   * @return  1 if ok, 0 otherwise
   * @note: the range is 137~1020Mhz for lora1276 ,137~525Mhz for lora1278
  */
  bool setFrequency(uint32_t freq);
  
  /**
   * @fn setRFpara
   * @brief Set RF parameter.
   * @param BW        bandwidth , range from 7.8k to 500k
   * @param CR        coding rate , range from 4/5 to 4/8
   * @param SF        spreading factor , range from 6 to 12
   * @param CRC
   * @n       LR_PAYLOAD_CRC_ON:enable CRC,
   * @n       LR_PAYLOAD_CRC_OFF:disable CRC
   * @return  1 if ok, 0 otherwise
   * @note: the RF data rate depends on bandwidth and spreading factor coding rate affects time on air. if SF=6 ,it will turn to implict mode in this function
   */
  bool setRFpara(uint8_t BW,uint8_t CR,uint8_t SF,uint8_t CRC);
  
  /**
   * @fn setPreambleLen
   * @brief Set preamble length.
   * @param len preamble length
   * @return  1 if ok, 0 otherwise
   */
  bool setPreambleLen(uint16_t length);
  
  /**
   * @fn setHeaderMode
   * @brief Set header mode
   * @param	mode LR_IMPLICIT_HEADER_MODE or LR_EXPLICIT_HEADER_MODE
   * @note	if SF=6 ,it must be implicit header mode
   */
  bool setHeaderMode(uint8_t mode);
  
  /**
   * @fn readRSSI
   * @brief Read rssi
   * @param  mode  0 read  current rssi, 1 read rssi of last packet received
   * @return  value of rssi
   */
  uint8_t readRSSI(uint8_t mode = 0);

  /**
   * @fn setTxPower
   * @brief Set tx power.
   * @param power  power level,0 to 15
   * @return  1 if ok, 0 otherwise
   */
  bool setTxPower(uint8_t power);

  /**
   * @fn setPayloadLength
   * @brief Set payload length
   * @param	len payload length
   * @note in implicit header mode ,payload length must be set first .length is fix in  implicit header mode
   */
  bool setPayloadLength(uint8_t len);
```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32   |      √       |             |            | 
FireBeetle-ESP8266 |      √       |             |            | 
FireBeetle-328P    |      √       |             |            | 

## History

- 2021/11/01 - Version 1.0.3 released.
- 2021/11/01 - Version 1.0.2 released.
- 2021/10/18 - Version 1.0.1 released.
- 2017/08/30 - version 1.0.0 released.

## Credits

Written by Fary(feng.yang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))
