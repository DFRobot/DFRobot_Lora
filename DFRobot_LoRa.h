/*!
 * @file DFRobot_LoRa.h
 * @brief DFRobot_LoRa class infrastructure
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangyang]
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-11-01
 * @url https://github.com/DFRobot/DFRobot_LoRa
 */
#ifndef __DFRobot_LoRa_H__
#define __DFRobot_LoRa_H__

#include <Arduino.h>
#include <SPI.h>

#if defined( ARDUINO_ESP32_DEV ) //FireBeetle-ESP32 
  #define NSS_PIN    D4 
  #define RESET_PIN  D2 
#elif defined (ARDUINO_ARCH_FIREBEETLE8266)//FireBeetle-ESP8266
  #define NSS_PIN    D4 
  #define RESET_PIN  D3 
#else
  #define NSS_PIN    4
  #define RESET_PIN  3
#endif

/**
 XOSC frequency,in lora1276/1278, F(XOSC)=32M
*/
#define LORA_XOSC						32000000
/**
 RF frequency band.Low frequency band is up to 525M
 lora1278 only support low frequency band
*/
#define LORA_FREQUENCY_BAND			LR_LowFreqModeOn_868M
/**
 the first bit in SPI address byte is a wnr bit
 1 for write access , 0 for read access
*/
#define LORA_SPI_WNR					 0x80
/**
 Waiting time for txdone irq
 this time is depended on BW ,SF,CR ,length of packet
 you must change this value according to actual parameter and length
 see more detail on page 27 of datasheet
*/
#define LORA_TX_TIMEOUT					1000
/**
 called by setAntSwitch
*/
#define LORA_MODE_RX					1
#define LORA_MODE_TX					2
#define LORA_MODE_STBY					0

/**
 LORA Internal registers Addres
*/
#define LR_RegFifo                       0x00
#define LR_RegOpMode                     0x01
#define LR_RegFrMsb                      0x06
#define LR_RegFrMid                      0x07
#define LR_RegFrLsb                      0x08
#define LR_RegPaConfig                   0x09
#define LR_RegPaRamp                     0x0A
#define LR_RegOcp                        0x0B
#define LR_RegLna                        0x0C
#define LR_RegFifoAddrPtr                0x0D
#define LR_RegFifoTxBaseAddr             0x0E
#define LR_RegFifoRxBaseAddr             0x0F
#define LR_RegFifoRxCurrentaddr          0x10
#define LR_RegIrqFlagsMask               0x11
#define LR_RegIrqFlags                   0x12
#define LR_RegRxNbBytes                  0x13
#define LR_RegRxHeaderCntValueMsb        0x14
#define LR_RegRxHeaderCntValueLsb        0x15
#define LR_RegRxPacketCntValueMsb        0x16
#define LR_RegRxPacketCntValueLsb        0x17
#define LR_RegModemStat                  0x18
#define LR_RegPktSnrValue                0x19
#define LR_RegPktRssiValue               0x1A
#define LR_RegRssiValue                  0x1B
#define LR_RegHopChannel                 0x1C
#define LR_RegModemConfig1               0x1D
#define LR_RegModemConfig2               0x1E
#define LR_RegSymbTimeoutLsb             0x1F
#define LR_RegPreambleMsb                0x20
#define LR_RegPreambleLsb                0x21
#define LR_RegPayloadLength              0x22
#define LR_RegMaxPayloadLength           0x23
#define LR_RegHopPeriod                  0x24
#define LR_RegFifoRxByteAddr             0x25
#define LR_RegModemConfig3               0x26
#define LR_RegDIOMAPPING1              	 0x40
#define LR_RegDIOMAPPING2                0x41
#define LR_RegVERSION                    0x42
#define LR_RegPLLHOP                     0x44
#define LR_RegTCXO                       0x4B
#define LR_RegPADAC                      0x4D
#define LR_RegFORMERTEMP                 0x5B
#define LR_RegAGCREF                     0x61
#define LR_RegAGCTHRESH1                 0x62
#define LR_RegAGCTHRESH2                 0x63
#define LR_RegAGCTHRESH3                 0x64

#define LR_LongRangeMode_FSK               0x00
#define LR_LongRangeMode_LORA              0x80
#define LR_ModulationType_FSK              0x00
#define LR_ModulationType_OOK              0x20
#define LR_LowFreqModeOn_868M              0x00
#define LR_LowFreqModeOn_433M              0x08
#define LR_Mode_SLEEP                	   0x00
#define LR_Mode_STBY                	   0x01
#define LR_Mode_FSTX                	   0x02
#define LR_Mode_TX                	   	   0x03
#define LR_Mode_FSRX                	   0x04
#define LR_Mode_RXCONTINUOUS               0x05
#define LR_Mode_RXSINGLE              	   0x06
#define LR_Mode_CAD               		   0x07

#define LR_PASELECT_RFO                	   0x00
#define LR_PASELECT_PA_POOST               0x80

#define LR_OCPON_ON                        0x20
#define LR_OCPON_OFF                       0x00

#define LR_LNAGAIN_G1                	   0x20
#define LR_LNAGAIN_G2                	   0x40
#define LR_LNAGAIN_G3                	   0x60
#define LR_LNAGAIN_G4                	   0x80
#define LR_LNAGAIN_G5                	   0xA0
#define LR_LNAGAIN_G6                	   0xC0
#define LR_LNABOOSTHF_0               	   0x00
#define LR_LNABOOSTHF_1               	   0x03

#define LR_RXTIMEOUT_MASK                  0x80
#define LR_RXDONE_MASK               	   0x40
#define LR_RXPCRCERROR_MASK                0x20
#define LR_VALIDHEADER_MASK                0x10
#define LR_TXDONE_MASK               	   0x08
#define LR_CADDONE_MASK               	   0x04
#define LR_FHSSCHANGECH_MASK               0x02
#define LR_CADDETECTED_MASK                0x01

#define LR_BW_7p8k                         0x00
#define LR_BW_10p4k                        0x10
#define LR_BW_15p6k                        0x20
#define LR_BW_20p8k                        0x30
#define LR_BW_31p25k                       0x40
#define LR_BW_41p7k                        0x50
#define LR_BW_62p5k                        0x60
#define LR_BW_125k                         0x70
#define LR_BW_250k                         0x80
#define LR_BW_500k                         0x90
#define LR_CODINGRATE_1p25                 0x02
#define LR_CODINGRATE_1p5                  0x04
#define LR_CODINGRATE_1p75                 0x06
#define LR_CODINGRATE_2                    0x08
#define LR_IMPLICIT_HEADER_MODE            0x01
#define LR_EXPLICIT_HEADER_MODE            0x00

#define LR_SPREADING_FACTOR_6              0x60
#define LR_SPREADING_FACTOR_7              0x70
#define LR_SPREADING_FACTOR_8              0x80
#define LR_SPREADING_FACTOR_9              0x90
#define LR_SPREADING_FACTOR_10             0xa0
#define LR_SPREADING_FACTOR_11             0xb0
#define LR_SPREADING_FACTOR_12             0xc0
#define LR_TX_CONTINUOUS_MODE              0x08
#define LR_TX_NORMAL_MODE                  0x00
#define LR_PAYLOAD_CRC_ON                  0x04
#define LR_PAYLOAD_CRC_OFF                 0x00

#define LR_STATIC_NODE                     0x00
#define LR_MOBILE_MODE                     0x08
#define LR_AGC_AUTO_ON                     0x04


#define LR_DIO0_RXDONE                    0x00
#define LR_DIO0_TXDONE                    0x40
#define LR_DIO0_CADDONE                   0x80
#define LR_DIO1_RXTIMEOUT                 0x00
#define LR_DIO1_FHSSCHANGECH              0x10
#define LR_DIO1_CADDETECTED               0x20
#define LR_DIO2_FHSSCHANGECH              0x00
#define LR_DIO3_CADDONE                   0x00
#define LR_DIO3_VALIDHEADER               0x01
#define LR_DIO3_CRCERROR                  0x02

#define LR_DIO4_CADDETECTED               0x00
#define LR_DIO4_PLLLOCK                   0x40
#define LR_DIO5_MODEREADY                 0x00
#define LR_DIO5_CLKOUT                    0x10


#define LR_EXT_CRYSTAL                    0x00
#define LR_TCXO_INPUT_ON                  0x10
#define LR_REGTCXO_RESERVED               0x09


#define LR_REGPADAC_RESERVED              0x80
#define LR_20DB_OUTPUT_ON                 0x07
#define LR_20DB_OUTPUT_OFF                0x04

  /**
   * @enum   eLORAOPMode_t
   */
typedef enum {
  eLORAOPModeSleep,
  eLORAOPModeStandBy,
  eLORAOPModeFsToTx,
  eLORAOPModeTx,
  eLORAOPModeFsToRx,
  eLORAOPModeRx
} eLORAOPMode_t;

class DFRobot_LoRa {
public:

  /**
   * @fn   DFRobot_LoRa
   * @brief Constructor.  
   * @param txEnPin  output pin,tx Antenna Switch,should be high while in tx mode
   * @param rxEnPin  output pin,rx Antenna Switch,should be high while in rx mode
   */
  DFRobot_LoRa(){}

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
   * @param length preamble length
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

protected:

  /**
   * @fn spiInit
   * @brief Initialise SPI.
   * @note Use standard Arduino SPI interface
   */
  void spiInit();

  /**
   * @fn pinInit
   * @brief Initialise other pin.
   */
  void pinInit();

  /**
   * @fn powerOnReset
   * @brief Power on lora module
   */
  void powerOnReset();

  /**
   * @fn setRxTimeOut
   * @brief Set rx time out.
   * @param symbTimeOut  actual timeout = symTimeout * (2^SF/BW)
   * @return  1 if ok, 0 otherwise
   */
  bool setRxTimeOut(uint16_t symbTimeOut);

  /**
   * @fn config
   * @brief Set initial parameters,called by init()
   * @param mode RF frequency=434Mhz,bandwidth = 500Hz,spreading factor=7,coding rate = 4/5,explict header mode
   * @return  true if ok, false otherwise
   */
  bool config();

  /**
   * @fn setFifoAddrPtr
   * @brief  both tx data and rx data store in same fifo.need to set start addr befor FIFO operating
   * @param addrReg register addr
   */
  void setFifoAddrPtr(uint8_t addrReg);

  /**
   * @fn enterRxMode
   * @brief  Enter rx mode
   */
  void enterRxMode();
  
  /**
   * @fn enterTxMode
   * @brief  Enter tx mode
   */
  void enterTxMode();
  
  /**
   * @fn writeFifo
   * @brief Write tx data in fifo
   * @param databuf  buf of tx data
   * @param length   length of tx data,less than 64 bytes
   */
  void writeFifo(uint8_t* databuf,uint8_t length);

  /**
   * @fn readFifo
   * @brief Read rx data in fifo
   * @param databuf  buf to save the rx data
   * @return length of rx data
   */
  uint8_t readFifo(uint8_t* databuf);

  /**
   * @fn setTxInterrupt
   * @brief Enable TxDone interrupt
   */
  void setTxInterrupt();

  /**
   * @fn setRxInterrupt
   * @brief Enable RxDone interrupt
   */
  void setRxInterrupt();
  
  /**
   * @fn readRegister
   * @brief read value of register
   * @param addr  register address
   */
  uint8_t readRegister(uint8_t addr);
  
  /**
   * @fn writeRegister
   * @brief set value of register
   * @param addr   register address
   * @param value  register value
  */
  void writeRegister(uint8_t addr, uint8_t value);
  
  /**
   * @fn readData
   * @brief Read buf trough SPI
   * @param addr  register address
   * @param	ptr  buf to save data
   * @param len  length to read, length>1
   */
  void readData(uint8_t addr, uint8_t *ptr, uint8_t len);
  
  /**
   * @fn writeData
   * @brief Write buf trough SPI
   * @param addr  register address
   * @param	ptr   data buf
   * @param len   length to write,length>1
   */
  void writeData(uint8_t addr, uint8_t *ptr, uint8_t len);

  /**
   * @fn writeRegBits
   * @brief Modifies a bit of a register
   * @param addr  register address
   * @param field  field
   * @param	data   data
   * @param offset   offset
   */
  void writeRegBits(uint8_t addr, uint8_t field, uint8_t data, uint8_t offset);

  /**
   * @fn setSymbTimeOut
   * @brief set Symb TimeOut 
   * @param t  time
   */
  void setSymbTimeOut(uint32_t t);

  /**
   * @fn writeBuffer
   * @brief write Buffer
   * @param addr  register address
   * @param	pBuf   data buf
   * @param len   length to write,length>1
   */
  void writeBuffer(uint8_t addr, uint8_t *pBuf, uint8_t len);

private:

  /*output pin,slave select pin*/
  uint8_t NSSPin;
  
  /*output pin,enter shutdown mode when driving low*/
  uint8_t NRESETPin;
  
  /*header mode*/
  uint8_t headerMode;
  uint8_t payloadLength;

};

#endif
