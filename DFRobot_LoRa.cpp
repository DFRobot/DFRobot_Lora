/*!
 * @file DFRobot_LoRa.cpp
 * @brief define DFRobot_LoRa class infrastructure, the implementation of basic methods
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangyang]
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-11-01
 * @url https://github.com/DFRobot/DFRobot_LoRa
 */
#include <DFRobot_LoRa.h>


void DFRobot_LoRa::spiInit()
{
  SPI.begin();
  pinMode(NSSPin, OUTPUT);
  digitalWrite(NSSPin, HIGH);
}

void DFRobot_LoRa::pinInit()
{
  pinMode(NRESETPin, OUTPUT);
  digitalWrite(NRESETPin, LOW);
}

bool DFRobot_LoRa::init(uint8_t _NSSPin, uint8_t _NRESETPin)
{
  NSSPin=_NSSPin;
  NRESETPin=_NRESETPin;
  pinInit();
  spiInit();
  powerOnReset();
  // check version
  uint8_t version = readRegister(LR_RegVERSION);
  if (version != 0x12) {
    return false;
  }
  config();
  return true;
}

void DFRobot_LoRa::powerOnReset()
{
  digitalWrite(NRESETPin, LOW); 
  delay(10);
  digitalWrite(NRESETPin, HIGH);
  delay(10);	
}

void DFRobot_LoRa::writeRegBits(uint8_t addr, uint8_t field, uint8_t data, uint8_t offset)
{
  uint8_t val = readRegister(addr);
  val &= ~(field << offset);
  val |= data << offset;
  writeRegister(addr, val);
}

void DFRobot_LoRa::setSymbTimeOut(uint32_t t)
{
  unsigned char RECVER_DAT[2];
  RECVER_DAT[0]=readRegister( 0x1e );
  RECVER_DAT[1]=readRegister( 0x1f );
  RECVER_DAT[0] = ( RECVER_DAT[0] & 0xfc ) | ( ( t >> 8 ) & ~0xfc );
  RECVER_DAT[1] = t & 0xFF;
  writeRegister( 0x1e, RECVER_DAT[0]);
  writeRegister( 0x1f, RECVER_DAT[1]);
}

void DFRobot_LoRa::writeBuffer(uint8_t addr, uint8_t *pBuf, uint8_t len)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NSSPin, LOW);
  SPI.transfer(addr);
  for(uint8_t i = 0; i < len; i ++)
    SPI.transfer(pBuf[i]);
  digitalWrite(NSSPin, HIGH);
  SPI.endTransaction();
}

const uint8_t pConfTable1[] = {0x6c, 0x80, 0x00, 0xcf, 0x09, 0x1b, 0x20, 0x00, 0x80, 0x00};
const uint8_t pConfTable2[] = {0x72, 0x73, 0xff, 0x00, 0x08, 0x01, 0xff, 0x00};

bool DFRobot_LoRa::config()
{

  writeRegBits(0x01, 0x07, 0, 0);  // sleep mode
  writeRegBits(0x01, 0x01, 1, 7);  // set to lora mode
  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeBuffer(0x06, (uint8_t*) pConfTable1, sizeof(pConfTable1));
  writeBuffer(0x1d, (uint8_t*) pConfTable2, sizeof(pConfTable2));
  writeRegister(0x26, 0x08);
  writeRegister(0x40, 0x00);
  writeRegister(0x41, 0x00);
  writeRegister(0x4d, 0x87);
  writeRegister(0x70, 0x10);

  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeRegister(0x26, 0x00);
  writeRegister(0x26, 0x02);
  writeRegister(0x3a, 0x7f);
  writeRegBits(0x31, 0x01, 1, 0);
  writeRegBits(0x1d, 0x0f, 9, 4);
  writeRegBits(0x1e, 0x0f, 11, 4);
  writeRegBits(0x1d, 0x07, 1, 1);
  setFrequency(868000000);

  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeRegBits(0x4d, 0x07, 0x07, 0);
  writeRegBits(0x09, 0x01, 1, 7);
  writeRegBits(0x09, 0x0f, 1, 0);
  setPreambleLen(16);
  rxInit();
  delay(5);
  return true;
}
bool DFRobot_LoRa::setFrequency(uint32_t freq)
{
  uint32_t frf;
  uint8_t reg[3];
  frf = freq / (LORA_XOSC / 524288);
  
  reg[0]=frf>>16&0xff;
  reg[1]=frf>>8&0xff;
  reg[2]=frf&0xff;
  
  writeRegister(LR_RegFrMsb,reg[0]);
  writeRegister(LR_RegFrMid,reg[1]);
  writeRegister(LR_RegFrLsb,reg[2]);	
  
  // read if the value has been in register
  if((reg[0]!=readRegister(LR_RegFrMsb))||(reg[1]!=readRegister(LR_RegFrMid))||(reg[2]!=readRegister(LR_RegFrLsb)))
    return false;
  return true;
}

bool DFRobot_LoRa::setRFpara(uint8_t BW,uint8_t CR,uint8_t SF,uint8_t payloadCRC)
{
  // check if the data is correct
  if(((BW&0x0f)!=0)||((BW>>8)>0x09))
    return false;
  if(((CR&0xf1)!=0)||((CR>>1)>0x04)||((CR>>1)<0x00))
    return false;
  if(((SF&0x0f)!=0)||((SF>>4)>12)||((SF>>4)<6))
    return false;
  if((payloadCRC&0xfb)!=0)
    return false;
  
  uint8_t temp;
  //SF=6 must be use in implicit header mode,and have some special setting
  if(SF==LR_SPREADING_FACTOR_6){
    headerMode=LR_IMPLICIT_HEADER_MODE;
    writeRegister(LR_RegModemConfig1,BW|CR|LR_IMPLICIT_HEADER_MODE);
    temp=readRegister(LR_RegModemConfig2);
    temp=temp&0x03;
    writeRegister(LR_RegModemConfig2,SF|payloadCRC|temp);	  
    // according to datasheet
    temp = readRegister(0x31);
    temp &= 0xF8;
    temp |= 0x05;
    writeRegister(0x31,temp);
    writeRegister(0x37,0x0C);	
  } else{
    temp=readRegister(LR_RegModemConfig2);
    temp=temp&0x03;
    writeRegister(LR_RegModemConfig1,BW|CR|headerMode);
    writeRegister(LR_RegModemConfig2,SF|payloadCRC|temp);
  }
  return true;
}

bool DFRobot_LoRa::setPreambleLen(uint16_t length)
{
  // preamble length is 6~65535
  if(length<6)
    return false;
  writeRegister(LR_RegPreambleMsb,length>>8);
   // the actual preamble len is length+4.25
  writeRegister(LR_RegPreambleLsb,length&0xff);  
  return true;	
}
bool DFRobot_LoRa::setHeaderMode(uint8_t mode)
{
  if(headerMode>0x01)
    return false;
  headerMode=mode;  
  uint8_t temp;
  // avoid overload the other setting
  temp=readRegister(LR_RegModemConfig1);
  temp=temp&0xfe;
  writeRegister(LR_RegModemConfig1,temp|mode);
  return true;
}
// in implict header mode, the payload length is fix len
// need to set payload length first in this mode
bool DFRobot_LoRa::setPayloadLength(uint8_t len)
{
  payloadLength=len;
  writeRegister(LR_RegPayloadLength,len);
  return true;
}
bool DFRobot_LoRa::setTxPower(uint8_t power)
{
  if(power>0x0f)
    return false;
  writeRegister(LR_RegPaConfig,LR_PASELECT_PA_POOST|0x70|power);
  return true;
}
// only valid in rx single mode
bool DFRobot_LoRa::setRxTimeOut(uint16_t symbTimeOut)
{
  //rxtimeout=symbTimeOut*(2^SF*BW)
  if((symbTimeOut==0)||(symbTimeOut>0x3ff))
    return false;  
  uint8_t temp;
  temp=readRegister(LR_RegModemConfig2);
  temp=temp&0xfc;
  writeRegister(LR_RegModemConfig2,temp|(symbTimeOut>>8&0x03));
  writeRegister(LR_RegSymbTimeoutLsb,symbTimeOut&0xff);
  return true;
}
// RSSI[dBm]=-137+rssi value
uint8_t DFRobot_LoRa::readRSSI(uint8_t mode)
{
  if(!mode){//read current rssi
    return readRegister(LR_RegRssiValue);
  } else{// read rssi of last packet received
    return readRegister(LR_RegPktRssiValue);
  }
}

bool DFRobot_LoRa::rxInit()
{
  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeRegBits(0x31, 0x01, 0x01, 5);
  writeRegister(0x11, 0x9e);
  writeRegister(0x24, 0);
  writeRegister(0x40, 0x00);
  writeRegister(0x41, 0x00);
  writeRegister(0x0d, readRegister(0x0f));
  writeRegBits(0x01, 0x07, 0x05, 0);
  return true;
}
bool DFRobot_LoRa::sendPackage(uint8_t* sendbuf,uint8_t sendLen)
{
  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeRegister(0x24, 0);
  writeRegister(0x11, 0xf7);
  writeRegister(0x22, sendLen);
  writeRegister(0x0e, 0x00);
  writeRegister(0x0d, 0x00);
  writeFifo(sendbuf, sendLen);
  writeRegBits(0x01, 0x07, 0x03, 0);
  uint16_t txTimer;

  // you should make sure the tx timeout is greater than the max time on air
  txTimer=2000;
  while(txTimer--){
    // wait for txdone
    if(waitIrq(LR_TXDONE_MASK)){
      idle();
      clearIRQFlags();
      return true;
    }
    delay(1);
  }
  // if tx time out , reset lora module
  init(NSSPin, NRESETPin);
  return false;
}
uint8_t DFRobot_LoRa::receivePackage(uint8_t* recvbuf)
{
  // read data from fifo
  return readFifo(recvbuf);	
}
bool DFRobot_LoRa::waitIrq(uint8_t irqMask)
{
  uint8_t flag;
  // read irq flag
  flag=readRegister(LR_RegIrqFlags);
  // if irq flag was set
  if(irqMask == LR_RXDONE_MASK) {
    if(flag & LR_RXPCRCERROR_MASK) {
      Serial.println("LoRa crc error");
      return false;
    }
  }
  if(flag&irqMask)
    return true;
  return false;
}
void DFRobot_LoRa::setFifoAddrPtr(uint8_t addrReg)
{
  uint8_t addr;
  // read BaseAddr     
  addr = readRegister(addrReg);		
  // BaseAddr->FifoAddrPtr     
  writeRegister(LR_RegFifoAddrPtr,addr);	
}
void DFRobot_LoRa::enterRxMode()
{
  writeRegBits(0x01, 0x07, 0x05, 0);
}
void DFRobot_LoRa::enterTxMode()
{
  writeRegBits(0x01, 0x07, 0x03, 0);
}
void DFRobot_LoRa::idle()
{
  writeRegister(LR_RegOpMode,0x89);
}
void DFRobot_LoRa::sleep()
{
  writeRegister(LR_RegOpMode,0x89);
  writeRegister(LR_RegOpMode,0x88);
}
void DFRobot_LoRa::writeFifo(uint8_t* databuf,uint8_t length)
{
  setFifoAddrPtr(LR_RegFifoTxBaseAddr);
  writeData(0x00,databuf,length);		
}
uint8_t DFRobot_LoRa::readFifo(uint8_t* databuf)
{
  uint8_t readLen;
  setFifoAddrPtr(LR_RegFifoRxCurrentaddr);
  readLen=payloadLength;
  readData(0x00, databuf, readLen);
  return readLen;
}
void DFRobot_LoRa::setTxInterrupt()
{
  writeRegister(LR_RegDIOMAPPING1,LR_DIO0_TXDONE);
  writeRegister(LR_RegIrqFlagsMask,0xff^LR_TXDONE_MASK);
}
void DFRobot_LoRa::setRxInterrupt()
{
  //DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
  writeRegister(LR_RegDIOMAPPING1,LR_DIO0_RXDONE);	
  // enable rxdone irq
  writeRegister(LR_RegIrqFlagsMask,0xff^(LR_RXDONE_MASK | LR_RXPCRCERROR_MASK));
}
void DFRobot_LoRa::clearIRQFlags()
{
  writeRegister(LR_RegIrqFlags,0xff);
}
// SPI read register
uint8_t DFRobot_LoRa::readRegister(uint8_t addr)
{
  uint8_t data; 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NSSPin, LOW);
  // write register address
  SPI.transfer(addr);	
  // read register value
  data = SPI.transfer(0);			
  digitalWrite(NSSPin, HIGH);
  SPI.endTransaction();
  return(data);
}

// SPI write register
void DFRobot_LoRa::writeRegister(uint8_t addr, uint8_t value)                
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NSSPin, LOW);
  // write register address
  SPI.transfer(addr|LORA_SPI_WNR);	
  // write register value
  SPI.transfer(value);			
  digitalWrite(NSSPin, HIGH);
  SPI.endTransaction();
}
void DFRobot_LoRa::readData(uint8_t addr, uint8_t *ptr, uint8_t len)
{
  uint8_t i;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // length>1,use burst mode
  if(len<=1){
    return;
  }else{
    digitalWrite(NSSPin, LOW);
    SPI.transfer(addr);
    for(i=0;i<len;i++){
      ptr[i] = SPI.transfer(0);
    }
    digitalWrite(NSSPin, HIGH);
  }
  SPI.endTransaction();
}

void DFRobot_LoRa::writeData(uint8_t addr, uint8_t *ptr, uint8_t len)
{ 
  uint8_t i;	
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // length>1,use burst mode
  if(len<=1){
   return;
  } else {
    digitalWrite(NSSPin, LOW);
    SPI.transfer(addr|LORA_SPI_WNR);
    for(i=0;i<len;i++){
      SPI.transfer(ptr[i]);
    }
    digitalWrite(NSSPin, HIGH);
  }
  SPI.endTransaction();
}
