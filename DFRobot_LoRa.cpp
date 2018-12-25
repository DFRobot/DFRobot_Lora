/*!
 * @file DFRobot_LoRa.h
 * @brief LORO
 * @n lora mode
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yangyang]
 * @version  V1.0
 * @date  2017-04-10
 */

#include <DFRobot_LoRa.h>

DFRobot_LoRa::DFRobot_LoRa()
{
}
void DFRobot_LoRa::spiInit()
{
  SPI.begin();
	//init slave select pin
  pinMode(NSSPin, OUTPUT);
  digitalWrite(NSSPin, HIGH);

//	// depends on DFRobot_LoRa spi timing
//  SPI.setBitOrder(MSBFIRST);
//  // too fast may cause error
//  SPI.setClockDivider(SPI_CLOCK_DIV32);
//  SPI.setDataMode(SPI_MODE0);
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

	// reset lora
	
	// Set RF parameter,like frequency,data rate etc
  config(LR_Mode_RXCONTINUOUS);

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
  uint8_t   val = readRegister(addr);
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

const uint8_t   pConfTable1[] = {0x6c, 0x80, 0x00, 0xcf, 0x09, 0x1b, 0x20, 0x00, 0x80, 0x00};
const uint8_t   pConfTable2[] = {0x72, 0x73, 0xff, 0x00, 0x08, 0x01, 0xff, 0x00};

bool DFRobot_LoRa::config(uint8_t mode)
{
//  writeRegister(0x12, 0xff);
//  delay(1);
//  writeRegister(0x01, 0x00);
//  delay(1);
//  writeRegister(0x4b, 0x09);
//  writeRegister(0x01, 0x80);
//  writeRegister(0x06, 0xd9);
//  writeRegister(0x07, 0x20);
//  writeRegister(0x08, 0x00);
//  writeRegister(0x09, 0xff);
//  writeRegister(0x0b, 0x0b);
//  writeRegister(0x0c, 0x23);
//  writeRegister(0x1d, 0x72);
//  writeRegister(0x1e, 0xc7);
//  writeRegister(0x26, 0x08);
//  writeRegister(0x1f, 0xff);
//  writeRegister(0x20, 0x00);
//  writeRegister(0x21, 0x0c);
//  writeRegister(0x41, 0x01);
//  writeRegister(0x01, 0x01);

////  headerMode=LR_EXPLICIT_HEADER_MODE;
////  setHeaderMode(headerMode);

//  if(mode == LR_Mode_TX) {
//    writeRegister(0x4d, 0x87); // * 0x84
//    writeRegister(0x24, 0x00); // * 0xff
//    writeRegister(0x40, 0x41); // * 0x01
//    writeRegister(0x12, 0xff);
//    writeRegister(0x11, 0xf7); // * 0x3f
//    writeRegister(0x22, 0x0d);
//    readRegister(0x0e);
//    writeRegister(0x0d, 0x80);
//    writeRegister(0x01, 0x01);
//  } else {
//    writeRegister(0x4d, 0x84); // * 0x84
//    writeRegister(0x24, 0xff); // * 0xff
//    writeRegister(0x40, 0x01); // * 0x01
//    writeRegister(0x11, 0x3f); // * 0x3f
//    writeRegister(0x12, 0xff);
//    writeRegister(0x22, 0x0d);
//    readRegister(0x0f);
//    writeRegister(0x0d, 0x00);
//    writeRegister(0x01, 0x0d);
//  }

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

//  // In setting mode, RF module should turn to sleep mode
//  // low frequency mode£¬sleep mode
//  writeRegister(LR_RegOpMode,LR_Mode_SLEEP|LORA_FREQUENCY_BAND);
//  // wait for steady
//  delay(5);

//  // external Crystal
//  writeRegister(LR_RegTCXO,LR_EXT_CRYSTAL|LR_REGTCXO_RESERVED);
//  // set lora mode
//  writeRegister(LR_RegOpMode,LR_LongRangeMode_LORA|LORA_FREQUENCY_BAND);
//  // RF frequency 434M
//  setFrequency(434000000);
//  // max power ,20dB
//  setTxPower(0x0f);
//  // close ocp
//  writeRegister(LR_RegOcp,LR_OCPON_OFF|0x0B);
//  // enable LNA
//  writeRegister(LR_RegLna,LR_LNAGAIN_G1|LR_LNABOOSTHF_1);

//  headerMode=LR_EXPLICIT_HEADER_MODE;

//  // bandwidth = 500Hz, spreading factor=7,
//  // coding rate = 4/5,implict header mode
//  setHeaderMode(headerMode);
//  setRFpara(LR_BW_125k,LR_CODINGRATE_1p5,LR_SPREADING_FACTOR_11,LR_PAYLOAD_CRC_ON);
//  // LNA
//  writeRegister(LR_RegModemConfig3,LR_MOBILE_MODE);
//  // max rx time out
//  setRxTimeOut(0x3ff);
//  // preamble 12+4.25 bytes
//  setPreambleLen(12);

//  // 20dBm on PA_BOOST pin
//  writeRegister(LR_RegPADAC,LR_REGPADAC_RESERVED|LR_20DB_OUTPUT_ON);
//   // no hopping
//  writeRegister(LR_RegHopPeriod,0x00);

//  // DIO5=ModeReady,DIO4=CadDetected
//  writeRegister(LR_RegDIOMAPPING2,LR_DIO4_CADDETECTED|LR_DIO5_MODEREADY);
//   // standby mode
//  writeRegister(LR_RegOpMode,LR_LongRangeMode_LORA|LR_Mode_STBY|LORA_FREQUENCY_BAND);

//  // default payload length is 10 bytes in implicit mode
//  setPayloadLength(10);

  delay(5);
  return true;
}
bool DFRobot_LoRa::setFrequency(uint32_t freq)
{
	uint32_t frf;
	uint32_t temp1;
	uint32_t temp2;
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
	if(SF==LR_SPREADING_FACTOR_6)		
	{
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
	}
	else
	{
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
}
// in implict header mode, the payload length is fix len
// need to set payload length first in this mode
bool DFRobot_LoRa::setPayloadLength(uint8_t len)
{
	payloadLength=len;
	writeRegister(LR_RegPayloadLength,len);
}
bool DFRobot_LoRa::setTxPower(uint8_t power)
{
	if(power>0x0f)
		return false;
	writeRegister(LR_RegPaConfig,LR_PASELECT_PA_POOST|0x70|power);
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
}
// RSSI[dBm]=-137+rssi value
uint8_t DFRobot_LoRa::readRSSI(uint8_t mode)
{
	if(!mode)	//read current rssi
	{
		return readRegister(LR_RegRssiValue);
	}
	else			// read rssi of last packet received
		return readRegister(LR_RegPktRssiValue);
}

bool DFRobot_LoRa::rxInit()
{
//  if(headerMode==LR_IMPLICIT_HEADER_MODE)
//    setPayloadLength(payloadLength);
//  setRxInterrupt();	// enable RxDoneIrq
//  clearIRQFlags();		// clear irq flag
//  setFifoAddrPtr(LR_RegFifoRxBaseAddr);	// set FIFO addr
//  enterRxMode();		// start rx

  writeRegBits(0x01, 0x07, 1, 0);  // stand by mode
  writeRegBits(0x31, 0x01, 0x01, 5);
  writeRegister(0x11, 0x9e);
  writeRegister(0x24, 0);
  writeRegister(0x40, 0x00);
  writeRegister(0x41, 0x00);
  writeRegister(0x0d, readRegister(0x0f));
  writeRegBits(0x01, 0x07, 0x05, 0);

}
bool DFRobot_LoRa::sendPackage(uint8_t* sendbuf,uint8_t sendLen)
{
	uint8_t temp;
	
//  setTxInterrupt();	// enable TxDoneIrq
//  clearIRQFlags();		// clear irq flag
//  writeFifo(sendbuf,sendLen);
//  enterTxMode();

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
	while(txTimer--)
	{
		// wait for txdone
    if(waitIrq(LR_TXDONE_MASK))
		{
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
	// enter rx continuous mode
//  writeRegister(LR_RegOpMode,LR_LongRangeMode_LORA|LR_Mode_RXCONTINUOUS|LORA_FREQUENCY_BAND);
  writeRegBits(0x01, 0x07, 0x05, 0);
}
void DFRobot_LoRa::enterTxMode()
{
	// enter tx mode
//  writeRegister(LR_RegOpMode,LR_LongRangeMode_LORA|LR_Mode_TX|LORA_FREQUENCY_BAND);
  writeRegBits(0x01, 0x07, 0x03, 0);
}
void DFRobot_LoRa::idle()
{
	// enter Standby mode
	writeRegister(LR_RegOpMode,0x89);  	
}
void DFRobot_LoRa::sleep()
{
	// enter sleep mode
	writeRegister(LR_RegOpMode,0x89); 
	writeRegister(LR_RegOpMode,0x88);  	
}
void DFRobot_LoRa::writeFifo(uint8_t* databuf,uint8_t length)
{
	// set packet length
//  if(headerMode==LR_EXPLICIT_HEADER_MODE)
//    writeRegister(LR_RegPayloadLength,length);
	// set Fifo addr
  setFifoAddrPtr(LR_RegFifoTxBaseAddr);
	// fill data into fifo
	writeData(0x00,databuf,length);		
}
uint8_t DFRobot_LoRa::readFifo(uint8_t* databuf)
{
	uint8_t readLen;

	// set Fifo addr
  setFifoAddrPtr(LR_RegFifoRxCurrentaddr);
	// read length of packet  
//  if(headerMode==LR_IMPLICIT_HEADER_MODE)
    readLen=payloadLength;
//  else
//    readLen = readRegister(LR_RegRxNbBytes);
	// read from fifo
	readData(0x00, databuf, readLen);		

	return readLen;
}
void DFRobot_LoRa::setTxInterrupt()
{
	// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader
	writeRegister(LR_RegDIOMAPPING1,LR_DIO0_TXDONE); 	
	// enable txdone irq
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
	if(len<=1)			
		return;
	else
	{
		digitalWrite(NSSPin, LOW);
		SPI.transfer(addr);
		for(i=0;i<len;i++)
			ptr[i] = SPI.transfer(0);
		digitalWrite(NSSPin, HIGH);
	}
  SPI.endTransaction();
}
void DFRobot_LoRa::writeData(uint8_t addr, uint8_t *ptr, uint8_t len)
{ 
	uint8_t i;	
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	// length>1,use burst mode
	if(len<=1)			
		return;
	else  
	{   
		digitalWrite(NSSPin, LOW);      
    SPI.transfer(addr|LORA_SPI_WNR);
		for(i=0;i<len;i++)
			SPI.transfer(ptr[i]);
		digitalWrite(NSSPin, HIGH); 
	}
  SPI.endTransaction();
}
