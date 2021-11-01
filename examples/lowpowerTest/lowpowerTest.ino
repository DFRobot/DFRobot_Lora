/*!
 * @file sleepTest.ino
 * @brief DFRobot's sleep mode
 * @n [Get the module here]
 * @n This example is the main configuration of lora mode and enter low power consumption.
 * @n [Connection and Diagram]
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangyang]
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-11-01
 * @url https://github.com/DFRobot/DFRobot_LoRa
 */

#include <DFRobot_LoRa.h>

DFRobot_LoRa lora;

/* The default pin:
 *		SS:D4
 *		RST:D2 (If you are using the FireBeetle Board-ESP8266 motherboard controller, the RST defaults to D3)	
 */

void setup()
{
	Serial.begin(115200);
	
  while(!lora.init()) {
		Serial.println("Starting LoRa failed!");
    delay(2000);
	}
	
	// frequency:the range is 137~1020Mhz for lora1276 ,137~525Mhz for lora1278
  lora.setFrequency(433000000);
	/*
	BW	        bandwidth , range from 0 to 9
	CR	        coding rate , range from 4/5 to 4/8---2,4,6,8
	SF	        spreading factor , range from 6 to 12
	CRC	        LR_PAYLOAD_CRC_ON:enable CRC,LR_PAYLOAD_CRC_OFF:disable CRC
	*/
	lora.setRFpara(7, 2, 11, LR_PAYLOAD_CRC_ON);
	// preamble length is 6~65535
	lora.setPreambleLen(12);
	// mode LR_IMPLICIT_HEADER_MODE or LR_EXPLICIT_HEADER_MODE
	lora.setHeaderMode(LR_EXPLICIT_HEADER_MODE);
	// power level,0 to 15
	lora.setTxPower(15);
	
	lora.sleep();  // Enter low power consumption
}
void loop()
{
	
}
