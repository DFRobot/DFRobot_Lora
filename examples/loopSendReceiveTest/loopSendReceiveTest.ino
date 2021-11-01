/*!
 * @file loopSendReceiveTest.ino
 * @brief DFRobot's Loop send-receive data
 * @n [Get the module here]
 * @n This example is looped to send and receive.
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

#define MASTER_MODE

#ifdef MASTER_MODE
uint8_t sendBuf[] = "AAAAA!";
uint8_t flag = 1;
#define DELAY_MS  1
#else
uint8_t sendBuf[] = "BBBBB!";
uint8_t flag = 0;
#define DELAY_MS	500
#endif
uint8_t sendCounter = 0;
uint8_t len;
uint8_t rxBuf[32];

DFRobot_LoRa lora;

/* The default pin:
 *		SS:D4
 *		RST:D2 (If you are using the FireBeetle Board-ESP8266 motherboard controller, the RST defaults to D3)	
 */

void setup()
{
	Serial.begin(115200);
	
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.println("Receiver Test");
	
	while(!lora.init()) {
		Serial.println("Starting LoRa failed!");
    delay(2000);
	}
	
	if(flag){
		lora.rxInit();
	}
}
void loop()
{
	if(flag == 0) {  
		Serial.print("Sending packet: ");
    Serial.println(sendCounter);
		
		lora.sendPackage(sendBuf, sizeof(sendBuf)-1);
		lora.rxInit();    
		flag = 1;             
    sendCounter++;
	} else {        
    if(lora.waitIrq()) {
			lora.clearIRQFlags();
			len = lora.receivePackage(rxBuf); 
			flag = 0;            
			Serial.write(rxBuf, len);    
			Serial.println();
			
			Serial.print("with RSSI ");
			Serial.println(lora.readRSSI());
			
			static uint8_t i;
			i = ~i;
			digitalWrite(LED_BUILTIN, i);
		}
	}

	delay(DELAY_MS);
}
