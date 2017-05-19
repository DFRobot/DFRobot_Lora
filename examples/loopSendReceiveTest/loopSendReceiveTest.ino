/*!
 * @file loopSendReceiveTest.ino
 * @brief DFRobot's Loop send-receive data
 * @n [Get the module here]
 * @n This example is looped to send and receive.
 * @n [Connection and Diagram]
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yangyang]
 * @version  V1.0
 * @date  2017-04-10
 */

#include <DFRobot_LoRa.h>

#define MASTER_MODE

#ifdef MASTER_MODE
uint8_t sendBuf[] = "AAAAA!";
uint8_t flag = 1;
#define DELAY_MS	1
#else
uint8_t sendBuf[] = "BBBBB!";
uint8_t flag = 0;
#define DELAY_MS	500
#endif
uint8_t counter = 0;
uint8_t len;
uint8_t rxBuf[32];

DFRobot_LoRa lora;

/* The default pin:
/		SS:D2
/		RST:D3
*/

void setup()
{
	Serial.begin(115200);
	
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.println("Receiver Test");
	
	if(!lora.init()) {
		Serial.println("Starting LoRa failed!");
		while (1);
	}
	
	if(flag){
		lora.rxInit();
	}
}
void loop()
{
	if(flag == 0) {  
		Serial.print("Sending packet: ");
		Serial.println(counter);
		
		lora.sendPackage(sendBuf, sizeof(sendBuf));
		lora.rxInit();    
		flag = 1;             
		counter++;
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
