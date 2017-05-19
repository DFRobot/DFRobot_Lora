/*!
 * @file receiverTest.ino
 * @brief DFRobot's received data
 * @n [Get the module here]
 * @n This example is receive.
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

DFRobot_LoRa lora;

uint8_t len;
uint8_t rxBuf[32];

void setup()
{
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
	Serial.println("Receiver Test");

	if(!lora.init()) {
		Serial.println("Starting LoRa failed!");
		while (1);
	}

	lora.rxInit();    
}
void loop()
{
	if(lora.waitIrq()) {   // wait for RXDONE interrupt
		lora.clearIRQFlags();
		len = lora.receivePackage(rxBuf);  // receive data
		Serial.write(rxBuf, len);    
		Serial.println();
		lora.rxInit();    // wait for packet from master
		
		// print RSSI of packet
    	Serial.print("with RSSI ");
    	Serial.println(lora.readRSSI());

		static uint8_t i;
		i = ~i;
		digitalWrite(LED_BUILTIN, i);
	}
}
