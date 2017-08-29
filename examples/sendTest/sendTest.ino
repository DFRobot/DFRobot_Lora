/*!
 * @file sendTest.ino
 * @brief DFRobot's send data
 * @n [Get the module here]
 * @n This example is send.
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

uint8_t counter = 0;
uint8_t sendBuf[] = "HelloWorld!";

/* The default pin:
 *		SS:D4
 *		RST:D2 (If you are using the FireBeetle Board-ESP8266 motherboard controller, the RST defaults to D3)	
 */

void setup()
{
	Serial.begin(115200);
	
	while(!lora.init()) {
		Serial.println("Starting LoRa failed!");
		delay(100);
	}
}
void loop()
{
	Serial.print("Sending packet: ");
	Serial.println(counter);

	// send packet
	lora.sendPackage(sendBuf, 11); // sending data
	lora.idle();    // turn to standby mode

	counter++;
#if 0
	if(counter%10 == 0) {
		lora.sleep();
		delay (5000);// sleep 5 seconds
	}
#endif

	delay(500);
}
