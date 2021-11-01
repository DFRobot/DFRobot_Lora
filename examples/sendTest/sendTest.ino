/*!
 * @file sendTest.ino
 * @brief DFRobot's send data
 * @n [Get the module here]
 * @n This example is send.
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

uint16_t sendCounter = 0;
uint8_t sendBuf[] = "HelloWorld!";

/* The default pin:
 *		SS:D4
 *		RST:D2 (If you are using the FireBeetle Board-ESP8266 motherboard controller, the RST defaults to D3)	
 */

void setup()
{
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  while(!lora.init()) {
    Serial.println("Starting LoRa failed!");
    delay(2000);
  }
  //lora.setFrequency(433000000);  // for 433mhz module
  Serial.println("LoRa begin successed");
}

void loop()
{
  static int blink;
  Serial.print("Sending packet: ");
  Serial.println(sendCounter);
  digitalWrite(LED_BUILTIN, blink);
  blink = ~blink;

  // send packet
  lora.sendPackage(sendBuf, sizeof(sendBuf)); // sending data, max len is 254
  lora.idle();    // turn to standby mode

  sendCounter++;
#if 0
if(counter%10 == 0) {
  lora.sleep();
  delay (5000);// sleep 5 seconds
}
#endif

  delay(2000);
}
