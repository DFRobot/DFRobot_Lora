/*!
 * @file receiverTest.ino
 * @brief DFRobot's received data
 * @n [Get the module here] http://wiki.dfrobot.com.cn/index.php?title=(SKU:TEL0125)FireBeetle_Covers-LoRa_Radio_868MHz
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

#define RX_LEN    11

uint8_t len;
uint8_t rxBuf[RX_LEN];

uint16_t    sendCounter = 0;
uint16_t    recvCounter = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Receiver Test");

  while(!lora.init()) {
    Serial.println("Starting LoRa failed!");
    delay(2000);
  }
  lora.setPayloadLength(RX_LEN);  // max len is 254
  //lora.setFrequency(433000000);  // for 433mhz module
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
    Serial.print("recvCounter ");
    Serial.println(recvCounter ++);
    static uint8_t i;
    i = ~i;
    digitalWrite(LED_BUILTIN, i);
  }
}
