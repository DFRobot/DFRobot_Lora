# Lora Module
![SVG1](https://raw.githubusercontent.com/DFRobot/binaryfiles/master/TEL0122/TEL0122svg1.png)

## Lora Library for Arduino

Provides an Arduino library for FireBeetle Covers-LoRa Radio

---------------------------------------------------------

### Initialize the function

	bool init(uint8_t NSSPin = NSS_PIN, uint8_t NRESETPin = RESET_PIN);

### Initialise rx mode

	bool rxInit();

### Sending packet through RF

	bool sendPackage(uint8_t* sendbuf,uint8_t sendLen); // length of data to send,less than 64 bytes

### Receive packet from RF

    uint8_t receivePackage(uint8_t* recvbuf);

### Inquire interrupt

    bool waitIrq(uint8_t irqMask = LR_RXDONE_MASK); // LR_RXDONE_MASK and LR_TXDONE_MASK

### Clear interrupt

    void clearIRQFlags();
	
### Enter low power consumption

    void sleep();
	
### Enter standby mode

    void idle();
	
### Read rssi

    uint8_t readRSSI(uint8_t mode = 0);
 
* @n [Get the module here]
* @n This is lora's receive and send test.
* @n [Connection and Diagram]
*
* @copyright	[DFRobot](http://www.dfrobot.com), 2016
* @copyright	GNU Lesser General Public License
*
* @author [yangyang]
* @version  V1.0
* @date  2017-04-10
