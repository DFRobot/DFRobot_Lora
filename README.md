LoRa 
---------------------------------------------------------

<br>
### Initialize the function

	bool init(uint8_t NSSPin = NSS_PIN, uint8_t NRESETPin = RESET_PIN);

<br>
### Initialise rx mode

	bool rxInit();

<br>
### Sending packet through RF

	bool sendPackage(uint8_t* sendbuf,uint8_t sendLen); // length of data to send,less than 64 bytes

<br>
### Receive packet from RF

    uint8_t receivePackage(uint8_t* recvbuf);

<br>
### Inquire interrupt

    bool waitIrq(uint8_t irqMask = LR_RXDONE_MASK); // LR_RXDONE_MASK and LR_TXDONE_MASK

<br>
### Clear interrupt

    void clearIRQFlags();
	
<br>
### Enter low power consumption

    void sleep();
	
<br>
### Enter standby mode

    void idle();
	
<br>
### Read rssi

    uint8_t readRSSI(uint8_t mode = 0);
 
<br>
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
 *
