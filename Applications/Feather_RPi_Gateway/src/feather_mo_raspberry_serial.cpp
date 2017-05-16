#define VERSION 13_10_2016

#include "Arduino.h"
#include <SPI.h>
#include <RH_RF95.h>

#define FEATHER_MSG_HEADER 0xaa

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t incomingByte = 0;

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t buf_out[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len;
uint8_t Serial1_input_buffer[60];


void setup() {
	pinMode(RFM95_RST, OUTPUT);
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	digitalWrite(RFM95_RST, HIGH);
//	Serial1.begin(9600);
	Serial1.begin(9600);
//	Serial1.println("setup()");
	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);
	while (!rf95.init()) {
	//Serial11.println("LoRa radio init failed");
		while (1);
	}

	//Serial1.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		//Serial1.println("setFrequency failed");
		while (1);
	}
	//Serial1.print("Set Freq to: ");
	//Serial1.println(RF95_FREQ);

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(5, false);

	Serial1.print("Allo");
}

void loop() {
	// put your main code here, to run repeatedly:
	if (rf95.available()){
		// Should be a message for us now
		//	  .print("Buffer length : ");
		len = sizeof(buf);
		//	  Serial1.println(len);


		if (rf95.recv(buf, &len)) {
//			digitalWrite(13, HIGH);
			Serial1.write((char*)buf, len);
//			digitalWrite(13, LOW);
			/*
			Serial1.print("Buffer length : ");
			Serial1.println(len);

			for(int i=0;i<len;i++){
			  Serial1.println(buf[i],HEX);
			}
			  */
		}
		else
		{
			//Receive failed
		}
	}
	while (Serial1.available()) {
		incomingByte = Serial1.read();
//		Serial1.println("Incoming bytes...");
		if(incomingByte == FEATHER_MSG_HEADER){
//			Serial1.println("FEATHER_MSG_HEADER");
			digitalWrite(13, HIGH);
			buf_out[0]=incomingByte;
			Serial1.readBytes((char*)Serial1_input_buffer, 8);
			for(int i = 0;i<8;i++){
				buf_out[i+1]=Serial1_input_buffer[i];
			}

//			for(int i=0;i<9;i++){
//				Serial1.print(buf_out[i],HEX);
//			}
//			Serial1.println("Fin reception");
			rf95.send(buf_out, 9);
			rf95.waitPacketSent();
//			delay(100);
			digitalWrite(13, LOW);
		}
	}
}
