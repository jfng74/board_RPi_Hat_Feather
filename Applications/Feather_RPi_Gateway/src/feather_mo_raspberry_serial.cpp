#define VERSION 13_10_2016

#include "Arduino.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <FanCompostMsg.h>
//#include <Adafruit_FeatherOLED.h>

// Ajout de commentaires 

// m0 & 32u4 feathers
#define VBATPIN A7


/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

//Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t incomingByte = 0;

float getBatteryVoltage();
void parseRF_data();
void parseSerial_data(void);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t buf_out[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len;
uint8_t Serial1_input_buffer[60];

int rx_count=0;
int tx_count=0;

void setup() {
	pinMode(RFM95_RST, OUTPUT);
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	digitalWrite(RFM95_RST, HIGH);
//	Serial1.begin(9600);
//
//	oled.init();
//  oled.setBatteryVisible(true);
//	oled.clearDisplay();
//	oled.println("OLED_SETUP : 1");
//	oled.println("OLED_SETUP : 2");
//	oled.println("OLED_SETUP : 3");
//	oled.println("OLED_SETUP : 4");
//	oled.display();
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
	rf95.setTxPower(23, false);

	Serial.print("Setup ok!");
}

void loop() {
	// put your main code here, to run repeatedly:


  // get the current voltage of the battery from
  // one of the platform specific functions below
//  float battery = getBatteryVoltage();

  // update the battery icon
//  oled.setBattery(battery);
//  oled.renderBattery();

	if (rf95.available()){
		Serial.println("*******************************************");
		Serial.println("RF95 data available");
		if (rf95.recv(buf, &len)) {
			Serial.println("RF95 Receive");
			Serial.print("len buffer : ");
			len = sizeof(buf);
			Serial.println(len);
			parseRF_data();
			Serial1.write((char*)buf, len);

		}
		else
		{
			Serial.println("RF95 Receive : Failed");
		}
		Serial.println("*******************************************");
	}
	if(Serial1.available()) {
		incomingByte = Serial1.read();
		if(incomingByte == FEATHER_MSG_HEADER){
			int read_bytes=0;
			buf_out[0]=incomingByte;
			read_bytes = Serial1.readBytes((char*)Serial1_input_buffer, 255);
			Serial.println("###########################################");
			Serial.print("Bytes lu : ");Serial.println(read_bytes+1);

			for(int i=0;i<read_bytes;i++){
				buf_out[i+1]=Serial1_input_buffer[i];
				Serial.print("[");Serial.print(buf_out[i+1]);Serial.print("]");
			}
			Serial.println(" ");
			Serial.println("Envoie des donnees sur le rf");


			parseSerial_data();
			rf95.send(buf_out, read_bytes+1);
			rf95.waitPacketSent();
			Serial.println("###########################################");
			digitalWrite(13, LOW);
		}
		else{
			int read_bytes=0;
			Serial.print("Read all false byte : ");
			read_bytes = Serial1.readBytes((char*)Serial1_input_buffer, 255);
			Serial.println(read_bytes);
		}
	}
}

float getBatteryVoltage() {

	float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;

}

void parseSerial_data(void){
//  byte float_array[4];
//  byte u16_array[2];
//  byte uint32_array[4];
//  int16_t node_id;
//  uint16_t conductivite;
//	float t_1,t_2,t_3,h_1,batt_voltage,pression;
  if (buf_out[0]==FEATHER_MSG_HEADER){
    Serial.println("parseSerial_data() : FEATHER_MSG_HEADER");
  }
  if (buf_out[0]==FEATHER_MSG_HEADER && buf_out[1]==FEATHER_MSG_SEND_ALL_TEMP){
    Serial.println("parseSerial_data() : FEATHER_MSG_SEND_ALL_TEMP");
		if(buf_out[3] == SEND_ALL_TEMP){
			Serial.println("FEATHER_MSG_SEND_ALL_TEMP : FEATHER_MSG_SEND_ALL_TEMP");
		}
  }
	else if (buf_out[0]==FEATHER_MSG_HEADER && buf_out[1]==FEATHER_MSG_SEND_SSR_NODE_CFG){
		Serial.println("parseSerial_data() : FEATHER_MSG_SEND_SSR_NODE_CFG");
	}
}


void parseRF_data(void){
//  byte float_array[4];
//  byte u16_array[2];
//  byte uint32_array[4];
//  int16_t node_id;
//  uint16_t conductivite;
//	float t_1,t_2,t_3,h_1,batt_voltage,pression;

  if (buf[0]==FEATHER_MSG_HEADER){
    Serial.println("parseRF_data() : FEATHER_MSG_HEADER");
  }
  if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_NODE_READY){
    Serial.println("parseRF_data() : FEATHER_MSG_NODE_READY");
  }
	else if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SSR_READY){
		Serial.println("parseRF_data() : FEATHER_MSG_SSR_READY");
	}
	else if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_READY_FOR_COMMANDS){
		Serial.println("parseRF_data() : FEATHER_MSG_READY_FOR_COMMANDS");
	}
	else if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_COMPOST_NODE_DATA){
		Serial.println("parseRF_data() : FEATHER_MSG_COMPOST_NODE_DATA");
	}
	else if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_RESPONSE_DATA){
		Serial.println("parseRF_data() : FEATHER_MSG_RESPONSE_DATA");
		if(buf[3]==SEND_ALL_CFG)
			Serial.println("FEATHER_MSG_RESPONSE_DATA : SEND_ALL_CFG");
	}
}
