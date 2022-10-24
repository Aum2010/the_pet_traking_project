/*
  LoRa Simple Gateway/Node Exemple

  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

  created 05 August 2018
  by Luiz H. Cassettari
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>
#include "CRC8.h"
#include "CRC.h"
#include "time.h"


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;
const int   daylightOffset_sec = 7 * 3600;

//#define ENABLE_FIREBASE_WIFI  

#define FIREBASE_HOST ""
#define FIREBASE_AUTH ""
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK             GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DI0_PIN               26
#define RADIO_RST_PIN               23
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              32

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

#define GPS_BAUD_RATE               9600
#define HAS_GPS
#define HAS_DISPLAY                 //Optional, bring your own board, no OLED !!

#define DISABLE_TRANSMIT_TO_NODE 


const long frequency = 923E6;  // LoRa Frequency

CRC8 crc;

union u_tag {
  uint8_t b[16];
  double fval[2];
} u;

union u_batt {
  uint8_t b[4];
  float fval;
} b;

typedef struct packet {
  uint8_t id;
  u_tag t;
  u_batt b;
  uint8_t crc8 ;
} __attribute__ ((packed)) PACKET_T;

PACKET_T packet_sent ;

uint8_t buf[100] ;

bool isReceive = 0;


void setup() {
  
  SPI.begin(RADIO_SCLK_PIN,RADIO_MISO_PIN,RADIO_MOSI_PIN,RADIO_CS_PIN);

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  
  Serial.begin(115200);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

#ifdef ENABLE_FIREBASE_WIFI

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  
#endif

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Gateway");
  Serial.println("Only receive messages from nodes");
  Serial.println("Tx: invertIQ enable");
  Serial.println("Rx: invertIQ disable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

void loop() {
  
#if defined ( DISABLE_TRANSMIT_TO_NODE )

#else
  if (runEvery(5000)) { // repeat every 5000 millis
    String message = "HeLoRa World! ";
    message += "I'm a Gateway! ";
    message += millis();

    LoRa_sendMessage(message); // send a message

    Serial.println("Send Message!");
  }
#endif

  if( isReceive ) 
  {
    isReceive = 0;

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();

    memcpy( &packet_sent , buf , sizeof( PACKET_T ));

//    packet_sent.lat = u.fval[0];
//    packet_sent.lon = u.fval[1];
//
    Serial.print("latitude: ");
    Serial.println(packet_sent.t.fval[0],6);
    Serial.print("longtitude: ");
    Serial.println(packet_sent.t.fval[1],6);
    Serial.print("battery: ");
    Serial.println(packet_sent.b.fval);

    if ( crc8((uint8_t *)&packet_sent, sizeof( PACKET_T ) - 1 , 0x07) == packet_sent.crc8 )
    {
      root["latitude"] = packet_sent.t.fval[0];
      root["longtitude"] = packet_sent.t.fval[1];
      root["battery"] = packet_sent.b.fval;
      root["timestamp"] = getTime();
      
#ifdef ENABLE_FIREBASE_WIFI
      if ( (packet_sent.id) )
          Firebase.push("Pet-Traking/0"+ String(packet_sent.id) , root);   
      //Firebase.push("Pet-Traking/01", root);
#endif

    }
  }
}

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  
  uint8_t index = 0;

  isReceive = 1;

  while (LoRa.available()) {
    //message += (char)LoRa.read();
    buf[index] = (char)LoRa.read();
    index++;
  }

  Serial.print("Gateway Receive: ");
  Serial.println( packetSize );

// Debug payload receive
  for( int i = 0 ; i < packetSize ; i++ ) 
  {
    Serial.print( buf[i] , HEX);
    Serial.print(" ");
  }

//    u.b[0] = buf[0];
//    u.b[1] = buf[1];
//    u.b[2] = buf[2];
//    u.b[3] = buf[3];
//    u.b[4] = buf[4];
//    u.b[5] = buf[5];
//    u.b[6] = buf[6];
//    u.b[7] = buf[7];


    //location.add("lat",u.fval[0]);
//    memset(buf,0,100);

}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
