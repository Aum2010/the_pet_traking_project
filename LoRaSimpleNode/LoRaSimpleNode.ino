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
#include <TinyGPS++.h>
#include <axp20x.h>
#include "CRC8.h"
#include "CRC.h"

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

#define ID_NODE  1
#define TIME_INTRAVEL  10000

#define DISABLE_RECEIVE

const long frequency = 923E6;  // LoRa Frequency

TinyGPSPlus     gps;
uint32_t        gpsLoopMillis = 0;
uint32_t        positioningMillis = 0;
void GpsLoop(void);

AXP20X_Class PMU;

char buff[5][256];

CRC8 crc;

typedef struct {
  uint8_t id;
  double lat;
  double lon;
  float batt;
  uint8_t crc8;
}__attribute__ ((packed)) PACKET_T;

static PACKET_T packet_sent ;

void setup() {
  
  SPI.begin(RADIO_SCLK_PIN,RADIO_MISO_PIN,RADIO_MOSI_PIN,RADIO_CS_PIN);

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);

  Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA, I2C_SCL);
  initPMU();
  
  Serial.begin(115200);
  
  while (!Serial);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.setSpreadingFactor(7);

#if defined ( DISABLE_RECEIVE )
  
#else
  LoRa.onReceive(onReceive);
#endif

  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  packet_sent.id = ID_NODE;
}

void loop() {
  if (runEvery(TIME_INTRAVEL)) { // repeat every 1000 millis

    String message = "HeLoRa World! ";
    message += "I'm a Node! ";
    message += millis();

//    LoRa_sendMessage(message); // send a message
    LoRa_sendMessage((uint8_t*)&packet_sent);
    Serial.println("Send Message!");
  }

  GpsLoop();
  
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

//void LoRa_sendMessage(String message) {
//  LoRa_txMode();                        // set tx mode
//  LoRa.beginPacket();                   // start packet
//  LoRa.print(message);                  // add payload
//  LoRa.endPacket(true);                 // finish packet and send it
//}

void LoRa_sendMessage( uint8_t* msg ) 
{

  LoRa_txMode();
//  Serial.println(msg->lat);
//  Serial.println(msg->lon);

  packet_sent.crc8 = crc8((uint8_t *)&packet_sent, sizeof( PACKET_T ) - 1 , 0x07) ;
  
  LoRa.beginPacket(); 
  LoRa.write(msg , sizeof( PACKET_T ) ); 
  LoRa.endPacket(true); 
}

void onReceive(int packetSize) {
  String message = "";

  Serial.println("onReceive State!");

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
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

void GpsLoop(void)
{
    while (Serial1.available()) {
        int r = Serial1.read();
        //Serial.write(r);
        gps.encode(r);
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
        snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
        snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
        Serial.println("No GPS detected");
        //return;
    }
    if (!gps.location.isValid()) {
        if (millis() - gpsLoopMillis > 1000) {
            snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
            snprintf(buff[1], sizeof(buff[1]), "Positioning(%u)S", positioningMillis++);
            gpsLoopMillis = millis();
        }
    } else {
        if (millis() - gpsLoopMillis > 1000) {
            snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
            snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
            snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
            snprintf(buff[3], sizeof(buff[3]), "satellites:%u", gps.satellites.value());

            packet_sent.lat = gps.location.lat();
            packet_sent.lon = gps.location.lng();

//            packet_sent.lat = 1.1111;
//            packet_sent.lon = 2.2222;
            
            Serial.printf("UTC:%d:%d:%d-LNG:%.4f-LAT:%.4f-satellites:%u\n",
                          gps.time.hour(),
                          gps.time.minute(),
                          gps.time.second(),
                          gps.location.lng(),
                          gps.location.lat(),
                          gps.satellites.value());

            Serial.println(PMU.getVbusVoltage()); //<--- No Connect Battery!
            //Serial.println(PMU.getBattVoltage());

            packet_sent.batt = (PMU.getVbusVoltage()) / 1000 ;
            
                          
            gpsLoopMillis = millis();
        }
    }
}

bool initPMU()
{
    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    /*
     * The charging indicator can be turned on or off
     * * * */
    // PMU.setChgLEDMode(LED_BLINK_4HZ);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC3Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC3, AXP202_ON);
    *
    * * * */

    /*
     *   Turn off unused power sources to save power
     * **/

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                   AXP202_VBUS_CUR_ADC1 |
                   AXP202_BATT_CUR_ADC1 |
                   AXP202_BATT_VOL_ADC1,
                   AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                  AXP202_VBUS_CONNECT_IRQ |
                  AXP202_BATT_REMOVED_IRQ |
                  AXP202_BATT_CONNECT_IRQ,
                  AXP202_ON);
    PMU.clearIRQ();

    return true;
}

void disablePeripherals()
{
    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
}
