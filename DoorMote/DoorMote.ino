// Get the RFM69 at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!

// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        28   //unique for each node on same network
#define GATEWAYID     1    //node Id of the receiver we are sending data to
#define NETWORKID     250  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_868MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define ACK_TIME      50   // max # of ms to wait for an ack
//*********************************************************************************************
#define SERIAL_EN                               // uncomment if you want serial debugging output
#define BATT_SENS                               // uncomment to enable batter reading
#define REEDSWITCH_SENS                         // uncomment to enable reed switch
//*********************************************************************************************
#ifdef BATT_SENS
float batteryVolts;           // current battery reading
float batteryPrevVolts;       // previous battery reading
#define BATT_HISTERESIS 0.5   // Difference between current and previous reading to send to the gateway.
#define BATT_MONITOR_PIN  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_FORMULA(reading) reading * 0.00322 * 1.47  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_LOW      3.5
#define BATT_EN_PIN 14
#define BATT_AVG_COUNT 5
#endif
//*********************************************************************************************
#ifdef REEDSWITCH_SENS
#define REEDSWITCH_PIN 3
bool readSwitchState;
#endif
//*********************************************************************************************
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#ifdef SERIAL_EN
#define SERIAL_BAUD   115200
#define DEBUG(input)   {Serial.print(input);}
#define DEBUGln(input) {Serial.println(input);}
#define SERIALFLUSH() {Serial.flush();}
#else
#define DEBUG(input);
#define DEBUGln(input);
#define SERIALFLUSH();
#endif
//*********************************************************************************************

//function prototypes
void txRadio(char* buff);

//global program variables
float diff;
byte sendLen;
String sendData;

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

void setup() {
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD); // Open serial monitor at 115200 baud to see ping results.
#endif

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
  DEBUGln(F("\r\nRFM69_ATC Enabled (Auto Transmission Control)"));
#endif

  DEBUG(F("\nDoorMote : ")); DEBUG(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915); DEBUG(F(" Mhz,"));
  DEBUG(F("id:")); DEBUG(NODEID);
  DEBUG(F(" nid:")); DEBUG(NETWORKID);
  DEBUG(F(" gid:")); DEBUGln(GATEWAYID);

#ifdef REEDSWITCH_SENS
  pinMode(REEDSWITCH_PIN, INPUT);
  attachInterrupt(1, wakeUp, CHANGE); //interrupt for waking up
  DEBUGln("ReedSwitch sens pin enabled");
#endif

#ifdef BATT_SENS
  pinMode(BATT_EN_PIN, OUTPUT);
  DEBUGln("Battery sens pin enabled");
#endif

  txRadio("START");
  radio.sleep();
  SERIALFLUSH();
}

void loop() {

  sendData = "";
  delay(50);
  //*********************************************************************************************
  //************ Receive data from radio network                                    *************
  //*********************************************************************************************
  if (radio.receiveDone()) {
    byte senderID = radio.SENDERID;
    DEBUGln();
    DEBUG("["); DEBUG(senderID); DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);
    DEBUG(F(" [RX_RSSI:")); DEBUG(radio.RSSI); DEBUG(F("]"));
  }

#ifdef REEDSWITCH_SENS
  //*********************************************************************************************
  //************ ReedSwitch Reading                                                 *************
  //*********************************************************************************************
  int rs = digitalRead(REEDSWITCH_PIN);
  if (rs != readSwitchState) {
    sendData = " DO1:" + (String)rs;
  }
  readSwitchState = rs;
#endif

#ifdef BATT_EN_PIN
  //*********************************************************************************************
  //************ Battery Reading                                                    *************
  //*********************************************************************************************
  readBattery();
#endif

  //*********************************************************************************************
  //************ Sending Data To Gateway Node                                       *************
  //*********************************************************************************************
  sendData.trim();

  if (sendData.length() > 3 ) {
    DEBUGln(sendData);
    txRadio(sendData.c_str());
    sendData = "";
  }

  radio.sleep();

  SERIALFLUSH();

  sleepNow();
}

/*
   Transmits the message to the gateway
*/
void txRadio(const char * buff) {
  digitalWrite(LED, HIGH);
  sendLen = strlen(buff);
  radio.sendWithRetry(GATEWAYID, buff, sendLen, 2);
  digitalWrite(LED, LOW);
}

#ifdef BATT_EN_PIN
void readBattery() {
  digitalWrite(BATT_EN_PIN, HIGH);          // Enable battery sensor reading
  delay(10);

  analogRead(BATT_MONITOR_PIN);
  unsigned int readings = 0;
  for (byte i = 0; i < BATT_AVG_COUNT; i++) //take several samples, and average
    readings += analogRead(BATT_MONITOR_PIN);
  batteryVolts = BATT_FORMULA(readings / BATT_AVG_COUNT);

  diff = batteryVolts - batteryPrevVolts;
  if (diff > BATT_HISTERESIS || diff < -BATT_HISTERESIS) {
    sendData += " BA1:" + (String)batteryVolts;

  }
  batteryPrevVolts = batteryVolts;

  digitalWrite(BATT_EN_PIN, LOW);           // Disable battery sensor reading
}
#endif


#ifdef REEDSWITCH_SENS
void wakeUp() {
  //needed for the digital input interrupt
}

void sleepNow() {
  attachInterrupt(1, wakeUp, CHANGE); //interrupt for waking up

  // How long to sleep
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life

  detachInterrupt(1); // disables interrupt 0 on pin 2 so the wakeUp code will not be executed during normal running time.
}
#endif

