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

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        30                 //unique for each node on same network
#define GATEWAYID     1                  //node Id of the receiver we are sending data to
#define NETWORKID     250                //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_868MHZ        //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW                       //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
#define ENCRYPTKEY    "16Nov1980@8PMgoa" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC                       //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define ACK_TIME      50                 // max # of ms to wait for an ack
//*********************************************************************************************
#define SERIAL_EN         // uncomment if you want serial debugging output
//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
#define LED           15                        // Moteino MEGAs have LEDs on D15
#define FLASH_SS      23
#else
#define LED           9                         // Moteinos have LEDs on D9
#define FLASH_SS      8
#endif
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

/************************************************************************************
  MQ-2 Gas Sensor Sandbox Electronics    2011-04-25
************************************************************************************/

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (1)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
#define         HISTERESIS                    1

/*****************************Globals***********************************************/
int             LPG = 0;
int             prevLPG = 0;
float           LPGCurve[3]  =  {2.3, 0.21, -0.47}; //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
int             CO = 0;
int             prevCO = 0;
float           COCurve[3]  =  {2.3, 0.72, -0.34};  //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
int             SMOKE = 0;
int             prevSMOKE = 0;
float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
/**********************************************************************************/

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

  DEBUG(F("\nMQ2Mote : ")); DEBUG(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915); DEBUG(F(" Mhz,"));
  DEBUG(F("id:")); DEBUG(NODEID);
  DEBUG(F(" nid:")); DEBUG(NETWORKID);
  DEBUG(F(" gid:")); DEBUGln(GATEWAYID);

  DEBUGln(F("Calibrating...\n"));
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air
  //when you perform the calibration
  DEBUGln(F("Calibration is done...\n"));
  DEBUG(F("Ro="));
  DEBUG(Ro);
  DEBUGln(F(" kohm"));

  txRadio("START");

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

  /*********************************************************************************************
     LPG Reading
   *********************************************************************************************/
  LPG = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  diff = LPG - prevLPG;
  if (diff > HISTERESIS || diff < -HISTERESIS) {

    sendData += " LP1:" + (String)LPG;
    prevLPG = LPG;
  }

  /*********************************************************************************************
     CO Reading
   *********************************************************************************************/
  CO = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  diff = CO - prevCO;
  if (diff > HISTERESIS || diff < -HISTERESIS) {

    sendData += " CO1:" + (String)CO;
    prevCO = CO;
  }

  /*********************************************************************************************
     SMOKE Reading
   *********************************************************************************************/
  SMOKE = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);
  diff = SMOKE - prevSMOKE;
  if (diff > HISTERESIS || diff < -HISTERESIS) {

    sendData += " SM1:" + (String)SMOKE;
    prevSMOKE = SMOKE;
  }

  /********************************************************************************************
   *********** Sending Data To Gateway Node                                       *************
   ********************************************************************************************/
  sendData.trim();

  if (sendData.length() > 3 ) {
    DEBUGln(sendData);
    txRadio(sendData.c_str());
    sendData = "";
  }

  SERIALFLUSH();
  delay(200);
}

/*
 * Transmits the message to the gateway
*/
void txRadio(const char * buff) {
  digitalWrite(LED, HIGH);
  sendLen = strlen(buff);
  radio.sendWithRetry(GATEWAYID, buff, sendLen, 2);
  digitalWrite(LED, LOW);
}

/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc) {
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  Ro of the sensor
  Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin) {
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
