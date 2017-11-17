// RFM69 MQTT gateway sketch
//
// This gateway relays messages between a MQTT-broker and several wireless nodes and will:
// - receive sensor data from several nodes periodically and on-demand
// - send/receive commands from the broker to control actuators and node parameters
//
//	Connection to the MQTT broker is over a fixed ethernet connection:
//
//		The MQTT topic is /home/rfm_gw/direction/nodeid/devid

#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPI.h>           //included with Arduino IDE (www.arduino.cc)
#include <Ethernet.h>
#include <PubSubClient.h>

#define VERSION "V1.5"

#define DEBUG             // uncomment for MQTT, RADIO and NETWORK debugging
#define DEBUGRADIO				// uncomment for radio debugging
//#define DEBUGNETWORK      // uncomment for network debugging
//#define DEBUGMQTT         // uncomment  for mqtt debugging

// Ethernet settings
byte mac[] = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };	// MAC address for ethernet
IPAddress mqtt_server(1, 1, 1, 1);		                // MQTT broker address (Mosquitto)
IPAddress ip(1, 1, 1, 1);			                        // Gateway address (if DHCP fails)

// Wireless settings
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID          1 //the ID of this node
#define NETWORKID     250 //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_868MHZ         //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey"  //identical 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME       30  // # of ms to wait for an ack packet
#define RFM_SS 8        // Slave Select RFM69 is connected to pin 8
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75  //target RSSI for RFM69_ATC (recommended > -80)
//*****************************************************************************************************************************
// Serial baud rate must match your Pi/host computer serial port baud rate!
#define SERIAL_BAUD  115200
//*****************************************************************************************************************************

// PIN settings
#define MQCON 7					// MQTT Connection indicator
#define R_LED 9					// Radio activity indicator
//****************************************************************************************************************

int	  dest;			                              // destination node for radio packet
int   BTN;                                    // Button index
int 	error;					                        // Syntax error code
long	lastMinute = -1;			                  // timestamp last minute
long	upTime = 0;				                      // uptime in minutes
bool	Rstat = false;				                  // radio indicator flag
bool	mqttCon = false;			                  // MQTT broker connection flag
bool	respNeeded = false;			                // MQTT message flag in case of radio connection failure
bool	promiscuousMode = false;	      	      // only receive closed network nodes
bool	verbose = true;				                  // generate error messages
long	onMillis;				                        // timestamp when radio LED was turned on
char  *rootTopic = "home/rfm_gw";             // Root topic
char	*subTopic = "home/rfm_gw/sb/#";		      // MQTT subscription topic ; direction is southbound
char	*clientName = "RFM_gateway";		        // MQTT system name of gateway
char	buff_topic[30];				                  // MQTT publish topic string
char	buff_mess[32];				                  // MQTT publish message string
int   startSBmessageAfter = 10;               // Start publishing sb messages only after n number of secs after system is up and running
long  timeSinceStart = 0;                     // Get current start time since system start

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

// Callback function header, must be declared before connecting to client
void mqtt_subs(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
PubSubClient mqttClient(mqtt_server, 1883, mqtt_subs, ethClient );

//
//==============  MQTT_SUBS
//
//    receive messages from subscribed topics
//    parse MQTT topic / message and construct radio message
//
//    The values in the MQTT topic/message are converted to corresponding values on the Radio network
//

void mqtt_subs(char* topic, byte* payload, unsigned int length) {

  int i;

#ifdef DEBUGMQTT
  Serial.print(F("Topic received from MQTT Server:   "));
  Serial.print(topic);
  Serial.print(F(", size ["));
  Serial.print(strlen(topic));
  Serial.print(F("]"));
  Serial.println();
#endif

  if (strlen(topic) == 27) {
    dest = (topic[19] - '0') * 10 + (topic[20] - '0') * 10 + topic[21] - '0';             // extract target node ID from MQTT topic
    BTN = (topic[26] - '0');                                      // extract button index
    payload[length] = '\0';                                       // terminate string with '0'
    String strPayload = String((char*)payload);  // convert to string
    sprintf(buff_mess, "BTN%d:%s", BTN, strPayload.c_str());
    respNeeded = true;

    // Start sending message only after system up and running time is reached.
    if (timeSinceStart >= startSBmessageAfter) {
      sendMsg(dest, buff_mess, 3 + sizeof(BTN) + strPayload.length());
    }
    
  } else {
    error = 1;
#ifdef DEBUGMQTT
    Serial.println(F("Wrong message format in MQTT subscription"));
#endif
  }

  if ((error != 0) && verbose) {          // in case of syntax error
    sprintf(buff_mess, "syntax error %d for node %d", error, dest);
    sprintf(buff_topic, "home/rfm_gw/nb/node001/dev91"); // construct MQTT topic and message
    mqtt_pubs(buff_topic, buff_mess);   // publish ...
#ifdef DEBUG
    Serial.print("Syntax err: ");
    Serial.println(error);
#endif
  }

} // end mqttSubs

//
//==============	SETUP
//

void setup() {
#ifdef DEBUG
  Serial.begin(SERIAL_BAUD);
  delay(10);
#ifdef DEBUGRADIO
  Serial.println(F("Initializing Radio..."));
#endif
#endif

  radio.setCS(RFM_SS);					                    // change default Slave Select pin for RFM
  radio.initialize(FREQUENCY, NODEID, NETWORKID);		// initialise radio module
#ifdef IS_RFM69HW
  radio.setHighPower(); 					                  // only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);				                // encrypt with shared key
  radio.promiscuous(promiscuousMode);               // listen only to nodes in closed network

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  pinMode(R_LED, OUTPUT);                           // set pin of radio indicator
  pinMode(MQCON, OUTPUT);					                  // set pin for MQTT connection indicator
  digitalWrite(MQCON, LOW);  				                // switch off MQTT connection indicator
  digitalWrite(R_LED, LOW);				                  // switch off radio indicator

#ifdef DEBUGRADIO
  Serial.print(F("GW Version "));
  Serial.println(VERSION);
  Serial.print(F("\nListening at "));
  Serial.print(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(F(" Mhz..."));
#endif

#ifdef DEBUGNETWORK
  Serial.println(F("Connecting to Network..."));
#endif

  if (Ethernet.begin(mac) == 0) {			              // start the Ethernet connection
#ifdef DEBUG && DEBUGNETWORK
    Serial.println(F("No DHCP"));
#endif
    Ethernet.begin(mac, ip);
  }

#ifdef DEBUGNETWORK
  Serial.println(F("Connecting to network"));
#endif

  delay(1000);
#ifdef DEBUG
#ifdef DEBUGNETWORK
  Serial.println(F("Network Connected"));
#endif
#ifdef DEBUGMQTT
  Serial.print("Connecting to MQTT Server [ ");
  for ( int i = 0; i < sizeof(mqtt_server); i++) {
    Serial.print(mqtt_server[i]);
    Serial.print(F(", "));
  }
  Serial.print(F(" ] as "));
  Serial.println(clientName);
#endif
#endif

  mqttCon = 0;					                            // reset connection flag
  while (mqttCon != 1) {				                    // retry MQTT connection every 2 seconds
    mqttCon = mqttClient.connect(clientName);	      // retry connection to broker
    delay(2000);					                          // every 2 seconds
  }

  if (mqttCon) {					                          // Connected !
#ifdef DEBUGMQTT
    Serial.println(F("MQTT-link OK"));
#endif
    digitalWrite(MQCON, HIGH);			                // switch on MQTT connection indicator

    sprintf(buff_mess, "%s %s", clientName, "Joined Server.");
    mqtt_pubs("home/rfm_gw/nb/", buff_mess);

    mqttClient.subscribe(subTopic);			            // subscribe to all southbound messages
  } else {
    // switch off MQTT connection indicator
    digitalWrite(MQCON, LOW);
#ifdef DEBUGMQTT
    Serial.println(F("MQTT connection lost"));
#endif
    abort();
  }

#ifdef DEBUG
  Serial.println(F("Setup completed"));
#endif
}	// end setup

//
//==============	MAIN
//

void loop() {

  // CONTROL RADIO LED AND CALCULATE UPTIME
  //

  if (Rstat) {						// turn off radio LED after 100 msec
    if (millis() - onMillis > 100) {
      Rstat = false;
      digitalWrite(R_LED, LOW);
    }
  }

  if (lastMinute != (millis() / 60000)) {			// another minute passed ?
    lastMinute = millis() / 60000;
    upTime++;
  }

  if (radio.receiveDone()) {
    processPacket(); // check for received radio packets and construct MQTT message
  }

  if (!mqttClient.loop()) {			// check connection MQTT server and process MQTT subscription input
    mqttCon = 0;
    digitalWrite(MQCON, LOW);

#ifdef DEBUGMQTT
    Serial.println(F("MQTT connection lost"));
#endif

    while (mqttCon != 1) {			// try to reconnect every 2 seconds
      mqttCon = mqttClient.connect(clientName);
      delay(2000);
    }

    if (mqttCon) {				// Yes, we have a link so,
#ifdef DEBUGMQTT
      Serial.println(F("MQTT-link OK"));
#endif
      digitalWrite(MQCON, HIGH);      // switch on MQTT connection indicator

      sprintf(buff_mess, "%s %s", clientName, "Joined Server.");
      mqtt_pubs("home/rfm_gw/nb/", buff_mess);

      mqttClient.subscribe(subTopic);     // subscribe to all southbound messages
#ifdef DEBUGMQTT
      Serial.println(F("MQTT connection restored"));
#endif
    }
  }

  // Keep increment time until system is up and running
  if (timeSinceStart <= startSBmessageAfter) {
    timeSinceStart = millis() / 1000;
  }

}	// end loop

//
//==============	SENDMSG
//
//	sends messages over the radio network

void sendMsg(int target, char* radioMessage, int msgSize) {

  Rstat = true;						// radio indicator on
  digitalWrite(R_LED, HIGH);				// turn on radio LED
  onMillis = millis();					// store timestamp

  int i = 1;						// number of transmission retries

  while (respNeeded && i > 0) {				// first try to send packets

    if (radio.sendWithRetry(target, radioMessage, msgSize, 1)) {
      respNeeded = false;
#ifdef DEBUGRADIO
      Serial.print(F("Msg to node " )); Serial.print(target);
      Serial.print(" msg size: "); Serial.print(msgSize);
      Serial.print(" msg: "); Serial.print(radioMessage);
      Serial.println(" ");
#endif
    } else delay(500);				// half a second delay between retries
    i--;
  }

  if (respNeeded && verbose) { 					// if not succeeded in sending packets after 5 retries
    //sprintf(buff_topic, "home/rfm_gw/nb/node%03d/dev90", target);	// construct MQTT topic and message
    //sprintf(buff_mess, "radio lost node %d", target);		// for radio loss (device 90)
    //mqtt_pubs(buff_topic, buff_mess);			// publish ...
    respNeeded = false;						// reset response needed flag
#ifdef DEBUGRADIO
    Serial.print(F("Node "));
    Serial.print(target);
    Serial.println(F(" off-line"));
#endif
  }

  memset(&buff_topic, '\0', sizeof(buff_topic));
  memset(&buff_mess, '\0', sizeof(buff_mess));
}	// end sendMsg

//
//==============	PROCESSPACKET
//
// receives data from the wireless network, parses the contents and constructs MQTT topic and value

void processPacket() {
  Rstat = true;							// set radio indicator flag
  digitalWrite(R_LED, HIGH);					// turn on radio LED
  onMillis = millis();						// store timestamp

#ifdef DEBUGRADIO
  Serial.print("Received radio packet from node ");
  Serial.print(radio.SENDERID);
  Serial.print(": ");
  for (byte i = 0; i < radio.DATALEN; i++) {
    Serial.print((char)radio.DATA[i]);
  }
  Serial.println();
#endif

  //listen for BTNx:y commands where x={0,1,2}, y={0,1}
  if (radio.DATALEN == 6 && radio.DATA[0] == 'B' && radio.DATA[1] == 'T' && radio.DATA[2] == 'N' && radio.DATA[4] == ':'
      && (radio.DATA[3] >= '0' && radio.DATA[3] <= '2') && (radio.DATA[5] == '0' || radio.DATA[5] == '1')) {
    if (radio.ACKRequested()) radio.sendACK(); //send ACK sooner when a ON/OFF + ACK is requested
    byte btnIndex;
    radio.DATA[3] == '0' ? btnIndex = 0 : radio.DATA[3] == '1' ? btnIndex = 1 : btnIndex = 2;
    sprintf(buff_topic, "%s/nb/node%03d/BTN%d", rootTopic, radio.SENDERID, btnIndex);
    mqtt_pubs(buff_topic, radio.DATA[5] == '0' ? "OFF" : "ON");

  } else if ((radio.DATALEN == 5 || radio.DATALEN == 6) && radio.DATA[0] == 'S' && radio.DATA[1] == 'T' && radio.DATA[2] == 'A' && radio.DATA[3] == 'R' && radio.DATA[4] == 'T') {
    // Check for start Message
    if (radio.ACKRequested()) radio.sendACK(); //send ACK sooner when a ON/OFF + ACK is requested
    sprintf(buff_topic, "%s/nb/node%03d/START", rootTopic, radio.SENDERID);
    sprintf(buff_mess, "%d%d", radio.SENDERID, random(10, 20));
    mqtt_pubs(buff_topic, buff_mess);

  } else {
    char pload[radio.DATALEN];

    for (byte i = 0; i < radio.DATALEN; i++) {
      pload[i] = radio.DATA[i];
    }

    pload[radio.DATALEN] = '\0';

    char *p = pload;
    char *str;
    char *kv;

    while ((str = strtok_r(p, " ", &p)) != NULL) {

      // Extract the key name
      kv = strtok(str, ":");
      if (kv) {

        memset(&buff_topic, '\0', sizeof(buff_topic));
        sprintf(buff_topic, "%s/nb/node%03d/%s", rootTopic, radio.SENDERID, kv);

        // Extract the value
        kv = strtok(NULL, ":");
        if (kv) {

          memset(&buff_mess, '\0', sizeof(buff_mess));
          sprintf(buff_mess, "%s", kv);

          mqtt_pubs(buff_topic, buff_mess);
        }
      }
    }
    memset(pload, '\0', sizeof(pload));
  }

  memset(&buff_topic, '\0', sizeof(buff_topic));
  memset(&buff_mess, '\0', sizeof(buff_mess));

  sprintf(buff_mess, "%d", radio.RSSI);
  sprintf(buff_topic, "%s/nb/node%03d/RSSI", rootTopic, radio.SENDERID);
  mqtt_pubs(buff_topic, buff_mess);
}	// end processPacket

/*
   MQTT_PUBS
   Send messages to published topics
*/
void mqtt_pubs(char* loTopic, char* loMsg) {
#ifdef DEBUGMQTT
  Serial.print(F("MQTT Topic: "));
  Serial.print(loTopic);
  Serial.print(F(", Message: "));
  Serial.println(loMsg);
#endif

  if (!mqttClient.publish(loTopic, loMsg)) {
#ifdef DEBUGMQTT
    Serial.println(F("Publish Failed"));
#endif
  }
}





