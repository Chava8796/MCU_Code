#include <Arduino.h>
#include <Stream.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <deprecated.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <require_cpp11.h>
#include <SPI.h>

//AWS
#include "sha256.h"
#include "Utils.h"

//WEBSockets
#include <Hash.h>
#include <WebSocketsClient.h>

//MQTT PUBSUBCLIENT LIB 
#include <PubSubClient.h>

//Current Time Library
#include <time.h>

//AWS MQTT Websocket
#include "Client.h"
#include "AWSWebSocketClient.h"
#include "CircularByteBuffer.h"

extern "C" {
  #include "user_interface.h"
}

//Sensors pins
#define HALL_SENSOR_PIN       A0
#define SDA_RFID_PIN          15
#define RST_RFID_PIN          5

//LED pin
#define ACCESS_LED            2

//Bicycle specs
#define WHEEL_DIAM            26
#define WHEEL_CIRCUMFERENCE   3.141592*WHEEL_DIAM*2.54/100

//GENERAL SETUP
#define SERIAL_BAUDRATE   115200
#define SENSOR_TH         400

//Pacific Time 
#define TIMEZONE -6

//IoT
//1t3s0IoT18

//AWS IOT config, change these:
char wifi_ssid[]       = "Wi-Fi ssid";
char wifi_password[]   = "password for your ssid";
char aws_endpoint[]    = "your aws endpoint";
char aws_key[]         = "your aws KEY";
char aws_secret[]      = "your aws Secret Key";
char aws_region[]      = "your aws region";
char aws_deviceID[]    = "your aws device ID";
const char* aws_topic  = "your first topic";
const char* aws_topic_1  = "your second topic";
const char* aws_topic_2 = "your third topic";

//MQTT + Websocket services for AWS
int port = 443;

//MQTT config
const int maxMQTTpackageSize = 256;
const int maxMQTTMessageHandlers = 1;

//RFC Cards 
byte myCards[] = {0x71,0x54,0xA6,0x08,0x24,0xD7,0x66,0x4B};      
byte dummy = 0x00;
byte readCard[4];
byte tempReadCard[4];

//RFID reading flags
static boolean successRead = false;
static boolean first_read = false;

//Variable to count revolutions
int revolutionCounter = 0;
float distanceVar = 0;

//Wi-Fi instance for the ESP8266 module
ESP8266WiFiMulti WiFiMulti;

//MFRC522(RFID) instance for the RFID module
MFRC522 mfrc522(SDA_RFID_PIN, RST_RFID_PIN); 

//WebSocket client instance AWS-specific
AWSWebSocketClient awsWSclient(1000);

//Client instance that enables the transmission and reception of messages
PubSubClient client(awsWSclient);

//Variables and struct for time
time_t now;
struct tm * timeinfo;

//number of connections
long connection = 0;

//generate random mqtt clientID
char* generateClientID () {
  char* cID = new char[23]();
  for (int i=0; i<22; i+=1)
    cID[i]=(char)random(1, 256);
  return cID;
}

int currentMonth = 11;
int currentYear  = 2018;

//count messages arrived
int arrivedcount = 0;

//callback to handle mqtt messages
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//Function that counts the number of rev.
void hallSensorRead(boolean re_start)
{
  int hallEffectSensorData = 0;
  static boolean sensorFlag = false;

  if(true == re_start){
    revolutionCounter = 0;
  }
  else
  {
    hallEffectSensorData = analogRead(HALL_SENSOR_PIN);
    if((hallEffectSensorData < SENSOR_TH) && (false == sensorFlag))
    {
      revolutionCounter++;
      sensorFlag = true;
      Serial.println(revolutionCounter);
    }
    if((hallEffectSensorData > SENSOR_TH) && (true == sensorFlag))
    {
      sensorFlag = false;
    }
    yield();
    //delay(5);
  }
}

//Handler to read RFC devices
boolean getID() 
{
  // Getting ready for Reading PICCs
  if (!mfrc522.PICC_IsNewCardPresent())
  {
    return false;
  }
  if (!mfrc522.PICC_ReadCardSerial())
  {
    return false;
  }
  Serial.println("");
  for (int i = 0; i < 4; i++){
    readCard[i] = mfrc522.uid.uidByte[i];
    Serial.print(readCard[i], HEX);
  }
  Serial.println("");
  mfrc522.PICC_HaltA(); 
  return true;
}

//connects to websocket layer and mqtt layer
bool connect () {
    if (client.connected()) {    
        client.disconnect ();
    }  
    //delay is not necessary... it just help us to get a "trustful" heap space value
    delay (1000);
    Serial.print (millis ());
    Serial.print (" - conn: ");
    Serial.print (++connection);
    Serial.print (" - (");
    Serial.print (ESP.getFreeHeap ());
    Serial.println (")");


    //creating random client id
    char* clientID = generateClientID ();
    
    client.setServer(aws_endpoint, port);
    if (client.connect(clientID)) {
      Serial.println("connected");     
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      return false;
    }
}

//subscribe to a mqtt topic
void subscribe () {
    client.setCallback(callback);
    client.subscribe(aws_topic_1);
   //subscript to a topic
    Serial.println("MQTT subscribed");
}


//send the start message to a mqtt topic
void sendStartMessage() {
    char msg[110];
    time(&now);
    timeinfo = localtime(&now);
    
    //send a message   
    snprintf (msg, 110,"{\"data\": \"%x%x%x%x;%s;%.2i-%.2i-%d%.2i:%.2i:%.2i\" }", 
                  tempReadCard[0], tempReadCard[1], tempReadCard[2], tempReadCard[3],
                  aws_deviceID, 
                  timeinfo->tm_mday, currentMonth, currentYear, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    Serial.print("Publish message: ");
    Serial.println(msg);
    int rc = client.publish(aws_topic, msg);
    Serial.println(aws_topic);
}

//send a periodic message to a mqtt topic
void sendPeriodicMessage() {
    char msg[115];
    int previousSec = timeinfo->tm_sec;
    int previousMin = timeinfo->tm_min;
    int previousHour = timeinfo->tm_hour;
    int previousMDay = timeinfo->tm_mday;
    int previousMon = currentMonth;
    int previousYear = currentYear;
    
    time(&now);
    timeinfo = localtime(&now);

    distanceVar = revolutionCounter * (WHEEL_CIRCUMFERENCE) / 1000;
    
    //send a message   
    snprintf (msg, 115, "{\"data\": \"%x%x%x%x;%s;%.2i-%.2i-%d%.2i:%.2i:%.2i;%.2i-%.2i-%d%.2i:%.2i:%.2i;%f\" }",
                  tempReadCard[0], tempReadCard[1], tempReadCard[2], tempReadCard[3],
                  aws_deviceID,
                  previousMDay, previousMon, previousYear, previousHour, previousMin, previousSec,
                  timeinfo->tm_mday, currentMonth, currentYear, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
                  distanceVar);
    Serial.print("Publish message: ");
    Serial.println(msg);
    int rc = client.publish(aws_topic_1, msg);
    Serial.println(aws_topic_1);
    Serial.println(rc);
}

//send an ending message to a mqtt topic
void sendFinalMessage() {
    char msg[110];
    
    time(&now);
    timeinfo = localtime(&now);
    
    //send a message   
    snprintf (msg, 110,"{\"data\": \"%x%x%x%x;%s;%.2i-%.2i-%d%.2i:%.2i:%.2i\" }",
                  tempReadCard[0], tempReadCard[1], tempReadCard[2], tempReadCard[3],
                  aws_deviceID, 
                  timeinfo->tm_mday, currentMonth, currentYear, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    Serial.print("Publish message: ");
    Serial.println(msg);
    int rc = client.publish(aws_topic_2, msg);
    Serial.println(aws_topic_2);
}

//Error reading handler function
void Error()
{
  Serial.println("USUARIO NO IDENTIFICADO"); 
}

void setup() {

    pinMode(ACCESS_LED, OUTPUT);

    digitalWrite(ACCESS_LED, HIGH);
    
    wifi_set_sleep_type(NONE_SLEEP_T);
    // initialize serial communication at 115200 bits per second
    Serial.begin(SERIAL_BAUDRATE);
    delay (2000);
    Serial.setDebugOutput(1);
    
    //Initialize SPI protocol
    SPI.begin();

    //Config and initialize the mfrc522 object
    mfrc522.PCD_Init();                             
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_avg); 
  
    //fill with ssid and wifi password
    WiFiMulti.addAP(wifi_ssid, wifi_password);
    Serial.println ("connecting to wifi");
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        Serial.print (".");
    }
    Serial.println ("\nconnected");

    //fill AWS parameters    
    awsWSclient.setAWSRegion(aws_region);
    awsWSclient.setAWSDomain(aws_endpoint);
    awsWSclient.setAWSKeyID(aws_key);
    awsWSclient.setAWSSecretKey(aws_secret);
    awsWSclient.setUseSSL(true);

    if (connect ()){
      subscribe ();
      Serial.println("IDENTIFIQUESE...");
      digitalWrite(ACCESS_LED, LOW);
      delay(1000);
      digitalWrite(ACCESS_LED, HIGH);
    }

    configTime(TIMEZONE * 3600, 0, "pool.ntp.org", "time.nist.gov");
    while (!time(nullptr)) {
      Serial.print(".");
      delay(1000);
    }
}

void validateIdCode(){
  if (readCard[0] == myCards[4] && readCard[1] == myCards[5] && readCard[2] == myCards[6] && readCard[3] == myCards[7])
  {  
    if(readCard[0] == tempReadCard[0] && readCard[1] == tempReadCard[1] && readCard[2] == tempReadCard[2] && readCard[3] == tempReadCard[3])
    {
      Serial.println("Nos vemos usuario 1");
      digitalWrite(ACCESS_LED, HIGH);
      first_read = false;
      sendFinalMessage();
      for(int i = 0; i<4; i++)
      {
        tempReadCard[i] = dummy;   // removing previous stored value from the readCard variable
      }
    }
    else if (dummy == tempReadCard[0] && dummy == tempReadCard[1] && dummy == tempReadCard[2] && dummy == tempReadCard[3])
    {
      Serial.println("Bienvenido a casa usuario 1");
      digitalWrite(ACCESS_LED, LOW);
      for(int i = 0; i<4; i++)
      {
        tempReadCard[i] = readCard[i];   
      }
      first_read = true;
      sendStartMessage();
      hallSensorRead(true);
     }
    else
    {
      Serial.println("Hay alguien más antes de ti");
    }
    for(int i = 0; i<4; i++)
    {
      readCard[i] = dummy;   // removing previous stored value from the readCard variable
    }
    successRead = false;
  }
  else if(readCard[0] == myCards[0] && readCard[1] == myCards[1] && readCard[2] == myCards[2] && readCard[3] == myCards[3])
  {
    if(readCard[0] == tempReadCard[0] && readCard[1] == tempReadCard[1] && readCard[2] == tempReadCard[2] && readCard[3] == tempReadCard[3]){
      first_read = false;
      Serial.println("Nos vemos usuario 2");
      digitalWrite(ACCESS_LED, HIGH);
      sendFinalMessage();
      for(int i = 0; i<4; i++){
        tempReadCard[i] = dummy;   // removing previous stored value from the readCard variable
      }
     }
     else if (dummy == tempReadCard[0] && dummy == tempReadCard[1] && dummy == tempReadCard[2] && dummy == tempReadCard[3]){
      Serial.println("Bienvenido a casa usuario 2");
      digitalWrite(ACCESS_LED, LOW);
      for(int i = 0; i<4; i++){
        tempReadCard[i] = readCard[i];   // removing previous stored value from the readCard variable
      }
      first_read = true;
      sendStartMessage();
      hallSensorRead(true);
     }
     else{
      Serial.println("Hay alguien más antes de ti");
     }
     for(int i = 0; i<4; i++){
      readCard[i] = dummy;   // removing previous stored value from the readCard variable
     }
     successRead = false;
  }
  else
  {
    Error();      //calling the error function
    successRead = false;
  }
}

void loop() {
  static int initialTime = 0;
  int currentTime;
  
  //keep the mqtt up and running
  if (awsWSclient.connected ()) 
  {    
      client.loop ();
      if(false == successRead){
        successRead = getID(); 
        if(true == first_read){
          currentTime = millis()/1000;
          if(10 == (currentTime-initialTime)){
            sendPeriodicMessage();
            hallSensorRead(true);
            initialTime = currentTime;
          }
          hallSensorRead(false);
        }
        else{
          /* do nothing */
        }
      }
      else{
        validateIdCode();
        initialTime = millis()/1000;
      }  
  } else {
    //handle reconnection
    if (connect ()){
      subscribe ();      
    }
  }
}
