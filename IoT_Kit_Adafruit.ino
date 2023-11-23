
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <mDash.h>

#define MDASH_APP_NAME "IoT-App"
#define DEVICE_PASSWORD "2SI90dO8iyTBKqoDVpOrZVw" //isi dengan password pada mDash
#define SS_PIN  5  // ESP32 pin GPIO5 
#define RST_PIN 0 // ESP32 pin GPIO27
#define Relay 17
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
Servo myservo;
WiFiClient espClient;
PubSubClient client(espClient);


float temp;
 
MFRC522 rfid(SS_PIN, RST_PIN);
// Your WiFi credentials.
// Set password to "" for open networks.
const char* ssid = "CUAKS";
const char* password = "12345678";
const char* mqttServer = "io.adafruit.com";
const int mqttPort = 1883;
const char* mqttUser = "Robotic_UBL";
const char* mqttPassword = "aio_VMhZ38cJ4tldmSTF6mT914QFHa6y";
const char* mqttTopic1 = "Robotic_UBL/feeds/relay";
const char* mqttTopic2 = "Robotic_UBL/feeds/lamp";
const char* mqttTopic3 = "Robotic_UBL/feeds/motion";
const char* mqttTopic4 = "Robotic_UBL/feeds/rfid";
const char* mqttTopic5 = "Robotic_UBL/feeds/gas";
const char* mqttTopic6 = "Robotic_UBL/feeds/flame";
const char* mqttTopic7 = "Robotic_UBL/feeds/soil";
const char* mqttTopic8 = "Robotic_UBL/feeds/temperature";
const char* mqttTopic9 = "Robotic_UBL/feeds/ldr";
const char* mqttTopic10 = "Robotic_UBL/feeds/ir";



/* Sensors */
const int flame = 14; 
const int gas = 39 ;
const int zervo = 33;
const int ledPin = 25;
const int motion = 26;
const int ir = 2;
const int fan = 15;
const int buzzer = 27;
const int LED = 12;

/* Two "independant" timed events */
const long eventTime_1_flame = 20000; //in ms
const long eventTime_2_gas = 20000; //in ms
const long eventTime_3 = 20000;
const long eventTime_4 = 20000;
const long eventTime_5 = 20000;
const long eventTime_6 = 20000;

/* When did they start the race? */
unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
unsigned long previousTime_4 = 0;
unsigned long previousTime_5 = 0;
unsigned long previousTime_6 = 0;

int status_relay = 0 ;
bool toggleState_1 = LOW;
int val_gas;
int val_flame = 0;
int val_ir = 0;
int val_motion = 0;

byte keyTagUID[4] = {0x43, 0x90, 0x09, 0xFB};


void wifi_connect() {
  Serial.print("Starting connecting WiFi.");
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void mqtt_setup() {
  client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    Serial.println("Connecting to MQTT…");
    while (!client.connected()) {        
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqttUser, mqttPassword )) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state  ");
            Serial.println(client.state());
            delay(2000);
        }
    }
    client.subscribe(mqttTopic1);
    client.subscribe(mqttTopic2);
    client.subscribe(mqttTopic3);
    client.subscribe(mqttTopic4);
    client.subscribe(mqttTopic5);
    client.subscribe(mqttTopic6);
    client.subscribe(mqttTopic7);
    client.subscribe(mqttTopic8);
    client.subscribe(mqttTopic9);
    client.subscribe(mqttTopic10);
}
char sPayload[100];
char message [40] ;
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String topicStr = topic;
  String response;
  String res;
  Serial.print(message);
  for (int i = 0; i < length; i++) {
    response += (char)payload[i];
  }
  Serial.println(response);
  Serial.println(res);
  //Subscsibe data Relay
  if (topicStr == "Robotic_UBL/feeds/relay") {
    if (response == "0"){
      Serial.println("RELAY OFF");
      digitalWrite(Relay, LOW);
    }

    else if (response =="1"){
      Serial.println("RELAY ON");
      digitalWrite(Relay, HIGH);
    }

    Serial.println();
    Serial.println(" — — — — — — — — — — — -");
  }
  //Subscsibe data Lampu
  else if (topicStr == "Robotic_UBL/feeds/lamp") {
    if (response == "OFF"){
        Serial.println("LAMP OFF");
        digitalWrite(LED, LOW);
    }

    else if (response == "ON"){
        Serial.println("LAMP ON");
        digitalWrite(LED, HIGH);
    }

    Serial.println();
    Serial.println(" — — — — — — — — — — — -");
  }   
}

void kirimSensor () {
  //Deklarasi variabel
  unsigned long currentTime = millis();
  val_flame =  digitalRead(flame);
  val_gas =  analogRead(gas) ;
  val_ir = digitalRead(ir);
  val_motion = digitalRead(motion);
  
  if ( currentTime - previousTime_1 >= eventTime_1_flame) {
    //mengirim data sensor flame
    Serial.print("flame: ");
    Serial.println(val_flame);
    if (val_flame == 0) {
      client.publish(mqttTopic6,"1");
      digitalWrite(buzzer, HIGH);
      
    }
    else {
      client.publish(mqttTopic6,"0");
      digitalWrite(buzzer, LOW);
    }
    
    previousTime_1 = currentTime;
  }
  if ( currentTime - previousTime_2 >= eventTime_2_gas) {
    //mengirim data sensor Gas
    Serial.print("gas: ");
    Serial.println(val_gas);
    String dataSend = String(val_gas);
    client.publish(mqttTopic5, dataSend.c_str());
    if (val_gas >= 700) {
      digitalWrite(fan, HIGH);
    }
    else {
      digitalWrite(fan, LOW);
    }

    previousTime_2 = currentTime;
  }
  if ( currentTime - previousTime_3 >= eventTime_3) {
    //mengirim data sensor RFID
    if (rfid.PICC_IsNewCardPresent()) { // new tag is available
      if (rfid.PICC_ReadCardSerial()) { // NUID has been readed
        MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
        
        if (rfid.uid.uidByte[0] == keyTagUID[0] &&
            rfid.uid.uidByte[1] == keyTagUID[1] &&
            rfid.uid.uidByte[2] == keyTagUID[2] &&
            rfid.uid.uidByte[3] == keyTagUID[3] ) 
            {
              Serial.println("Access is granted");
              client.publish(mqttTopic4, "Pintu Terbuka");
              myservo.write(10);  // unlock the door for 2 seconds
              delay(2000);
              myservo.write(90);
              client.publish(mqttTopic4, "Pintu Tertutup");
      }
      else
      {
        Serial.print("Access denied, UID:");
        client.publish(mqttTopic4, "Pintu Tidak Terbuka !!!");
        for (int i = 0; i < rfid.uid.size; i++) {
          Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
          Serial.print(rfid.uid.uidByte[i], HEX);
        }
        Serial.println();
      }

      rfid.PICC_HaltA(); // halt PICC
      rfid.PCD_StopCrypto1(); // stop encryption on PCD
      }
    }
    previousTime_3 = currentTime;
  }
  if ( currentTime - previousTime_4 >= eventTime_4) {
    //mengirim data sensor IR
    Serial.print("IR: ");
    Serial.println(val_ir);
    if (val_ir == 0) {
      client.publish(mqttTopic10, "1");
      myservo.write(10);  // unlock the door for 2 seconds
      delay(2000);
      myservo.write(90);
    }
    else {
      client.publish(mqttTopic10, "0");
      myservo.write(90);
    }

    previousTime_4 = currentTime;
  }
  if ( currentTime - previousTime_5 >= eventTime_5) {
    //mengirim data sensor Motion
    Serial.print("Motion: ");
    Serial.println(val_motion);
    if (val_motion == 1) {
      client.publish(mqttTopic3, "Ada Orang !!!");
    }
    else {
      client.publish(mqttTopic3, "Tidak ada Orang");
    }
    previousTime_5 = currentTime;
  }
  if (currentTime - previousTime_6 >= eventTime_6) {
    //mengirim data sensor Suhu
    temp = dht.readTemperature();//baca suhu
    if (isnan(temp)) { //jika tidak ada hasil
      Serial.println("DHT11 tidak terbaca... !");
      delay(1000);
      return;
    }
    else {//jika ada hasilnya 
      Serial.print("Suhu=");  //kirim serial "Suhu"
      Serial.print(temp);     //kirim serial nilai suhu
      Serial.println("C");
      String suhu;
      suhu = String(temp);
      client.publish(mqttTopic8, suhu.c_str());
    }
    previousTime_6 = currentTime;
  }
}

void setup() {
  Serial.begin(9600);
  SPI.begin(); // init SPI bus
  wifi_connect();
  mqtt_setup();
  mDashBegin(DEVICE_PASSWORD);
  myservo.attach(zervo);
  rfid.PCD_Init(); // init MFRC522
  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");
  pinMode(ledPin, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(motion, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(flame, INPUT);
}

void loop() {
  client.loop();
  kirimSensor(); //memanggil fungsi kirimSensor
}
