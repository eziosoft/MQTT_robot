#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Servo.h>

const char* ssid = "Robot";
const char* outTopic = "tank/out";
const char* inTopic = "tank/in";
const char* mqtt_server = "192.168.0.19";
const char* mqtt_server1 = "test.mosquitto.org";


//PINS
//0-15 build in pull up
//16 build in pull down
const int PIN_BBL = D1;
const int PIN_BBR = D2;
const int PIN_BFR = D5;
const int PIN_BFL = D6;
const int servo1 = D4;
const int servo2 = D3;
const int servo3 = D7;
const int servo4 = D8;
const int PIN_LED = D0;


char buffer1[20];//multiusage
WiFiClient espClient;
PubSubClient client(espClient); //MQTT

Servo s1;
Servo s2;
Servo s3;
Servo s4;

int BBL, BBR, BFR, BFL;

void setup() {
  pinMode(PIN_BBL, INPUT_PULLUP);
  pinMode(PIN_BBR, INPUT_PULLUP);
  pinMode(PIN_BFR, INPUT_PULLUP);
  pinMode(PIN_BFL, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);

//  s3.attach(servo3);
//  s4.attach(servo4);

  // Connecting WiFi
  WiFiManager wifiManager;
  wifiManager.autoConnect("Robot");


  //OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  //OTA END

  checkBumpers();
  if (BFL == LOW) //connecto to server1 when front left bumper activated during power on
  {
    client.setServer(mqtt_server1, 1883);
    for (int i = 0; i < 5; i++) //blink 5 times with led
    {
      digitalWrite(PIN_LED, HIGH);
      delay(50);
      digitalWrite(PIN_LED, LOW);
      delay(50);
    }

  } else {
    client.setServer(mqtt_server, 1883);
  }
  client.setCallback(callback);

  digitalWrite(PIN_LED, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);

}


void loop() {
  ArduinoOTA.handle();

  //MQTT
  if (!client.connected()) reconnect();
  else client.loop();

  checkBumpers();

  if (millis() % 200 == 0) {
    sprintf(buffer1, "T;%d;RSSI=%d;%d;%d;%d;%d", millis() / 1000, WiFi.RSSI(), BBL, BBR, BFR, BFL);
    client.publish(outTopic, buffer1);
  }
}


void checkBumpers()
{
  BBL = digitalRead(PIN_BBL);
  BBR = digitalRead(PIN_BBR);
  BFR = digitalRead(PIN_BFR);
  BFL = digitalRead(PIN_BFL);

  digitalWrite(PIN_LED, !BBL || !BBR || !BFR || !BFL);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (length > 0) {
    if ((char)payload[0] == '$') {

      int len = payload[1];
      int ch1 = payload[2] - 100;
      int ch2 = payload[3] - 100;
      int ch3 = payload[4] - 100;
      int ch4 = payload[5] - 100;


      if (ch1 == 0 && ch2 == 0) //stop
      {
        s1.detach(); //detach to prevent servo drift
        s2.detach();
      } else
      {
        if (!s1.attached()) s1.attach(servo1);
        if (!s2.attached()) s2.attach(servo2);
      }

      if (ch3 == 0 && ch4 == 0) //stop
      {
        s3.detach(); //detach to prevent servo drift
        s4.detach();
      } else
      {
        if (!s3.attached()) s3.attach(servo3);
        if (!s4.attached()) s4.attach(servo4);
      }

      //chanel mixer
      int m1 = ch2 - ch1;
      int m2 = ch2 + ch1;
      if (m1 > 100)m1 = 100;
      if (m1 < -100) m1 = -100;
      if (m2 > 100)m2 = 100;
      if (m2 < -100) m2 = -100;


      if (BFR == LOW || BFL == LOW) //stop forward if front bumper activated
      {
        if (m1 > 0)m1 = 0;
        if (m2 > 0)m2 = 0;
      }

      if (BBR == LOW || BBL == LOW) //stop backward if back bumper activated
      {
        if (m1 < 0)m1 = 0;
        if (m2 < 0)m2 = 0;
      }

      m1 = map(m1, -100, 100, 110, 70) + 3;
      m2 = map(m2, -100, 100, 70, 110) + 3;
      int m3 = map(ch3, -100, 100, 0, 180)+10;
      int m4 = map(ch4, -100, 100, 0, 180);
      s1.write(m1);
      s2.write(m2);
      s3.write(m3);
      s4.write(m4);

      //debug
      Serial.print(ch1);
      Serial.print(" ");
      Serial.print(ch2);
      Serial.print("->");
      Serial.print(m1);
      Serial.print(" ");
      Serial.println(m2);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ssid)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, "Tank READY");
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}











