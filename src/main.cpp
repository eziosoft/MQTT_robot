#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Servo.h>

const char *ssid = "Robot";               //robot creates wifi hotspot when wifi connection is not configured
const char *outTopic = "tank/out";        //MQTT topic for robot telemetry messages
const char *inTopic = "tank/in";          //MQTT topic for control messages
const char *mqtt_server = "192.168.0.19"; //my defauld MQTT server
const char *mqtt_server1 = "test.mosquitto.org";

const int servo1 = D4; //servo driving the wheel
const int servo2 = D3; //servo driving the wheel

const int PIN_LED = LED_BUILTIN; // LED lights

char buffer1[20]; //multiusage
WiFiClient espClient;
PubSubClient client(espClient); //MQTT

Servo s1;
Servo s2;

int servo_trim1 = 3; // servo trim used to find center position of a servo
int servo_trim2 = 3; // servo trim used to find center position of a servo

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  //control is done by 6 byte messages : [$][len][ch1][ch2][ch3][ch4]
  // [$] - start of the frame
  // [len] - number of bytes to read. in this case always 4
  // [ch1][ch2][ch3][ch4] - control channels. Each channel has a range from 0 to 200. Middle is at 100 - servo stop
  if (length > 0)
  {
    if ((char)payload[0] == '$')
    {

      int len = payload[1];
      int ch1 = payload[2] - 100;
      int ch2 = payload[3] - 100;
      int ch3 = payload[4] - 100;
      int ch4 = payload[5] - 100;

      if (ch1 == 0 && ch2 == 0) //stop
      {
        s1.detach(); //detach to prevent servo drift when stationary and also saves power
        s2.detach(); //detach to prevent servo drift when stationary and also saves power
      }
      else
      {
        if (!s1.attached())
          s1.attach(servo1); //attach servo when needed
        if (!s2.attached())
          s2.attach(servo2); //attach servo when needed
      }

      //chanel mixer
      int m1 = ch2 - ch1;
      int m2 = ch2 + ch1;
      if (m1 > 100)
        m1 = 100;
      if (m1 < -100)
        m1 = -100;
      if (m2 > 100)
        m2 = 100;
      if (m2 < -100)
        m2 = -100;

      m1 = map(m1, -100, 100, 110, 70) + servo_trim1;
      m2 = map(m2, -100, 100, 70, 110) + servo_trim2;

      s1.write(m1);
      s2.write(m2);

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

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ssid))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, "Tank READY");
      // ... and resubscribe
      client.subscribe(inTopic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/////////////////////////////////////////////////////////////
void setup()
{
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);

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
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  //OTA END

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //blink led
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(PIN_LED, LOW);
    delay(50);
    digitalWrite(PIN_LED, HIGH);
    delay(50);
  }
}

void loop()
{
  ArduinoOTA.handle();

  //MQTT
  if (!client.connected())
    reconnect();
  else
    client.loop();

  //send telemetry every 200ms
  if (millis() % 200 == 0)
  {
    sprintf(buffer1, "T;%d;RSSI=%d;", (millis() / 1000), WiFi.RSSI());
    client.publish(outTopic, buffer1);
  }
}
