#include <Arduino.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
//#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define MQTT_VERSION MQTT_VERSION_3_1_1

//Motor A
const int dir1PinA = 12;
const int dir2PinA = 13;
const int pwmEn = 5;

const int tacPin = 14;
const int led = 2;
const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int tac = 0;
int lastTac = 0;

volatile int WatchCount = 0;

Ticker secondTick;

const int rollerMax = 120;
const int rollerMin = 0;
int count = 0;
int setCount = 0;
int tempCount = 0;

boolean Stopper = true;
boolean dir_up = true;

// Wifi: SSID and password
const char* WIFI_SSID = "...";
const char* WIFI_PASSWORD = "...";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "Bedroom_roller";
const PROGMEM char* MQTT_SERVER_IP = "192.168.1.32";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "...";
const PROGMEM char* MQTT_PASSWORD = "...";

// MQTT: topics
const char* MQTT_ROLLER_STATE_TOPIC = "bedroom/rollers";
const char* MQTT_ROLLER_COMMAND_TOPIC = "bedroom/rollers/set";
const char* MQTT_TEMP_STATE_TOPIC = "bedroom/temperature";

// payloads by default (on/off)
const char* ROLLER_OPEN = "OPEN";
const char* ROLLER_CLOSE = "CLOSE";
const char* ROLLER_STOP = "STOP";

//payloads for manually overriding blinds position 
const char* ROLLER_RESETLOW = "RESETLOW";
const char* ROLLER_RESETHIGH = "RESETHIGH";
const char* ROLLER_NEGONE = "negOne";
const char* ROLLER_POSONE = "posOne";

// payloads by default (on/off)
const char* PAYLOAD_OPEN = "open";
const char* PAYLOAD_CLOSE = "closed";

const int LED_PIN = 2;

//WebUpdate
const char* host = "esp8266-blinds";
ESP8266WebServer server(80);
const char* serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";


long lastMsg = 0;
float temp = 0.0;
float diff = 0.1;

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE];

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void publishBlindsState() {
  tempCount = ((count) * (1000 / rollerMax) / 10);
  if (tempCount > 97) {
    tempCount = 100;
  }
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", tempCount);
  client.publish(MQTT_ROLLER_STATE_TOPIC, m_msg_buffer, true);
}

void publishTempState() {
  client.publish(MQTT_TEMP_STATE_TOPIC, String(temp).c_str(), true);
}

void setBlindsState() {
  if (!Stopper) {
    if (dir_up == 1) {
      digitalWrite(dir1PinA, HIGH);
      digitalWrite(dir2PinA, LOW);
      Serial.println("Going up");
    }
    else {
      digitalWrite(dir1PinA, LOW);
      digitalWrite(dir2PinA, HIGH);
      Serial.println("Going down");
    }
  }
}

void StopState() {
  Stopper = true;
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, LOW);
  Serial.println("Stop");
}

void setRollerDirection() {
  if (setCount == count) {
    StopState();
  }
  else if (setCount > count) {
    dir_up = 1;
  }
  else if (setCount < count) {
    dir_up = 0;
  }
}



// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  // handle message topic
  if (String(MQTT_ROLLER_COMMAND_TOPIC).equals(p_topic)) {
    Serial.println(payload);
    if (payload.equals(String(ROLLER_OPEN))) {
      Stopper = false;
      WatchCount = 0;
      setCount = rollerMax;
      setRollerDirection();
      setBlindsState();
      publishBlindsState();
    }
    else if (payload.equals(String(ROLLER_CLOSE))) {
      Stopper = false;
      WatchCount = 0;
      setCount = rollerMin;
      setRollerDirection();
      setBlindsState();
      publishBlindsState();
    }

    else if (payload.equals(String(ROLLER_STOP))) {
      Stopper = true;
      StopState();
      setBlindsState();
      publishBlindsState();
    }
    else if (payload.equals(String(ROLLER_RESETLOW))) {
      count = rollerMin;
      Stopper = true;
      StopState();
      setBlindsState();
      publishBlindsState();
    }
    else if (payload.equals(String(ROLLER_RESETHIGH))) {
      count = rollerMax;
      Stopper = true;
      StopState();
      setBlindsState();
      publishBlindsState();
    }
    else if (payload.equals(String(ROLLER_NEGONE))) {
      count--;
      Stopper = true;
      StopState();
      setBlindsState();
      publishBlindsState();
    }
    else if (payload.equals(String(ROLLER_POSONE))) {
      count++;
      Stopper = true;
      StopState();
      setBlindsState();
      publishBlindsState();
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      // Once connected, publish an announcement...
      publishBlindsState();
      // ... and resubscribe
      client.subscribe(MQTT_ROLLER_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void stopWatchdog() {
  WatchCount++;
  if (Stopper == false) {
    if (WatchCount > 3) {
      Serial.println();
      Serial.println("Watchdog Stop");
      Stopper = true;
      StopState();
      setBlindsState();
    }
  }
}

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  Serial.print("INFO: Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if(WiFi.waitForConnectResult() == WL_CONNECTED){
    MDNS.begin(host);
    server.on("/", HTTP_GET, [](){
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/html", serverIndex);
    });
    server.on("/update", HTTP_POST, [](){
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
      ESP.restart();
    },[](){
      HTTPUpload& upload = server.upload();
      if(upload.status == UPLOAD_FILE_START){
        Serial.setDebugOutput(true);
        WiFiUDP::stopAll();
        Serial.printf("Update: %s\n", upload.filename.c_str());
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)){//start with max available size
          Update.printError(Serial);
        }
      } else if(upload.status == UPLOAD_FILE_WRITE){
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
          Update.printError(Serial);
        }
      } else if(upload.status == UPLOAD_FILE_END){
        if(Update.end(true)){ //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
    server.begin();
    MDNS.addService("http", "tcp", 80);

    Serial.printf("Ready! Open http://%s.local in your browser\n", host);
  } else {
    Serial.println("WiFi Failed");

    delay(5000);
    ESP.restart();
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  secondTick.attach(1, stopWatchdog);

  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  //pinMode(tacPin, INPUT);
  pinMode(pwmEn, OUTPUT);
  digitalWrite(led, 1);
  digitalWrite(pwmEn, 1);
  sensors.begin();

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  tac = digitalRead(tacPin);
  if (Stopper == false) {
    if (tac == !lastTac) {
      if (dir_up == true) {
        count++;
        WatchCount = 0;
      }
      else {
        count--;
        WatchCount = 0;
      }

      setRollerDirection();
      publishBlindsState();
    }
    lastTac = tac;
  }

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    if (Stopper == true) {
      sensors.requestTemperatures(); // Send the command to get temperatures
      float  newTemp = sensors.getTempCByIndex(0);
      //Serial.println(sensors.getTempCByIndex(0));

      if (checkBound(newTemp, temp, diff)) {
        temp = newTemp;
        //Serial.print("New temperature:");
        //Serial.println(String(temp).c_str());
        publishTempState();
        //client.publish(temperature_topic, String(temp).c_str(), true);
      }
    }
  }

  server.handleClient();
  client.loop();
}
