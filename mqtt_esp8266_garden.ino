/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NeoPixelBus.h>
#include "Timer.h"

// Update these with values suitable for your network.

const char* ssid = "SSSSSSSS";
const char* password = "PPPPPPPPPPPP";
const char* mqtt_server = "192.168.150.192";
const char* mqtt_username = "UUUUUUUUUUUU";
const char* mqtt_password = "PPPPPPPPPPPP";
String MQTT_TOPIC_SUFFIX = "garden";

#define LIGHT 14
#define SOCKET 12
#define BUTTON_SOCKET 13
#define BUTTON_LIGHT 16

const int pins[] = { SOCKET, LIGHT };
const int pins_size = sizeof(pins) / sizeof(int);

const int button[] = { BUTTON_SOCKET, BUTTON_LIGHT };
const int button_size = sizeof(button) / sizeof(int);

bool pins_last_state[pins_size];

Timer t;

#define AUTOOFF 120
#define LED 2
#define LEDFLASH 3
int timeout = AUTOOFF;
bool newtimeout = false;

uint32_t spikecounter[button_size];
bool execute[button_size];

// Neopixel settings
const int numLeds = 1; // change for your setup
const int numberOfChannels = numLeds * 3; // Total number of channels you want to receive (1 led = 3 channels)
NeoPixelBus  leds = NeoPixelBus(numLeds, LED, NEO_GRB | NEO_KHZ800);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

int time_tasks[button_size];
// state LOW/HIGH, pin num, auto off time in min
void doSwitch( bool state, int pin, int auto_off ) {

  uint8_t i;
  for(i = 0; i<pins_size; i++){
    if( pins[i] == pin ) break;
  }
  
  if ( state == LOW ) {
    digitalWrite(pin, LOW);
    if( time_tasks[i] > -1 ){
      t.stop(time_tasks[i]);
      time_tasks[i] = -1;
    }
  } else {
    time_tasks[i] = t.pulseImmediate(pin, auto_off * 60 * 1000, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  
  //pinMode(LED, OUTPUT);
  leds.Begin();
  leds.SetPixelColor(0, RgbColor(0,0,0));
  leds.Show();

   for (uint8_t i = 0; i < pins_size; i++) {
    pinMode(pins[i], OUTPUT);
  }
  for (uint8_t i = 0; i < button_size; i++) {
    pinMode(button[i], INPUT);
    spikecounter[i] = 0;
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char pl[sizeof(payload)];
  for(int i = 0; i< length; i++){
    pl[i] = (char)payload[i];
  }
  String s_pl = String(pl);
  s_pl.trim();
  Serial.println( s_pl );

  if( String(topic) == (MQTT_TOPIC_SUFFIX+"/light/switch") ){
    Serial.print( "light " );
    if( s_pl == "on" ){
      Serial.println( "on" );
      doSwitch(HIGH, LIGHT, timeout);
    }else{
      Serial.println( "off" );
      doSwitch(LOW, LIGHT, timeout);
    }
  }else if( String(topic) == (MQTT_TOPIC_SUFFIX+"/socket/switch") ){
     Serial.print( "socket " );
     if( s_pl == "on" ){
      Serial.println( "on" );
      doSwitch(HIGH, SOCKET, timeout);
    }else{
      Serial.println( "off" );
       doSwitch(LOW, SOCKET, timeout);
    }
  }else if( String(topic) == (MQTT_TOPIC_SUFFIX+"/timeout") ){
    timeout = s_pl.toInt();
    Serial.println( "new timeout "+ String(timeout) );
    newtimeout = true;
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client_garden", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement... 
      for (uint8_t i = 0; i < pins_size; i++) {
        pins_last_state[i] = !digitalRead(pins[i]);
      }
      client.publish( (MQTT_TOPIC_SUFFIX+"/timeout/status").c_str(), String(timeout).c_str() );
      // ... and resubscribe
      client.subscribe( (MQTT_TOPIC_SUFFIX+"/light/switch").c_str());
      client.subscribe( (MQTT_TOPIC_SUFFIX+"/socket/switch").c_str() );
      client.subscribe( (MQTT_TOPIC_SUFFIX+"/timeout").c_str() );
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int ledstate = 0;
void loop() {

  if (!client.connected()) {
    Serial.println( "reconnect" );
    reconnect();
  }
  client.loop();

  t.update();

// Button entprellt
  for (uint8_t i = 0; i < button_size; i++) {
    //Serial.println( "stat: "+String(i)+" "+String(digitalRead(button[i])==LOW)+ " "+String(button[i]) );
    if(digitalRead(button[i]) == LOW && execute[i]) {
      if (spikecounter[i] >= 4000) {
        doSwitch( !digitalRead(pins[i]), pins[i], timeout );
        Serial.println( "button pressed: " + String(button[i]) + " / "+String(!digitalRead(button[i]))+"\n" );
        spikecounter[i] = 0;
        execute[i] = false;
      } else {
        spikecounter[i]++;
      }
    }
    if(digitalRead(button[i]) == HIGH && !execute[i]) execute[i] = true;
  }  

// status bei geändertem PIN
  if( pins_last_state[0] != digitalRead(pins[0]) ){
      if( digitalRead(pins[0]) ) client.publish( (MQTT_TOPIC_SUFFIX+"/socket/status").c_str(), "on");
      else client.publish( (MQTT_TOPIC_SUFFIX+"/socket/status").c_str(), "off");
      pins_last_state[0] = digitalRead(pins[0]);
  }
  if( pins_last_state[1] != digitalRead(pins[1]) ){
      if( digitalRead(pins[1]) ) client.publish( (MQTT_TOPIC_SUFFIX+"/light/status").c_str(), "on");
      else client.publish( (MQTT_TOPIC_SUFFIX+"/light/status").c_str(), "off");
      pins_last_state[1] = digitalRead(pins[1]);
  }

  if( newtimeout ){
    newtimeout = false;
    client.publish( (MQTT_TOPIC_SUFFIX+"/timeout/status").c_str(), String(timeout).c_str() );
  }

// Farbkombi für Schalter
  if(digitalRead(pins[0]) && !digitalRead(pins[1]) && ledstate != 1 ){
    ledstate = 1;
    Serial.println( "LED 1" );
    leds.SetPixelColor(0, RgbColor(100,0,0));
    leds.Show();
  }else if(!digitalRead(pins[0]) && digitalRead(pins[1]) && ledstate != 2){
    ledstate = 2;
    Serial.println( "LED 2" );
    leds.SetPixelColor(0, RgbColor(0,100,0));
    leds.Show();
  }else if(digitalRead(pins[0]) && digitalRead(pins[1]) && ledstate != 3 ){
    ledstate = 3;
    Serial.println( "LED 3" );
    leds.SetPixelColor(0, RgbColor(0,0,100));
    leds.Show();
  }else if(!digitalRead(pins[0]) && !digitalRead(pins[1]) && ledstate != 0){
    ledstate = 0;
    Serial.println( "LED 0" );
    leds.SetPixelColor(0, RgbColor(0,0,0));
    leds.Show();
  }
}
