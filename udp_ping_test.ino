/*
  WiFi UDP Send and Receive String

 This sketch waits for a UDP packet on localPort using the WiFi module.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 created 30 December 2012
 by dlf (Metodo2 srl)

 */


#include <SPI.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010)
#include <WiFiNINA.h>
#elif defined(ARDUINO_PORTENTA_H7_M7)
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <Chrono.h>

#if defined (LED_BUILTIN)
#define LED LED_BUILTIN 
#define led_setup()  pinMode(LED,OUTPUT);

#if defined(ARDUINO_PORTENTA_H7_M7)
#define WIFI_FIRMWARE_LATEST_VERSION "1.0"
#define led_on() digitalWrite(LED,LOW);
#define led_off() digitalWrite(LED,HIGH);
#define indic(_i_) digitalWrite(LED, _i_ == 0? HIGH: LOW);
#define indicp(_i_) do{digitalWrite(LEDR, _i_ == 0? HIGH: LOW);digitalWrite(LEDB, _i_ == 0? HIGH: LOW); }while(0);
#else
#define led_on() digitalWrite(LED,HIGH);
#define led_off() digitalWrite(LED,LOW);
#define indic(_i_) digitalWrite(LED, _i_ == 0? LOW: HIGH);
#define indicp(_i_) digitalWrite(LED, _i_ == 0? LOW: HIGH);
#endif
#else
#define led_setup()
#define led_on() 
#define led_off()
#define indic(_i_) 
#endif

#define STEP_LEN 500
int status = WL_IDLE_STATUS;
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned long st = 0, et = 0;

unsigned int localPort = 2390;      // local port to listen on
static long cntr = 0;

#define LEN 64
byte pack[LEN]; //buffer to hold incoming packet
byte ack[LEN];
IPAddress targetIP(192,168,1,22);
int targetPort = 5760;
WiFiUDP Udp;
Chrono chrono;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  led_setup();
  //while (!Serial) continue;

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD /* WL_NO_MODULE */) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.print("Please upgrade the firmware to");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  }

  // attempt to connect to WiFi network:
  checkWiFi();
  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {
  if(chrono.hasPassed(STEP_LEN)){
    chrono.restart();
    checkWiFi();
    ;
    ++cntr;
    
    Serial.print(cntr);
    Serial.print(".");
    Serial.println(" Send pack ");
    
    pack_pack();
    st = millis();
    Udp.beginPacket(targetIP, targetPort);
    Udp.write(pack,LEN);
    indic(1);
    Udp.endPacket();
    indic(0);
  }
  // if there's data available, read a packet
  
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("  Received packet of size ");
    Serial.println(packetSize);
    Serial.print("  From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(ack, LEN);
    if (len == LEN) {
      et = millis();
      long pcntr = *(long*) ack;
      char c = ack[sizeof(long)+1];
      long sensor = *(long*)(ack+1+sizeof(long));
      Serial.print("  ack cntr:");
      Serial.print(pcntr);

      if(cntr == pcntr){
        indic(1);
        delay(50);
        Serial.print("... [ ack pack ok] ... ");
        
      }
      Serial.print(" sensor:");
      Serial.print(sensor);
      Serial.print(" takes:");
      Serial.print(et-st);
      Serial.print(" ms.");
      
      Serial.println();
    }
    indic(0);
  }
}

void pack_pack(){
  
  *(long*)(pack) = cntr;
  pack[sizeof(long)] = 'a';
  *(long*)(pack+sizeof(long)+1) = 0;
  
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void checkWiFi(){
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.end();
    indicp(1);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(2000);
    indicp(0);
    delay(2000);
  }
}
