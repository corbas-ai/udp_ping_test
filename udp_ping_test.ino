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

byte mem_buffer[2048]={};
#define GBS_CHUNK_SIZE 32

#define GBS_SB1 0x4e
#define GBS_SB2 0x41
#define GBS_ACK 0xe3
#define GBS_END 0xe4

#define GBS_ERR 0x10
#define GBS_ERR_MSG_TOO_BIG 0x11

// 1 byte pix , 1 byte crc8 : 2
#define GBS_CHUNK_SRV_LEN 2 

#define is_start(_buff_, _buff_len_) (_buff_len_>= 2 && _buff_[0] == GBS_SB1 && _buff_[1] == GBS_SB2 )

static enum get_buffers_state_t{
  GBS_READY, GBS_READING
} get_buffers_state = GBS_READY;

Chrono gbs_chrono;


//время мс передачи одного чанка
#define GBS_WAIT1   1

int get_buffers(Stream& port, byte* mem, int mem_size, int &tag) { //returns count bytes received
  int retval = 0;
  static short take = 0;
  static unsigned taked = 0;
  byte n_parts = 0;
  if(port.available()){
    if(get_buffers_state == GBS_READY){
      byte buff[]={0xff,0xff,0x0,0x0,0x0};
      int pos = 0;
      while(port.available()){
        buff[pos++] = port.read();
        if(pos == sizeof(buff)){
          if( is_start(buff, pos)){
            gbs_chrono.restart(); 
            take = buff[3]<<8|buff[4];
            taked = 0;
            tag = buff[2];
            n_parts = gbs_send_ack( port, take, mem_size );
            if(n_parts > 0){      
              get_buffers_state = GBS_READING;
            }
          }else{
            if(gbs_chrono.isRunning()){
              gbs_chrono.stop();  
            }
          }
          pos = 0;
          memset(buff,0xff,sizeof(buff));  
        }
      }    
    }
    if (get_buffers_state == GBS_READING){
      while(1){
        if(gbs_chrono.hasPassed(GBS_WAIT1*n_parts)){
          gbs_chrono.stop();
          get_buffers_state = GBS_READY;
          retval = 0;
          break; 
        }else{
          static byte chunk[GBS_CHUNK_SIZE];
          int chpos = 0;
          int chunk_len = sizeof(GBS_CHUNK_SIZE);
          int data_len = chunk_len-GBS_CHUNK_SRV_LEN;
          while(port.available()){
            chunk[chpos++] = port.read();
            if(chpos == GBS_CHUNK_SIZE){
              if(chkcrc8tag(tag,chunk,chpos)){
                int pix = chunk[0];
                if(pix < n_parts){
                  int cpy = data_len;
                  if((pix+1)*data_len > take){
                      cpy = data_len - ((pix+1)*data_len-take);
                  }
                  memcpy(mem+pix*data_len,chunk+1, cpy );           
                  taked += cpy;
                }
              }
            }
          }
        }  
        if (taked == take && taked > 0 ){
          gbs_chrono.stop();
          get_buffers_state = GBS_READY;
          retval = taked;  
          gbs_send_end_transmission(port, 1);
          break;
        }
      }
    }
  }
}

int gbs_send_ack(Stream& port, int msg_size, int buff_size){
  if(msg_size <= buff_size){
    byte msg[2]={GBS_ACK, GBS_CHUNK_SIZE};
    port.write(msg, sizeof(msg));
    int data_len = GBS_CHUNK_SIZE - GBS_CHUNK_SRV_LEN;
    return (msg_size+data_len-1)/data_len;
  }else{
      byte msg[2]={GBS_ERR, GBS_ERR_MSG_TOO_BIG};
      port.write(msg, sizeof(msg));
      return 0;
  }
}

void gbs_send_end_transmission(Stream& port, int is_ok ){
  if(is_ok){
    byte msg[2]={GBS_END, GBS_END};
    port.write(msg, sizeof(msg));
  }else{
    byte msg[2]={GBS_END, GBS_ERR};
    port.write(msg, sizeof(msg));
  }  
}

uint8_t gencrc8(uint8_t *buff, int len){
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= *buff++;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80){
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }else{
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool chkcrc8(uint8_t* b, int b_len){
  if(b_len <= 0){
    return false;  
  }
  return b[b_len-1] == gencrc8(b, b_len-1); 
}

uint8_t gencrc8tag(uint8_t tag, uint8_t *buff, int len){
    uint8_t crc = 0x00;
    crc ^= tag;
    for (int j = 0; j < 8; j++) {
        if (crc & 0x80){
            crc = (uint8_t)((crc << 1) ^ 0x07);
        }else{
            crc <<= 1;
        }
    }
    for (int i = 0; i < len; i++) {
        crc ^= *buff++;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80){
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }else{
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool chkcrc8tag(uint8_t tag, uint8_t* b, int b_len){
  if(b_len <= 0){
    return false;  
  }
  return b[b_len-1] == gencrc8tag(tag, b, b_len-1); 
}

void pcrc8tag( uint8_t tag, uint8_t* b, int b_len){
  if( b_len > 0){
    b[b_len-1] = gencrc8tag( tag, b, b_len-1);
  }
}
