#include <ESP8266WiFi.h>

extern "C" { 
  #include <espnow.h> 
}

#include "RC.h"

// board role. Server is receiver.
#define SERVER // define which board we flash, master or slave.

// receiver output
//#define SERVO
#define PWM
//#define PPM

#define WIFI_CHANNEL 4

volatile boolean recv;
volatile int peernum = 0;

void recv_cb(u8 *macaddr, u8 *data, u8 len)
{
  recv = true;
  //Serial.print("recv_cb ");
  //Serial.println(len); 
  if (len == RCdataSize)
  {
    for (int i=0;i<RCdataSize;i++) RCdata.data[i] = data[i];
  }
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
};

void send_cb(uint8_t* mac, uint8_t sendStatus) 
{
  //Serial.print("send_cb ");
};

void setup() 
{
  Serial.begin(115200); Serial.println();

  WiFi.mode(WIFI_STA); // Station mode for esp-now 
  WiFi.disconnect();

  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); 
  Serial.printf(", channel: %i\n", WIFI_CHANNEL); 

  if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  esp_now_register_recv_cb(recv_cb);
  esp_now_register_send_cb(send_cb);

  init_RC();
}

char hello[] = "hello world";

void loop() 
{
  if (recv)
  {
    recv = false;    
    buf_to_rc();
    writeServo();

    Serial.print(rcValue[0]); Serial.print("  ");
    Serial.print(rcValue[1]); Serial.print("  ");
    Serial.print(rcValue[2]); Serial.print("  ");
    Serial.print(rcValue[3]); Serial.println();

    //if (peernum > 0) esp_now_send(NULL, (u8*)hello, sizeof(hello)); // NULL means send to all peers
  }
  delay(10);
}


