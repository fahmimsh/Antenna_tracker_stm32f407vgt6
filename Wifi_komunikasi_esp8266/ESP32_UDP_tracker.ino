#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udpconn;
uint8_t inPacket[255], inUDP[255];

#define WIFI_AP_SSID "Efalcon Quadcopter"
#define WIFI_AP_PASS "efrisa2019"

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(131);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 6, false, 3);
  udpconn.begin(60111);
}

int byteSize=0;
long lastmil = 0;
void loop() {
  int packSize = udpconn.parsePacket();
  if(packSize){
//    Serial.printf("Received %d bytes from %s, port %d\n", packSize, udpconn.remoteIP().toString().c_str(), udpconn.remotePort());
    int len = udpconn.read(inUDP, 255);
//    Serial.print("Recv: ");
    Serial.write(inUDP,len);
    Serial.println();
  }
  if(Serial.available()){
    byteSize = Serial.readBytesUntil('\n',inPacket,255);
//    Serial.write(inPacket,byteSize);
//    Serial.println("|ByteSize: "+String(byteSize));
    sendToDest();
  }
  if(millis() - lastmil > 2000){
    Serial.println();
    lastmil = millis();
  }
}

void sendToDest(){
    udpconn.beginPacket("192.168.4.2", 9601);
    udpconn.write(inPacket,byteSize);
    udpconn.endPacket();
    udpconn.beginPacket("192.168.4.3", 9601);
    udpconn.write(inPacket,byteSize);
    udpconn.endPacket();
    byteSize=0;
}
