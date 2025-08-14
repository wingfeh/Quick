#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// PERFORMANCE OPTIMIZED FIRMWARE - MEGA3
// File: mega3.cpp
// IP: 192.168.100.103
// MAC: 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03
// Minimal overhead, maximum speed

// Network configuration - MEGA3
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03};
IPAddress ip(192, 168, 100, 103);

// UDP configuration
unsigned int localPort = 8888;
char packetBuffer[16];  // Minimal buffer size
EthernetUDP Udp;

// Relay configuration
const int RELAY_PIN = 22; // Standardized to pin 22 like mega1 and mega2
bool relayState = false;

// Sensor configuration
const int SENSOR_PIN = 23;

// Pre-computed responses for maximum speed (no string operations)
const char* RESP_RELAY_ON_OK = "RELAY_ON:OK";
const char* RESP_RELAY_OFF_OK = "RELAY_OFF:OK"; 
const char* RESP_STATUS_ON = "STATUS:ON";
const char* RESP_STATUS_OFF = "STATUS:OFF";
const char* RESP_SENSOR_HIGH = "SENSOR:HIGH";
const char* RESP_SENSOR_LOW = "SENSOR:LOW";
const char* RESP_ERROR = "ERROR";

// Fast inline response function
inline void sendResponse(const char* response) {
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(response);
  Udp.endPacket();
}

void setup() {
  // Minimal setup for maximum speed
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  // Setup sensor pin
  pinMode(SENSOR_PIN, INPUT);
  
  // Fast ethernet init (no gateway/subnet needed for point-to-point)
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  
  // Serial debugging enabled
  Serial.begin(115200);
  Serial.println("OPTIMIZED RELAY & SENSOR CONTROLLER MEGA3 READY");
  Serial.print("IP: ");
  Serial.println(ip);
}

void loop() {
  // Ultra-fast packet processing
  int packetSize = Udp.parsePacket();
  
  if (packetSize > 0) {
    // Read packet directly without length checks for speed
    int len = Udp.read(packetBuffer, 15);
    packetBuffer[len] = 0;
    
    // Fast command processing using first character optimization
    switch (packetBuffer[0]) {
      case 'R':
        if (packetBuffer[6] == 'O' && packetBuffer[7] == 'N') {  // RELAY_ON
          digitalWrite(RELAY_PIN, HIGH);
          relayState = true;
          sendResponse(RESP_RELAY_ON_OK);
        } else if (packetBuffer[6] == 'O' && packetBuffer[7] == 'F') {  // RELAY_OFF
          digitalWrite(RELAY_PIN, LOW);
          relayState = false;
          sendResponse(RESP_RELAY_OFF_OK);
        } else {
          sendResponse(RESP_ERROR);
        }
        break;
        
      case 'S':
        if (packetBuffer[1] == 'T') {  // STATUS
          sendResponse(relayState ? RESP_STATUS_ON : RESP_STATUS_OFF);
        } else if (packetBuffer[1] == 'E') {  // SENSOR
          bool sensorValue = digitalRead(SENSOR_PIN);
          sendResponse(sensorValue ? RESP_SENSOR_HIGH : RESP_SENSOR_LOW);
        }
        break;
        
      default:
        sendResponse(RESP_ERROR);
        break;
    }
  }
  
  // No delays, no heartbeat, no unnecessary processing
}
