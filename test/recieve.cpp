//Ryan Anderson

//Proxy Code

//Libraries
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int xpos;
  int ypos;
  int button;
} struct_message;
// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  float xpos = myData.xpos; // Convert from mA to A
  int ypos = myData.ypos;
  int button = myData.button;
}

void printMacAddress() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.print("Device MAC Address: ");
  for (int i = 0; i < 6; ++i) {
    if (i != 0) Serial.print(", ");
    Serial.print("0x");
    if (mac[i] < 0x10) Serial.print("0"); // pad single digit
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for Serial to be ready (for native USB)
  printMacAddress();
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return; }

  // Once ESPNow is successfully connected, data will be sent through Serial
  esp_now_register_recv_cb((OnDataRecv));



}
 
void loop() {

}