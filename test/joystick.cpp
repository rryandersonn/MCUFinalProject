// include libraries
#include <Arduino.h>
#include <WiFi.h>                                          // for WiFi functions
#include <esp_wifi.h>                                      // for esp_wifi_set_channel
#include <Arduino.h>                                       // for Arduino functions
#include <esp_now.h>                                       // for ESP-NOW functions

int joystickButton = 5; // GPIO5 for joystick button
int joystickButtonState = 0; 

int xpin = 39; // GPIO39 for X axis
int ypin = 36; // GPIO36 for Y axis

int xpos = 0; // X position of joystick
int ypos = 0; // Y position of joystick

// Proxy MAC address (replace with your proxy's MAC)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast to ALL devices reading for struct_message

// set up bluetooth structure - esp now
typedef struct struct_message {
  int xpos;
  int ypos;
  int button;
} struct_message;

struct_message myData;                                      // name the structured message as myData
esp_now_peer_info_t peerInfo;                               // name the peer information structure as peerInfo

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");                            // Callback when data is sent, Define if the function sent the data successfully or not
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(9600);                                       //set baud rate to 9600
  WiFi.mode(WIFI_STA);                                      // Set device as a Wi-Fi Station (testing purposes)
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);           // Set WiFi channel to 1 (or your chosen channel)
  Serial.println("Setup Starting...");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {                           //check if esp_now_init() returns ESP_OK
    Serial.println("Error initializing ESP-NOW Error 1");   //this should always be true -- if this fails,
    return;                                                 //restart the ESP32. If it fails again then there is a deeper issue 
  }
  esp_now_register_send_cb(OnDataSent); 

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;                                     //set peer (proxy) channel to 0 (default)
  peerInfo.encrypt = false;                                 //no encryption
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {              //check if esp_now_add_peer() returns ESP_OK
    Serial.println("Failed to add peer");                   //if it fails, there is a problem beyond a code issue
    return;                                                 //restart ESP32, if this fails there is a deeper issue
  }

  //initlizae joystick button and x/y pins
  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(joystickButton, INPUT_PULLUP);
  

  Serial.println("Setup Complete");                        // Indicate setup is complete
}


void loop() {
  myData.xpos = analogRead(xpin); // read x position and send to myData.xpos
  myData.ypos = analogRead(ypin); // read y position and send to myData.ypos
  
  
  myData.button = digitalRead(joystickButton);
  joystickButtonState = digitalRead(joystickButton);

  // send data to proxy
  esp_err_t result = esp_now_send(broadcastAddress,       // Broadcast data to nearby ESP32s
      (uint8_t *)&myData, sizeof(myData));                  
    if (result == ESP_OK) {                                 // If data sent successfully, print the data in serial monitor
      Serial.printf("XPOS: %d | YPOS: %d | BUTTON: %d\n",
          myData.xpos, myData.ypos, myData.button);
    } else {                                                // If data not sent successfully, print error message in serial monitor
      Serial.printf("Error sending data\n");
    }
}

