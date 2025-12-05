#include <Arduino.h>
#include <WiFi.h>                                          // for WiFi functions
#include <esp_wifi.h>                                      // for esp_wifi_set_channel
#include <Arduino.h>                                       // for Arduino functions
#include <esp_now.h>                                       // for ESP-NOW functions


#define RIGHT_PIN 34    // ADC pin for your reflectance sensor
#define LEFT_PIN 35    // ADC pin for your reflectance sensor
#define ENABLE_PIN 12 //pin for button enable
const int THRESHOLD = 3000;  // Trigger level

int rightled = 26;
int leftled  = 27;
int upled    = 14;
int downled  = 25;
int enableled = 13;

int d = 50;


//========== BLUETOOTH =========== //
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast to ALL devices reading for struct_message

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


//============= SETUP ============== //
void setup() {
  Serial.begin(115200);
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
  delay(200);

  analogSetPinAttenuation(RIGHT_PIN, ADC_11db); // full 0–4095 range
  analogSetPinAttenuation(LEFT_PIN, ADC_11db); // full 0–4095 range
  pinMode(INPUT_PULLUP, ENABLE_PIN);
  pinMode(rightled, OUTPUT);
  pinMode(leftled, OUTPUT);
  pinMode(upled, OUTPUT);
  pinMode(downled, OUTPUT);
  pinMode(enableled, OUTPUT);
}

//============= LOOP =============== //
void loop() {
  int right = analogRead(RIGHT_PIN);
  int left = analogRead(LEFT_PIN);
  int buttonstate = digitalRead(ENABLE_PIN);

    if (buttonstate == LOW) {
    Serial.println("DISABLED ");
    digitalWrite(rightled, LOW);
    digitalWrite(leftled, LOW);
    digitalWrite(upled, LOW);
    digitalWrite(downled, LOW);
    digitalWrite(enableled, HIGH);
  }

  if (right > THRESHOLD && left < THRESHOLD && buttonstate == HIGH) {
    Serial.println("Moving: Right ");
    myData.xpos = 1;
    myData.ypos = 0;
    digitalWrite(rightled, HIGH);
    digitalWrite(leftled, LOW);
    digitalWrite(upled, LOW);
    digitalWrite(downled, LOW);
    digitalWrite(enableled, HIGH);
    delay(d);
    digitalWrite(enableled, LOW);
    delay(d);
  }

    if (right < THRESHOLD && left > THRESHOLD && buttonstate == HIGH) {
    Serial.println("Moving: Left ");
    myData.xpos = -1;
    myData.ypos = 0;
    digitalWrite(rightled, LOW);
    digitalWrite(leftled, HIGH);
    digitalWrite(upled, LOW);
    digitalWrite(downled, LOW);
    digitalWrite(enableled, HIGH);
    delay(d);
    digitalWrite(enableled, LOW);
    delay(d);
  }

    if (right > THRESHOLD && left > THRESHOLD && buttonstate == HIGH) {
    Serial.println("Moving: Up ");
    myData.xpos = 0;
    myData.ypos = 1;
    digitalWrite(rightled, LOW);
    digitalWrite(leftled, LOW);
    digitalWrite(upled, HIGH);
    digitalWrite(downled, LOW);
    digitalWrite(enableled, HIGH);
    delay(d);
    digitalWrite(enableled, LOW);
    delay(d);
  }

    if (right < THRESHOLD && left < THRESHOLD && buttonstate == HIGH) {
    Serial.println("Moving: Down ");
    myData.xpos = 0;
    myData.ypos = -1;
    digitalWrite(rightled, LOW);
    digitalWrite(leftled, LOW);
    digitalWrite(upled, LOW);
    digitalWrite(downled, HIGH);
    digitalWrite(enableled, HIGH);
    delay(d);
    digitalWrite(enableled, LOW);
    delay(d);
  }

//delay for stability
  delay(10);

    // send data to proxy
  esp_err_t result = esp_now_send(broadcastAddress,       // Broadcast data to nearby ESP32s
      (uint8_t *)&myData, sizeof(myData));                  
    if (result == ESP_OK) {                                 // If data sent successfully, print the data in serial monitor
      Serial.printf("XPOS: %d | YPOS: %d ",
          myData.xpos, myData.ypos);
    } else {                                                // If data not sent successfully, print error message in serial monitor
      Serial.printf("Error sending data\n");
    }
}
