/*
 * Ryan Siegel
 * University of Vermont
 * Department of Biomedical Engineering
 * CMPE 3815: Microcontroller Systems
 * 2025 NOV 12
 * The purpose of this script is to:
 * Receive ESP-NOW data from watch sensor bridge, and use to control 2axis servo
 * 
 * The hardware set-up is:
 * Servo X on pin 32, Servo Y on pin 33
 * Buttons on 18, 19, 21 
 * joystick x on 35, joystick y on 34
 * laser on 2
 */

// INIT ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+===
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

// declare servo
Servo servoX;
Servo servoY;
const int pinServoX = 32; 
const int pinServoY = 33;

// setting bounding box 
const int btnPin1 = 18;
const int btnPin2 = 19;
const int btnPin3 = 21;

// joystick for bounding box
const int pinJSX = 35; 
const int pinJSY = 34;

// set laser on/off
const int pinLaser = 25;

// servo max PWM bounds
const int PWMMIN = 500;
const int PWMMAX = 2500;

// bounding box defined limits
int xMin = 1500; // starts at 90 deg
int xMax = 1500;
int yMin = 1500;
int yMax = 1500;


// Structure to receive data - must match sender
typedef struct struct_message {
  int xpos;
  int ypos;
  int button;
} struct_message;
struct_message myData;

// Parse Data from BT
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}


// SETUP ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+===
void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP-NOW Servo Controller...");
  
  // Initialize WiFi and ESP-NOW first
  WiFi.mode(WIFI_STA);
//  printMacAddress();
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register callback with correct signature
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW initialized successfully");

  // Initialize servo system
  analogReadResolution(12);


  // set ESP32 PWM settings for Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // set PWM frequency
  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);

  // attach servo 
  servoX.attach(pinServoX, PWMMIN, PWMMAX);// change PWM pulses
  servoY.attach(pinServoY, PWMMIN, PWMMAX);
  
  // set servo to middle
  servoX.write(90);
  servoY.write(90);

  Serial.println("Servo Initialized");

  // set up joystick for boudning box
  pinMode(pinJSX, INPUT);
  pinMode(pinJSY, INPUT);

  // set button pins
  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(btnPin2, INPUT_PULLUP);
  pinMode(btnPin3, INPUT_PULLUP);


  // set laser
  pinMode(pinLaser, OUTPUT);
  digitalWrite(pinLaser, HIGH);


  Serial.println("Setup complete - ready to receive commands");
}



// FUNCTIONS ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+===
void writeServos(Servo &servo, int angPWM, int angMin, int angMax){
  // Write a given angle to the specified servo, ensuring within the 
  // bounds given (defined/set via Joystick drawing)
  
  // make sure angle given is in range
  angPWM = constrain(angPWM, angMin, angMax);

  // write angles to servo
  servo.writeMicroseconds(angPWM);
  return;
}



byte button_debounce(int pin, bool prior) {
  bool currentVal = digitalRead(pin);
  if (currentVal != prior) {
    delay(10); // wait for bounce
    currentVal = digitalRead(pin);
  }
  return currentVal;
}



// LOOP ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+=== ===+===
void loop() {
  //track angle & set initital position to center
  static int degXpwm = 1500;
  static int degYpwm = 1500;
  
  // read buttons
  static bool btn1Prior = 0;
  static bool btn2Prior = 0;
  static bool btn3Prior = 0;
  bool btn1val = button_debounce(btnPin1, btn1Prior);
  bool btn2val = button_debounce(btnPin2, btn2Prior);
  bool btn3val = digitalRead(btnPin3);

  // button 1 swithces between sensor & joystick
  static bool setState = 0; // 0-joystick, 1-wrist
  if (!btn1val && btn1val != btn1Prior) 
  {
    setState = !setState;

    // reset lims when switched to joystick mode
    if (!setState)
    {
      xMin = degXpwm;
      xMax = degXpwm;
      yMin = degYpwm;
      yMax = degYpwm;
    }
  }
  
  // joystick button removes limits
  if (!btn3val) 
  {
    xMin = PWMMIN;
    xMax = PWMMAX;
    yMin = PWMMIN;
    yMax = PWMMAX;
  }
  

  // button 2 toggles laser
  static bool laserState = 1;
  if (!btn2val && btn2val!= btn2Prior) 
  {
    laserState= !laserState;
    digitalWrite(pinLaser, laserState);
  }

  // store button priors
  btn1Prior = btn1val;
  btn2Prior = btn2val;
  btn3Prior = btn3val;
//  delay(2); 

  
  // ------------------- Control Servos ------------------- 

  // ------- servo via sensors -------
  if (setState)
  {
    // Get data from ESP-NOW
    int xpos = myData.xpos;
    int ypos = myData.ypos;
    int btn = myData.button;
  
    // update position from sensor bridge
    degXpwm += xpos;
    degYpwm += ypos;
  
    // constrain degrees
    degXpwm = constrain(degXpwm, xMin, xMax);
    degYpwm = constrain(degYpwm, yMin, yMax);
  
    // debug/view
    Serial.print("<< Sensor Mode >>  ");
    Serial.print(xpos); Serial.print(", ");
    Serial.print(ypos); Serial.print(", ");
    Serial.print(btn);
  
    Serial.print(" | Servo X: "); Serial.print(degXpwm);
    Serial.print(" | Servo Y: "); Serial.println(degYpwm);
  
    // set servos
    writeServos(servoX, degXpwm, xMin, xMax);
    writeServos(servoY, degYpwm, yMin, yMax);
  }

  
  // ------- servos via joystick -------
  else if (!setState)
  {
    // read values
    int potXVal = analogRead(pinJSX);
    int potYVal = analogRead(pinJSY);
  
    // Joystick processing parameters
    const int potLo = 20;
    const int potHi = 4070;
    const int degStep = 10; // 10us increments when outside deadzone


    // Process joystick input
    if (potXVal < potLo) degXpwm += degStep;
    else if (potXVal > potHi) degXpwm -= degStep;
    if (potYVal < potLo) degYpwm -= degStep;
    else if (potYVal > potHi) degYpwm += degStep;

    // constrain degrees
    degXpwm = constrain(degXpwm, PWMMIN, PWMMAX);
    degYpwm = constrain(degYpwm, PWMMIN, PWMMAX);
    
    // write servos based on joystick
    writeServos(servoX, degXpwm, PWMMIN, PWMMAX);
    writeServos(servoY, degYpwm, PWMMIN, PWMMAX);

    // set bounds based on current position
    xMin = min(xMin, degXpwm);
    xMax = max(xMax, degXpwm);
    yMin = min(yMin, degYpwm);
    yMax = max(yMax, degYpwm);

    Serial.print("<< Joystick Mode >>  ");
    Serial.print(potXVal); Serial.print(", ");
    Serial.print(potYVal);
  
    Serial.print(" | Servo X: "); Serial.print(degXpwm);
    Serial.print(" | Servo Y: "); Serial.print(degYpwm);

    Serial.print(" | X-lims: "); Serial.print(xMin); Serial.print(","); Serial.print(xMax);
    Serial.print(" | Y-lims: "); Serial.print(yMin); Serial.print(","); Serial.println(yMax);
  }


  
}

// ===+=== ===+=== ===+=== ===+=== ===+=== END OF SKETCH ===+=== ===+=== ===+=== ===+=== ===+===
