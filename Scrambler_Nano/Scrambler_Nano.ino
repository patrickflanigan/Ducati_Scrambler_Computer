// Scrambler Board Software
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BotleticsSIM7000.h" // https://github.com/botletics/Botletics-SIM7000/tree/main/src

// For botletics SIM7000 shield
#define SIMCOM_7000
#define BOTLETICS_PWRKEY 6
#define RST 7
#define TX 12 // Microcontroller RX
#define RX 11 // Microcontroller TX

// ------------------------------------------- GPS / Modem ------------------------------------------
// this is a large buffer for replies
char replybuffer[255];

SoftwareSerial modemSS = SoftwareSerial(TX, RX);
SoftwareSerial *modemSerial = &modemSS;
Botletics_modem_LTE modem = Botletics_modem_LTE();
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!

//=-------------------------------------------------- MPU6050 IMU
// Create instance of the MPU6050 IMU
MPU6050 mpu;

// ---------------------------------------------------------------------- Scrambler State Machine -------------------------------------------
enum State {INITIAL, OFF, ON, SENTRY, ALARMING, SCANNING, STARTING, RUNNING};
State Scrambler_State = INITIAL;
State Previous_State = INITIAL;
enum Scan_Options {NONE, PAT};
Scan_Options Scan_Results = NONE;
// ---------------------------------------------------------------------- Pinouts ----------------------------------------------------------
//const int Red_LED  =  10;     //Digital I/O  Pin 10 for Red LED
const int Blue_LED  =  9;     //Digital I/O  Pin 10 for Blue LED
const int Green_LED  =  8;     //Digital I/O  Pin 10 for Green LED
const int RDM_PWR_Relay  =  7;     //Digital I/O  Pin 7 to turn on/off relay for RDM6300 power
const int Batt_Read_Relay  =  6;     //Digital I/O  Pin 6 to turn on/off relay for Bike Battery Voltage Divider
const int IMU_Interrupt_Pin  =  5;     //Digital I/O  Pin 6 to turn on/off relay for Bike Battery Voltage Divider
const int Rx_From_RDM  =  4;     //use digital I/O pin 4
const int SIM7000A_Power_Pin  =  3;     //use digital I/O pin 8
/*
This pin is connected to the SIM7000's "power key" (PWRKEY) pin. Pulsing it LOW for at least 72ms then setting it back to HIGH will turn on the SIM7000 if it was OFF previously. 
Note that there is a 4.2s lag before the green power LED will indicate that the module is on after you have initiated the turn-on pulse. 
To turn it OFF from being ON you have to pulse it for at least 1.2s, after which the power LED will lag about 1.3s to turn off. 
Botletics SIM7000 shield v6 have a button next to the "PWR" LED that allows you to turn it on easily by pressing and holding the button.
*/

int Start_Relay = A1;


// ------------------------------------------------------------------------- RDM6300 Buffer Stuff Etc. ------------------------------------------
const int BUFFER_SIZE = 14; // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_SIZE = 10; // 10byte data (2byte version + 8byte tag)
const int DATA_VERSION_SIZE = 2; // 2byte version (actual meaning of these two bytes may vary)
const int DATA_TAG_SIZE = 8; // 8byte tag
const int CHECKSUM_SIZE = 2; // 2byte checksum
const int RDM_PWR = 2; //Using digital pin D2 for RDM relay high/low
SoftwareSerial ssrfid = SoftwareSerial(Rx_From_RDM, 5); 
uint8_t buffer[BUFFER_SIZE]; // used to store an incoming data frame 
int buffer_index = 0;
String tagid = "";
//-------------------------------------------------------------------------- Some Globals -----------------------------------------------------------
unsigned long starttime;  // Generally used for timers
unsigned long currenttime;  //  Generally used for timers
unsigned long scan_time;
unsigned long locked_scantime = 20000;
unsigned long off_scantime = 30000;
int texts_sent = 0;

const int LED_HIGH = 0;
const int LED_LOW = 1;
int Battery_Voltage = 0;
int Alarm_Counter = 0;

float Wake_Button_Value = 0;


void setup()
{
// ------------------------------------ Configuring Inputs/Outputs ----------------
 //pinMode(Red_LED, OUTPUT);
 pinMode(Blue_LED, OUTPUT);
 pinMode(Green_LED, OUTPUT);
 pinMode(RDM_PWR_Relay, OUTPUT); 
 pinMode(Batt_Read_Relay, OUTPUT);
 pinMode(IMU_Interrupt_Pin, INPUT);
 pinMode(SIM7000A_Power_Pin, OUTPUT);

 pinMode(A0, OUTPUT); // Bike ON
 pinMode(Start_Relay, OUTPUT); // Bike START Relay
 pinMode(A2, INPUT); // Button
 pinMode(A3, INPUT); // Kickstand Voltage Divider
 pinMode(A6, INPUT); // Battery Voltage Divider
 pinMode(A7, OUTPUT); // 5V -> 4V Buck Converter


 //digitalWrite(Red_LED, LED_LOW); // 
 digitalWrite(Blue_LED, LED_LOW); // 
 digitalWrite(Green_LED, LED_LOW); // 


 digitalWrite(Start_Relay, LOW); // 
 digitalWrite(RDM_PWR_Relay, LOW); // 
 digitalWrite(Batt_Read_Relay, LOW); // 
 digitalWrite(SIM7000A_Power_Pin, LOW); // 

 digitalWrite(2, LOW); // 
 digitalWrite(Start_Relay, LOW); // 
 digitalWrite(A2, LOW); // 



//----------------------------------- RDM6300 Init ------------------------
 Serial.begin(9600); // Serial Begin
 ssrfid.begin(9600); // SSRFID Begin
 Serial.println("System Initialized");


// --------------------------------------- MPU 6050 init (IMU) -------------------------
    Wire.begin();
    mpu.initialize();
    // Optional: mpu.setInterruptPin(12); // Set interrupt pin, if used

// --------------------------------- MODEM / GPS
  pinMode(RST, OUTPUT);
  //digitalWrite(RST, HIGH); // Default state
  pinMode(BOTLETICS_PWRKEY, OUTPUT);
  //modem.powerOn(BOTLETICS_PWRKEY); // Power on the module
  Serial.println(F("Modem basic test"));
  Serial.println(F("Initializing....(May take several seconds)"));
  // Software serial:
  modemSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  modemSS.println("AT+IPR=9600"); // Set baud rate
  delay(25);

  delay(100); // Short pause to let the command run
  modemSS.begin(9600);
  if (! modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    while (1); // Don't proceed if it couldn't find the device
  }
  type = modem.type();
  Serial.println(F("Modem is OK"));
  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  // Set modem to full functionality
  modem.setFunctionality(1); // AT+CFUN=1
  modem.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  //modem.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  //modem.setNetLED(false); // Disable network status LED


} // End Setup

void loop(){
  
  switch (Scrambler_State) {
    case INITIAL:
      Previous_State = INITIAL;
      Scrambler_State = OFF;
      Serial.println("The Scrambler is in OFF State!");
      break;

    case OFF:
      //Arduino in sleep mode
      Wake_Button_Value = analogRead(A2);
      //Serial.println(Wake_Button_Value);
      if (Wake_Button_Value > 1020) { // This actually has to be done with an interrupt!
        starttime = millis();  //  Start a timer
        currenttime = starttime;  //  Current Time (to be updated constantly)

        while(true){
        if (currenttime - starttime > 1500){
          Wake_Button_Value = analogRead(A2);
          if (Wake_Button_Value > 1020){
            Scrambler_State = SENTRY;
            Previous_State = OFF;
            Serial.println("Long Button Press/Hold Detected: The Scrambler is switching into SENTRY State!");
            break;
          }
          else{
            Scrambler_State = SCANNING;
            Previous_State = OFF;
            Serial.println("Short Button Press Detected: The Scrambler is switching into SCANNING State!");
            break;
          }
        }
        currenttime = millis();
        }
      }
      break;
    
    case ON:
      Serial.print("Bike is in ON state!");
      // Bike is ON
      break;

    case SENTRY:
      // Blink Blue LED (slowely) <----------------------------------------------------- ********************************************************
      digitalWrite(Blue_LED, LED_HIGH);
      delay(200);
      digitalWrite(Blue_LED, LED_LOW);

      // This needs to be reworked <---------------------------------------------------- *********************************************************
      Wake_Button_Value = analogRead(A2);
      Serial.println(Wake_Button_Value);
      //Serial.println(Wake_Button_Value);
      if (Wake_Button_Value > 1020) { // This actually has to be done with an interrupt!
            Scrambler_State = SCANNING;
            Previous_State = SENTRY;
            Serial.println("Short Button Press Detected: The Scrambler is switching into SCANNING State (from SENTRY).");
            break;
          }

      // Read raw accelerometer and gyroscope data
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.println(abs(gx+gy+gz));

        if(abs(gx+gy+gz) > 750){
          Alarm_Counter ++;

          if(Alarm_Counter > 100){
            Serial.println("The bike alarm was tripped!, Switching Scrambler to ALARMING State!");
            Alarm_Counter = 0; // Good form to reset this I reckon...
            Scrambler_State = ALARMING;
            Previous_State = SENTRY;
            break;
          }
        }
      break;

    case SCANNING:
      {
      ssrfid.listen();
      digitalWrite(RDM_PWR_Relay, HIGH); // 
      delay(100);
      starttime = millis();  //  Start a timer
      currenttime = starttime;  //  Current Time (to be updated constantly)
      if (Previous_State == OFF){scan_time = off_scantime;}  // If the bike was OFF (not 'locked') in previous state, scan for 60 seconds
      else {scan_time = locked_scantime;};  //  If the bike was in SENTRY ('locked') previous state, scan for 20 seconds

      while (currenttime - starttime < scan_time)  //test whether the period has elapsed
      {
        //digitalWrite(Blue_LED, LED_HIGH); // 
      //----------------------------------------------------- RFID Search ------------------------------------------------------------------------
        if (ssrfid.available() > 0){
          Serial.println("Serial Available");
          digitalWrite(Blue_LED, LED_HIGH); // 
          bool call_extract_tag = false;
          
          int ssvalue = ssrfid.read(); // read 
          if (ssvalue == -1) { // no data was read
            return;
          }

          if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
            buffer_index = 0;
          } else if (ssvalue == 3) { // tag has been fully transmitted       
            call_extract_tag = true; // extract tag at the end of the function call
          }

          if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
            Serial.println("Error: Buffer overflow detected!");
            return;
          }
          
          buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer

          if (call_extract_tag == true) {
            if (buffer_index == BUFFER_SIZE) {
              unsigned tag = extract_tag();
              Serial.print("Found an RFID Tag!");
              //Serial.print(tagid);
              if (tagid == "xxxxxxxxxxxxxxxxx") {
                Serial.println("Identified Pat's RFID Tag!");
                Scan_Results = PAT;
                digitalWrite(Green_LED, LED_HIGH); // 
              }
            } else { // something is wrong... start again looking for preamble (value: 2)
              buffer_index = 0;
              return;
            }
          }    
          tagid = "";
        }

        currenttime = millis(); // Update the timer, waiting to kill the search process   
        
        if (Scan_Results == PAT){
          Serial.println("Pat Found! Stopping the epic search.");
          break;
        }
      //----------------------------------------------------- End RFID Search ------------------------------------------------------------------------
      } // End while loop

      digitalWrite(Blue_LED, LED_LOW);

      digitalWrite(RDM_PWR_Relay, LOW); // 
      delay(100);

      if (Scan_Results == PAT){
        Serial.println("The Scrambler is switching into STARTING State!");
        Previous_State = SCANNING;
        Scrambler_State = STARTING;
        break;
      }

      Serial.print("No Tag Found, Switching to ");
      if(Previous_State == 1){
        Serial.println("OFF State");
      }
      Scrambler_State = Previous_State;
      Previous_State = SCANNING;
      break;
      }



    case ALARMING:
      Serial.println("Bike is in ALARMING state!");
      modemSS.listen();
          // Flash Blue and Red LEDs quickly back and forth
      // send an SMS!
      //char sendto[21], message[141];
      
      char sendto[] = "xxxxxxxxxxxxxxxx";
      char message[] = "This is the Scrambler! The alarm has been tripped! Please check out the Bike!";
      Serial.print("Sending Text Message : ");
      Serial.println(message);
      if (!modem.sendSMS(sendto, message)) {
        Serial.println(F("Failed"));
        texts_sent += 1;
        delay(500);
      } else {
        Serial.print("Texts sent :");
        Serial.println(texts_sent);
        Serial.println(F("Sent!"));
        texts_sent += 1;
        delay(500);
      }
      if (texts_sent >= 5){
        Scrambler_State = SENTRY;
        Previous_State = ALARMING;
      }
      break;

    case STARTING:
      delay(1000);
      digitalWrite(Start_Relay, HIGH); // Provide power to the start relay of the bike
      delay(2000);
      digitalWrite(Start_Relay, LOW); // Remove power to the start relay of the bike
      delay(2500);
      
      Serial.println("Checking if the bike successfully started");
      Battery_Voltage = checkbatteryvoltage();
      Serial.print("Is bike on? : ");
      Serial.println(Battery_Voltage);

      if (Battery_Voltage == 1){ // If the Bike has started sucessfully (is running)
        Serial.println("The Scrambler started! Switching into RUNNING State!");
        Scrambler_State = RUNNING;
        Previous_State = STARTING;
      }
      else{
        Serial.println("The Scrambler did not start :( Switching into Previous State! (SCANNING State)");
        Scrambler_State = Previous_State;
        Previous_State = STARTING;
      }
      break;
      

    case RUNNING:
      Serial.print("Bike is in RUNNING state!");
      starttime = millis();  //  Start a timer
      currenttime = starttime;  //  Current Time (to be updated constantly)
      while(currenttime - starttime < 60000){
        delay(200);
        currenttime = millis();
      }
      Battery_Voltage = checkbatteryvoltage();
      if (Battery_Voltage == 0){
        Scrambler_State = OFF;
        Previous_State = RUNNING;
      }
      break;

  }
} // End Main Loop












// ------------------------------------------------------------------- RFID Search & Identify Functions ----------------------------------------------------------
unsigned extract_tag() {
    uint8_t msg_head = buffer[0];
    uint8_t *msg_data = buffer + 1; // 10 byte => data contains 2byte version + 8byte tag
    uint8_t *msg_data_version = msg_data;
    uint8_t *msg_data_tag = msg_data + 2;
    uint8_t *msg_checksum = buffer + 11; // 2 byte
    uint8_t msg_tail = buffer[13];

    // print message that was sent from RDM630/RDM6300
    // Serial.println("--------");

    // Serial.print("Message-Head: ");
    // Serial.println(msg_head);

    // Serial.println("Message-Data (HEX): ");
    for (int i = 0; i < DATA_VERSION_SIZE; ++i) {
      // Serial.print(char(msg_data_version[i]));
    }
    // Serial.println(" (version)");
    for (int i = 0; i < DATA_TAG_SIZE; ++i) {
      // Serial.print(char(msg_data_tag[i]));
      tagid.concat(msg_data_tag[i]);
    }
    // Serial.println(" (tag)");

    // Serial.print("Message-Checksum (HEX): ");
    for (int i = 0; i < CHECKSUM_SIZE; ++i) {
      // Serial.print(char(msg_checksum[i]));
    }
    // Serial.println("");

    // Serial.print("Message-Tail: ");
    // Serial.println(msg_tail);

    // Serial.println("--");

    long tag = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);
    // Serial.print("Extracted Tag: ");
    // Serial.println(tag);

    long checksum = 0;
    for (int i = 0; i < DATA_SIZE; i+= CHECKSUM_SIZE) {
      long val = hexstr_to_value(msg_data + i, CHECKSUM_SIZE);
      checksum ^= val;
    }
    // Serial.print("Extracted Checksum (HEX): ");
    // Serial.print(checksum, HEX);
    if (checksum == hexstr_to_value(msg_checksum, CHECKSUM_SIZE)) { // compare calculated checksum to retrieved checksum
      // Serial.print(" (OK)"); // calculated checksum corresponds to transmitted checksum!
    } else {
      // Serial.print(" (NOT OK)"); // checksums do not match
    }

    // Serial.println("");
    // Serial.println("--------");

    return tag;
}

long hexstr_to_value(char *str, unsigned int length) { // converts a hexadecimal value (encoded as ASCII string) to a numeric value
  char* copy = malloc((sizeof(char) * length) + 1); 
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; 
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol(copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  free(copy); // clean up 
  return value;
}
// ------------------------------------------------------------------- End RFID Search & Identify Functions ----------------------------------------------------------


// Check Bike Batter Voltage Function
int checkbatteryvoltage(){
      //Read Bike Battery Voltage 
      digitalWrite(Batt_Read_Relay, HIGH);
      delay(250);
      int Battery_Voltage = analogRead(A6); // This will need some scaling factor FOR SURE
      Serial.print("Battery Voltage Reading :");
      Serial.println(Battery_Voltage);
      digitalWrite(Batt_Read_Relay, LOW);
      delay(250);

      if (Battery_Voltage > 12){ // If the Bike has started sucessfully (is running)
        return(1);
      }
      else{
        return(0);
      }
}


// -------------------------Modem / GPS functions --------------


void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}

uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}


