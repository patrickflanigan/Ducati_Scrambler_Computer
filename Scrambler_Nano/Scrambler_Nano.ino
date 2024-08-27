// Scrambler Board Software
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BotleticsSIM7000.h" // https://github.com/botletics/Botletics-SIM7000/tree/main/src 

// For botletics SIM7000 shield
#define SIMCOM_7000
#define BOTLETICS_PWRKEY 5
#define TX 12 // Microcontroller RX
#define RX 11 // Microcontroller TX

// ------------------- Notes from Kush Shah ----------------------------------------------------
/*
-Anything that uses a checked state (such as check battery voltage or button state) doesn't need a global) anything current should just be a function call.
or make them a local variable
-LED on /off make a function for.
- setstate function (pass state variable);
*/



// ------------------------------------------- GPS / Modem ----------------------------------------------------------------------------
// this is a large buffer for replies
char replybuffer[255];

SoftwareSerial modemSS = SoftwareSerial(TX, RX);
SoftwareSerial *modemSerial = &modemSS;
Botletics_modem_LTE modem = Botletics_modem_LTE();
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
char passbuffer[5] = {'X','X','X','X','X'};

// -------------------------------------------------- MPU6050 IMU -------------------------------------------------------------------------
// Create instance of the MPU6050 IMU
MPU6050 mpu;
bool State_initialized = false;

// ---------------------------------------------------------------------- Scrambler State Machine -------------------------------------------
enum State {INITIAL, OFF, ON, STANDBY, SENTRY, ALARMING, SCANNING, STARTING, RUNNING};
State Scrambler_State = INITIAL;
State Previous_State = INITIAL;
enum Scan_Options {NONE, PAT};
Scan_Options Scan_Results = NONE;

// ---------------------------------------------------------------------- General Pinouts ----------------------------------------------------------
const int Red_LED  =  10;     //Digital I/O  Pin 10 for Red LED
const int Blue_LED  =  9;     //Digital I/O  Pin 10 for Blue LED
const int Green_LED  =  8;     //Digital I/O  Pin 10 for Green LED
const int RDM_PWR_Relay  =  7;     //Digital I/O  Pin 7 to turn on/off relay for RDM6300 power
const int Rx_From_RDM  =  4;     //use digital I/O pin 4
/*
This pin is connected to the SIM7000's "power key" (PWRKEY) pin. Pulsing it LOW for at least 72ms then setting it back to HIGH will turn on the SIM7000 if it was OFF previously. 
Note that there is a 4.2s lag before the green power LED will indicate that the module is on after you have initiated the turn-on pulse. 
To turn it OFF from being ON you have to pulse it for at least 1.2s, after which the power LED will lag about 1.3s to turn off. 
Botletics SIM7000 shield v6 have a button next to the "PWR" LED that allows you to turn it on easily by pressing and holding the button.
*/
int Start_Relay = 6;
int Battery_Read_Relay = A1;
int Bike_ON_Relay = A0;

// ------------------------------------------------------------------------- RFID Reader ------------------------------------------
SoftwareSerial ssrfid = SoftwareSerial(2, Rx_From_RDM); 
int buffer[14];       // buffer array for data receive over serial port
int targetbuffer[14] = {XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX};
int count = 0;                  // counter for buffer array


//-------------------------------------------------------------------------- Some Globals -----------------------------------------------------------
unsigned long starttime;  // Generally used for timers
unsigned long currenttime;  //  Generally used for timers
unsigned long scan_time;
unsigned long locked_scantime = 20000;
unsigned long off_scantime = 30000;
int texts_sent = 0;

const int LED_HIGH = 0;
const int LED_LOW = 1;
float Raw_Battery_Reading = 0;
float Voltage_Reading = 0;
float Battery_Voltage = 0;
int Alarm_Counter = 0;

// For button debouncing
int Button_Input = A2;
int rawbuttonreading = 0;
int currentButtonState = 0;
int lastButtonState = 0;
unsigned long lastDebounceTime = 0;
int debounceDelay = 225;

int ButtonState = 0;
int PreviousButtonState = 0;
int ButtonCount = 0;
int ButtonHoldThreshold = 2500;
int InterPressThreshold = 2000;
unsigned long PH_Timer = 0;
unsigned long Post_Press_Timer = 0;

//-------------------------------------------------------------------------------------- End of Configuration ---------------------------------------------------


void setup(){

 pinMode(Red_LED, OUTPUT);
 pinMode(Blue_LED, OUTPUT);
 pinMode(Green_LED, OUTPUT);
 pinMode(RDM_PWR_Relay, OUTPUT); 

 pinMode(Start_Relay, OUTPUT); // Bike Start Relay
 pinMode(A1, OUTPUT); // Battery Read Relay
 pinMode(A0, OUTPUT); // Bike ON Relay
 pinMode(A2, INPUT); // Button
 pinMode(A3, OUTPUT); // IMU power transistor
 pinMode(A6, INPUT); // Battery Voltage Divider



 digitalWrite(Red_LED, LED_LOW); // 
 digitalWrite(Blue_LED, LED_LOW); // 
 digitalWrite(Green_LED, LED_LOW); // 

 digitalWrite(Start_Relay, LOW); // 
 digitalWrite(Battery_Read_Relay, LOW); // 
 digitalWrite(Bike_ON_Relay, LOW); //
 digitalWrite(RDM_PWR_Relay, LOW); // 



 Serial.begin(9600); // Serial Begin
 //----------------------------------- RDM6300 Init -------------------------------------
 ssrfid.begin(9600); // SSRFID Begin
 Serial.println(F("RDM 6300 System Initialized"));

 // --------------------------------- SIM 7000A SIM/GPS ----------------------------------
  //pinMode(RST, OUTPUT);
  //digitalWrite(RST, HIGH); // Default state
  digitalWrite(BOTLETICS_PWRKEY, HIGH);
  pinMode(BOTLETICS_PWRKEY, OUTPUT);



} // End Setup

void loop(){
  switch (Scrambler_State) {
    case INITIAL:
      powerOffSMS();
      if(checkbatteryvoltage() == 0){
        Previous_State = INITIAL;
        Scrambler_State = OFF;
        Serial.println(F("Transitioning to OFF State from INTIAL!"));
      }
      else{
        Previous_State = INITIAL;
        Scrambler_State = RUNNING;
        Serial.println(F("Transitioning to RUNNING State from INTIAL!"));
      }
      break; // This is correct pat!

    case OFF:
    //Serial.print("The bike is in OFF state!");
      //Arduino in sleep mode
      ButtonState = read_button();
      delay(50);
      //Serial.println(ButtonState);
      if (ButtonState != PreviousButtonState){
        if(ButtonState == 1){
          PH_Timer = millis();
        }
        if(ButtonState == 0){
          if(millis() - PH_Timer > ButtonHoldThreshold){
            Serial.println(F("Button Hold Detected!"));
              Scrambler_State = SENTRY;
              Previous_State = OFF;
              ButtonCount = 0;
              PreviousButtonState = ButtonState;
              break;
          }
          else{
            ButtonCount ++;
            Post_Press_Timer = millis();
            Serial.println(F("Button Press Detected!"));
          }
        }
      }

      if(ButtonCount != 0){
        if(millis() - Post_Press_Timer > InterPressThreshold){
          if(ButtonCount == 1){
            Serial.println(F("1 Button Press Detected, Switching to SCANNING State"));
            Scrambler_State = SCANNING;
            Previous_State = OFF;
            ButtonCount = 0;
            PreviousButtonState = ButtonState;
            break;}
          if(ButtonCount > 4){
            Serial.println(F("5 Button Presses Detected. Switching to STANDBY State")); 
            Scrambler_State = STANDBY;
            Previous_State = OFF;
            ButtonCount = 0;
            PreviousButtonState = ButtonState;
            break;}
          else{Serial.println(F("No action taken.")); ButtonCount = 0; PreviousButtonState = ButtonState;}
        }
      }

      PreviousButtonState = ButtonState;
      break;
    
    case STANDBY:
      Serial.println(F("Bike is in STANDBY state!"));

      if(State_initialized == false){
        modemSS.listen();
        powerOnSMS();
        delay(3000);
        State_initialized = true;
        Serial.print("initialized SMS");
      }



      // Bike is ON
      starttime = millis();  //  Start a timer
      currenttime = starttime;  //  Current Time (to be updated constantly)
      while(currenttime - starttime < 1000){
        delay(200);
        currenttime = millis();
      }

      // Retrieve SMS sender address/phone number.
      if (! modem.getSMSSender(0, replybuffer, 250)) {
        Serial.println(F("No Text!"));
        break;
      }
      Serial.print(F("FROM: ")); Serial.println(replybuffer);
      
      // Retrieve SMS value.
      uint16_t smslen;
      if (! modem.readSMS(0, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println(F("No text!"));
          break;
        }
      
      Serial.print(F("Text received! : "));
      Serial.println(replybuffer);
    
      // if(replybuffer == "START"){
      // Scrambler_State = STARTING;
      // Previous_State = STANDBY;
      // State_initialized = false;
      // powerOffSMS();
      // delay(500);}
      // else{"Wrong password";}

      if (compareCharBuffers(replybuffer, passbuffer, 5)) {
        Scrambler_State = STARTING;
        Previous_State = STANDBY;
        State_initialized = false;
        powerOffSMS();
        delay(500);
        }
      else{"Wrong password";}




      if (modem.deleteAllSMS()) {
        Serial.println(F("OK!"));
      } else {
        Serial.println(F("Couldn't delete"));
      }

      break;

    case SENTRY:
      if(State_initialized == false){
        // --------------------------------------- MPU 6050 init (IMU) -------------------------
        digitalWrite(A3, HIGH); //
        delay(1000);
        Wire.begin();
        mpu.initialize();
        // Optional: mpu.setInterruptPin(12); // Set interrupt pin, if used
        State_initialized = true;
        delay(500);
        digitalWrite(Blue_LED, LED_HIGH);
      } // ---------------------------------------- End of init for IMU -----------------------------
        
      ButtonState = read_button();
      //Serial.println(ButtonState);
      //Serial.println(ButtonState);
      if (ButtonState == 1) { // This actually has to be done with an interrupt!
          MPU_OFF();
          digitalWrite(A3, LOW); // Kill power to MPU
          State_initialized = false;
          Scrambler_State = SCANNING;
          Previous_State = SENTRY;
          Alarm_Counter = 0; // Good form to reset this I reckon...
          digitalWrite(Blue_LED, LED_LOW);
          Serial.println(F("Short Button Press Detected: The Scrambler is switching into SCANNING State (from SENTRY)."));
          break;
        }

      // Read raw accelerometer and gyroscope data
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //Serial.println(abs(gx+gy+gz));

        if(abs(gx+gy+gz) > 750){
          Alarm_Counter ++;

          if(Alarm_Counter > 100){
            MPU_OFF();
            digitalWrite(A3, LOW); // Kill power to MPU
            State_initialized = false;
            Serial.println(F("Bike alarm tripped!, Switching to ALARMING State!"));
            Alarm_Counter = 0; // Good form to reset this I reckon...
            Scrambler_State = ALARMING;
            Previous_State = SENTRY;
            digitalWrite(Blue_LED, LED_LOW);
            digitalWrite(Red_LED, LED_HIGH);
            break;
          }
        }
      break;

    case STARTING:
      Serial.println(F("Bike is in STARTING state!"));
      digitalWrite(Bike_ON_Relay, HIGH);
      delay(1500);
      digitalWrite(Start_Relay, HIGH); // Provide power to the start relay of the bike
      delay(2000);
      digitalWrite(Start_Relay, LOW); // Remove power to the start relay of the bike
      delay(2500);
      
      Serial.println(F("Checking if the bike successfully started"));
      Battery_Voltage = checkbatteryvoltage();
      Serial.print(F("Is bike on? : "));
      Serial.println(Battery_Voltage);

      //if (Battery_Voltage == 1){ // If the Bike has started sucessfully (is running)
      if (1){ // If the Bike has started sucessfully (is running)
        Serial.println(F("The Scrambler started! Switching into RUNNING State!"));
        Scrambler_State = RUNNING;
        Previous_State = STARTING;
      }
      else{
        Serial.println(F("The Scrambler did not start :( Switching into Previous State! (SCANNING State)"));
        Scrambler_State = Previous_State;
        Previous_State = STARTING;
      }
      break;







    case ALARMING:
      Serial.println(F("Bike is in ALARMING state!"));
      powerOnSMS();
      delay(1000);
      modemSS.listen();
      delay(3000);
      //Flash Blue and Red LEDs quickly back and forth
      //send an SMS!
      //char sendto[21], message[141];
      
      while(texts_sent <2){
        Serial.print(F("Sending ALARMING Text Message! "));
        if (!modem.sendSMS("XXXXXXXXXXXXX", "This is the Scrambler! The alarm has been tripped! Please check out the Bike!")) {
          Serial.println(F("Failed"));
          texts_sent += 1;
          delay(3000);
        } else {
          //Serial.print("Texts sent :");
          //Serial.println(texts_sent);
          Serial.println(F("Sent!"));
          texts_sent += 1;
          delay(3000);
        }
      }

      powerOffSMS();
      delay(500);
      Serial.println(F("Sent two warning texts! Switching back to SENTRY state."));
      Scrambler_State = SENTRY;
      Previous_State = ALARMING;
      digitalWrite(Red_LED, LED_LOW);
      texts_sent = 0;
      delay(50);

        break;
    
    
    



    case RUNNING:
      Serial.println(F("Bike is in RUNNING state!"));
      starttime = millis();  //  Start a timer
      currenttime = starttime;  //  Current Time (to be updated constantly)
      while(currenttime - starttime < 10000){
        delay(200);
        currenttime = millis();
      }
      Battery_Voltage = checkbatteryvoltage();
      if (Battery_Voltage == 0){
        Serial.println(F("The bike is off, switching to OFF state."));
        Scrambler_State = OFF;
        Previous_State = RUNNING;
      }
      break;
      


    case SCANNING:
      ssrfid.listen();
      digitalWrite(RDM_PWR_Relay, HIGH); // 
      delay(100);
      starttime = millis();  //  Start a timer
      currenttime = starttime;  //  Current Time (to be updated constantly)
      if (Previous_State == OFF){scan_time = off_scantime;}  // If the bike was OFF (not 'locked') in previous state, scan for 60 seconds
      if (Previous_State == STARTING){scan_time = off_scantime;}  // If the bike was OFF (not 'locked') in previous state, scan for 60 seconds
      else {scan_time = locked_scantime;};  //  If the bike was in SENTRY ('locked') previous state, scan for 20 seconds

      while (currenttime - starttime < scan_time)  //test whether the period has elapsed
      {
        digitalWrite(Red_LED, LED_HIGH); // 
      //----------------------------------------------------- RFID Search ------------------------------------------------------------------------
        if (ssrfid.available()){
          Serial.println("Detected RFID Tag!");
          digitalWrite(Blue_LED, LED_HIGH); // 
          
          while(ssrfid.available())               // reading data into char array
          { 
        
            buffer[count++] = ssrfid.read();      // writing data into array
            // Print the value in different formats
            // Serial.print("Raw value: ");
            // Serial.print(byteRead);  // Print as a raw integer

            // Serial.print("Hex value: ");
            // Serial.print(byteRead, HEX);  // Print as a hexadecimal value

            // Serial.print("Char value: ");
            // Serial.println((char)byteRead);  // Print as a character

            // if(count == 26){
            //   Serial.print("Ending");
            //   break;}
            if(count == 14)break;
          }
          // for (int i = 0; i < 14; i++) Serial.print(buffer[i]);
          // Serial.println("buffer read");

          delay(150);
          
          // for (int i = 0; i < 14; i++) Serial.print(targetbuffer[i]);
          // Serial.println("buffer read");

          // for (int i = 0; i < 14; i++) Serial.print(targetbuffer2[i]);
          // Serial.println("buffer read");

          if (compareRawBuffers(buffer, targetbuffer, 14)) {
              Serial.println("PAT DETECTED!");
              Scan_Results = PAT;
              clearBufferArray();             // call clearBufferArray function to clear the stored data from the array
              count = 0;                      // set counter of while loop to zero
              break;
          } else {
              Serial.println(F("RFID read, not pat!"));
              clearBufferArray();
          }

          Serial.println(F("end"));

          clearBufferArray();             // call clearBufferArray function to clear the stored data from the array
          count = 0;                      // set counter of while loop to zero
          
          
          
          
          
          
          
        }
          
          
          /*int ssvalue = ssrfid.read(); // read 
          if (ssvalue == -1) { // no data was read
            return;
          }

          if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
            buffer_index = 0;
          } else if (ssvalue == 3) { // tag has been fully transmitted       
            call_extract_tag = true; // extract tag at the end of the function call
          }

          if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
            Serial.println(F("Error: Buffer overflow detected!"));
            return;
          }
          
          buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer

          if (call_extract_tag == true) {
            if (buffer_index == BUFFER_SIZE) {
              if (extract_tag() == "XXXXXXXXXXXXXXXX") {
                Serial.println(F("Identified Pat's RFID Tag!"));
                Scan_Results = PAT;
                digitalWrite(Green_LED, LED_HIGH); 
                break;
              }
            } else { // something is wrong... start again looking for preamble (value: 2)
              buffer_index = 0;
              return;
            }
          }    
        }

        currenttime = millis(); // Update the timer, waiting to kill the search process   */

      //----------------------------------------------------- End RFID Search ------------------------------------------------------------------------
      } // End while loop

      clearBufferArray();

      delay(250);
      digitalWrite(Blue_LED, LED_LOW);
      digitalWrite(Red_LED, LED_LOW);
      digitalWrite(Green_LED, LED_LOW);
      digitalWrite(RDM_PWR_Relay, LOW);
      delay(200);

      if (Scan_Results == PAT){
        Serial.println(F("The Scrambler is switching into STARTING State!"));
        Scrambler_State = STARTING;
        Previous_State = SCANNING;
        Scan_Results = NONE;

        Serial.println(Scrambler_State);
        Serial.println(Previous_State);
        Serial.println(Scan_Results);
        break;
      }

      Serial.print(F("No Tag Found, Switching to Prev. State"));
      Scan_Results = NONE;
      Scrambler_State = Previous_State;
      Previous_State = SCANNING;
      break;







    default:
      break;

  
  } // End Switch Statement
} // End Main Loop

int read_button(){
    rawbuttonreading = analogRead(A2);
    //Serial.println(rawbuttonreading);

      if (rawbuttonreading > 950){
        currentButtonState = 1;
      }
      else{
        currentButtonState = 0;
      }

      if (currentButtonState != lastButtonState) {
        lastDebounceTime = millis();
      }

      if ((millis() - lastDebounceTime) > debounceDelay) {
        //Serial.print("Button Raw Reading : ");
        //Serial.println(rawbuttonreading);
        //Serial.print("Button State : ");
        //Serial.println(currentButtonState);
        return currentButtonState;
      }
      
      lastButtonState = currentButtonState;
}

//-- MPU ---------------
void MPU_OFF(){
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

// ------------------------------------------------------------------- RFID Search & Identify Functions ----------------------------------------------------------
void clearBufferArray() { // function to clear buffer array
  for (int i = 0; i<count; i++) { // clear all index of array with command NULL
    buffer[i] = 0;
  }
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
      digitalWrite(Battery_Read_Relay, HIGH);
      delay(250);
      Raw_Battery_Reading = analogRead(A6);
      Voltage_Reading = Raw_Battery_Reading/40.3137254902; // Good Ol' Scaling Factor
      Serial.print(F("The Battery Voltage is : "));
      Serial.println(Voltage_Reading);
      digitalWrite(Battery_Read_Relay, LOW);
      delay(1250);
      if (Battery_Voltage > 14){ // If the Bike has started sucessfully (is running)
        return(1);
      }
      else{
        return(0);
      }
}

// ------------------------- SIM 7000A Modem/GPS functions below ---------------------------------

void powerOnSMS() {
  if(analogRead(A7) <= 275){
    Serial.print(F("the sms board is off. turning on. reading:"));
    // Serial.println(analogRead(A7));
    // digitalWrite(BOTLETICS_PWRKEY, LOW);
    // delay(300);
    // digitalWrite(BOTLETICS_PWRKEY, HIGH);
    modem.powerOn(BOTLETICS_PWRKEY); // Power on the module
    delay(4500);
  }
  else{
    delay(50);
    Serial.println("Already On");
  }

  Serial.println(F("Initializing....(May take several seconds)"));
  modemSS.begin(115200); // Default SIM7000 shield baud rate
  delay(100);

  Serial.println(F("Configuring to 9600 baud"));
  modemSS.println("AT+IPR=9600"); // Set baud rate

  delay(100); // Short pause to let the command run
  modemSS.begin(9600);
  if (! modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
  }
  else{
  type = modem.type();
  Serial.println(F("Modem is OK"));
  uint8_t imeiLen = modem.getIMEI(imei); // Print module IMEI number.
  modem.setFunctionality(1); // AT+CFUN=1 // Set modem to full functionality
  modem.setNetworkSettings(F("hologram")); // For Hologram SIM card
  delay(150);
  }
}

void powerOffSMS() {
  if(analogRead(A7) > 275){
    //Serial.print("the sms board is on. turning off. reading:");
    //Serial.println(analogRead(A7));
    digitalWrite(BOTLETICS_PWRKEY, LOW);
    delay(1300);
    digitalWrite(BOTLETICS_PWRKEY, HIGH);
    delay(50);
  }
  else{
    delay(50);
    Serial.println("Already Off");
  }
}



void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available()){
    continue;
  }
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




bool compareRawBuffers(const int* buffer1, const int* buffer2, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (buffer1[i] != buffer2[i]) {
            return false;  // Buffers are not equal
        }
    }
    return true;  // Buffers are equal
}

bool compareCharBuffers(const char* buffer1, const char* buffer2, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (buffer1[i] != buffer2[i]) {
            return false;  // Buffers are not equal
        }
    }
    return true;  // Buffers are equal
}




void printBufferInHex(byte* buffer, int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(buffer[i], HEX); // Print each byte in hexadecimal format
    Serial.print(" "); // Add a space between bytes for readability
  }
  Serial.println(); // Add a new line at the end
}

