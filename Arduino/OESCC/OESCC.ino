// TODO:
// work around for blocking on serial write (done: confirmed, default code doesn't block on write)
// increase temperature samples to every 2 seconds (done)
// increase time proportional control frequency - 500 miliseconds (done)
// send remote commands to set setpoint, emergency cutoff, PID parameters (done)
// add command validation to avoid bad command parmeters to PID routine (done)
// add command in PID library to clear out accumulator PID retuning ClearITerm();  (done)
// add emergency cutoff control for controller temp (done)
// ignore cr/lf on commands to accomodate andriod bluetooth app that always sends cr or lf (done)
// modify pid routine for adaptive integration - don't accumulate if already at full power (done)
// modify pid routine to use an average of the last 5 samples for the Pd function, to reduce the effect of noise (done)
// @225012002250# - ribs
// @250012002250# - ribs
// user interrupts for control and temp sample timing instead of polling

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

// Arduino Pin Assignments
#define ONE_WIRE_BUS_1 2 // smoker temperature
#define ONE_WIRE_BUS_2 4 // Arduino & SSR enclosure temperature
#define ONE_WIRE_BUS_3 6 // Outside Temperature

// The resolution of the temperature sensor is user-configurable to 9, 10, 11, or 12 bits, corresponding to increments of 0.5°C, 0.25°C, 0.125°C, and 0.0625°C, respectively
// 10 bit conversion time 187.5ms
#define TEMPERATURE_PRECISION 11

// wireless UART used for bluetooth module
SoftwareSerial mySerial(10, 11); // RX, TX

// solid state relay pin
int ssrpin = 3;

// status led on arduino to show SSR state
int ledpin = 13;

// Setup a oneWire instances to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(ONE_WIRE_BUS_1);
OneWire oneWire2(ONE_WIRE_BUS_2);
OneWire oneWire3(ONE_WIRE_BUS_3);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
DallasTemperature sensor3(&oneWire3);

float smokerTemp;
float controllerTemp;
float outsideTemp;

//int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

long previousMillis = 0;        // will store last time LED was updated

float smokerEmergencyShutoffTemp = 284;  // degrees F
float controllerEmergencyShutoffTemp = 158;  // degrees F
boolean inEmergencyShutdown = false;
boolean ssrOn = false;
String state = "Run";

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp = 0, Ki = 0, Kd = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long WindowSize = 1000;  // control sensor loop & PID calculation duration 1st cycle is read temps, second is calculate PID, so sample period is double the window size
unsigned long tpcWindowSize = 500; // time proportional control window size
unsigned long windowStartTime;
unsigned long currentMillis = 0;

boolean readTemp = false;

int commandInts[12];

// used for debouncing switch inputs
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
int lasta0 = HIGH;
int lasta1 = HIGH;
int lasta2 = HIGH;
int lasta3 = HIGH;

int switchMode = 0;
  // 0 = off
  // 3 = smoker low
  // 4 = smoker med
  // 5 = smoker high
  // 6 = sous vide low
  // 7 = sous vide med
  // 8 = sous vide high

void setup(void)
{
  currentMillis = millis();

  // PID setup
  Kp = 0;
  Ki = 0;
  Kd = 0;
  Setpoint = 80;
  windowStartTime = currentMillis;
  myPID.SetOutputLimits(0, tpcWindowSize); //tell the PID to range between 0 and the full tpc window size
  myPID.SetMode(AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(2000); // sets the PID sample time in millis

  pinMode(ssrpin, OUTPUT); // setup SSR pin as output
  pinMode(ledpin, OUTPUT); // setup LED pin as output  

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  // initialize the temperature sensor buses
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();

  // don't delay in temperature reads.. come back later to collect the temperature

  sensor1.setWaitForConversion(false);
  sensor2.setWaitForConversion(false);
  sensor3.setWaitForConversion(false);

  // set up the 3 way switch inputs using the analog pins as gpio
  // controller mode: smoker our sous vide
  pinMode(A0, INPUT); //A0 pulled low indicates smoker mode
  digitalWrite(A0, HIGH); //pull up resistor on
  pinMode(A1, INPUT); //A1 pulled low indicates sous vide mode
  digitalWrite(A1, HIGH); //pull up resistor on
  pinMode(A2, INPUT); //A2 pulled low indicates low temp mode (225 smoker, tbd sous vide)
  digitalWrite(A2, HIGH); //pull up resistor on
  pinMode(A3, INPUT); //A3 pulled low indicates high temp mode (tbd smoker, tbd sous vide)
  digitalWrite(A3, HIGH); //pull up resistor on  
  // with both A2 and A3 left high this is mid temp mode (tbd smoker, tbd sous vide)
  // 9 possible modes 3^2

}

void loop(void)
{ 
  PID_Temp();
  readSwitches();
  processCommand();
  tpc(); // time proportional control
}

void readSwitches(){
  int a0 = digitalRead(A0);
  int a1 = digitalRead(A1);
  int a2 = digitalRead(A2);
  int a3 = digitalRead(A3);

  //debounce check
  if (a0 != lasta0 || a1 != lasta1 || a2 != lasta2 || a3 != lasta3){
    lastDebounceTime = millis(); 
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // now that the switch state is confirmed to have changed to the new state(s) process the results
    processSwitches(a0,a1,a2,a3);
  }

  lasta0 = a0;
  lasta1 = a1;
  lasta2 = a2;
  lasta3 = a3;
}

void processSwitches(int a0, int a1, int a2, int a3){
  //A0 pulled low indicates smoker mode
  //A1 pulled low indicates sous vide mode
  //A2 pulled low indicates low temp mode (225 smoker, tbd sous vide)
  //A3 pulled low indicates high temp mode (tbd smoker, tbd sous vide)
  // with both A2 and A3 left high this is mid temp mode (tbd smoker, tbd sous vide)
  
  // int switchMode
  // 0 = off
  // 3 = smoker low
  // 4 = smoker med
  // 5 = smoker high
  // 6 = sous vide low
  // 7 = sous vide med
  // 8 = sous vide high
  
  if (a0 == LOW){ // smoker mode
    if (a2 == LOW){ // low temp mode
      // we are in low temp smoker mode
      if (switchMode != 3){
      mySerial.println("toggle switch: setting PID parameters for smoker mode, low temp 225");
      Setpoint = 225; // setpoint
      Kp = 12; // Kp
      Ki = 2; // Ki
      Kd = 250; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 3;
      }
    }
    else if (a3 == LOW){ // high temp mode
      // we are in high temp smoker mode
      if (switchMode != 5){
      mySerial.println("toggle switch: setting PID parameters for smoker mode, high temp 250");
      Setpoint = 250; // setpoint
      Kp = 12; // Kp
      Ki = 2; // Ki
      Kd = 250; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 5;
      }
    }
    else { // medium temp mode
      // we are in med temp smoker mode
      if (switchMode != 4){
      mySerial.println("toggle switch: setting PID parameters for smoker mode, medium temp 240");
      Setpoint = 240; // setpoint
      Kp = 12; // Kp
      Ki = 2; // Ki
      Kd = 250; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 4;
      }
    }
    
  }
  else if (a1 == LOW){ // sous vide mode
    if (a2 == LOW){ // low temp mode
      // we are in low temp sous vide mode
      if (switchMode != 6){
      mySerial.println("toggle switch: setting PID parameters for sous vide mode, low temp 120");
      Setpoint = 120; // setpoint
      Kp = 26; // Kp
      Ki = 0; // Ki
      Kd = 0; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 6;
      }
    }
    else if (a3 == LOW){ // high temp mode
      // we are in high temp sous vide mode
      if (switchMode != 5){
      mySerial.println("toggle switch: setting PID parameters for sous vide mode, high temp 170");
      Setpoint = 170; // setpoint
      Kp = 26; // Kp
      Ki = 0; // Ki
      Kd = 0; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 5;
      }
    }
    else { // medium temp mode
      // we are in low temp smoker mode
      if (switchMode != 7){
      mySerial.println("toggle switch: setting PID parameters for sous vide mode, medium temp 140");
      Setpoint = 140; // setpoint
      Kp = 26; // Kp
      Ki = 0; // Ki
      Kd = 0; // Kd
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.ClearITerm();
      switchMode = 7;
      }
    }

  }
  else{ // neither a0 or a1 low, so mode control is off
    // we are off
    if (switchMode != 0){
    mySerial.println("toggle switch: setting PID parameters for controller off");
    Setpoint = 1; // setpoint
    Kp = 0; // Kp
    Ki = 0; // Ki
    Kd = 0; // Kd
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.ClearITerm();
    switchMode = 0;
    }
  }
}

void PID_Temp(){
  currentMillis = millis();
  if(currentMillis - previousMillis > WindowSize) {
    previousMillis = currentMillis; // save the last time temperature sent

    // request the temperatures in this cycle
    if (!readTemp){
      readTemp=true;
      // Send the command to get temperatures
      sensor1.requestTemperatures();
      sensor2.requestTemperatures();
      sensor3.requestTemperatures();
    }
    // read the temperature in this cycle
    else{
      readTemp=false;
      //get the temps
      smokerTemp = getTemp(sensor1);
      controllerTemp = getTemp(sensor2);
      outsideTemp = getTemp(sensor3);

      // print the temperatures
      //printTempStatus();

      // check if we have reached emergency cutoff temp
      if(((smokerTemp > smokerEmergencyShutoffTemp) || (controllerTemp > controllerEmergencyShutoffTemp)) && !inEmergencyShutdown){
        emergencyShutOff();  // if so shut down the relay!
      }

      Input = smokerTemp;
      // turn on accumulator (integrator) only if temp is within 10 degrees of setpoint, to avoid windup
      // P gain of 600 = 6000 (full power) at 10 degrees under setpoint
      // I gain of 240 = 6000(full power) at 2.5 minutes (25 samples) of droop... 4% increase in power every 6 seconds.
      /*
      if ((Setpoint-Input) < 10 && (Setpoint-Input) > -10){
       myPID.SetTunings(600,240,500);
       }
       else{
       myPID.SetTunings(600,0,500);  // accumulator off when greater than 10 degrees away from set point
       }
       */
      myPID.Compute();
      printStatus();
    }
  }

}

float getTemp(DallasTemperature sensor){
  if(sensor.getAddress(tempDeviceAddress, 0))
  {
    float tempC = sensor.getTempC(tempDeviceAddress);
    float tempF = DallasTemperature::toFahrenheit(tempC);
    return tempF;
  }
  return 0;
}

// time proportioning control
void tpc()
{
  if (state = "Run"){

    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if (Output < 10 ) Output = 0;  // round the on time so it is at least 10 ms, else 0
    if (Output > tpcWindowSize - 10) Output = tpcWindowSize;  // round the off time so it is at least 10 ms.
    unsigned long windowruntime = currentMillis - windowStartTime;
    if(windowruntime > tpcWindowSize)
    { 
      windowStartTime = currentMillis;  //time to shift the Relay Window
      windowruntime = 0;
    }
    if(windowruntime > Output && ssrOn == true)
    { 
      digitalWrite(ledpin,LOW);
      digitalWrite(ssrpin,LOW);
      ssrOn = false;
    }
    else if (windowruntime < Output && ssrOn == false)
    {
      digitalWrite(ledpin,HIGH);
      digitalWrite(ssrpin,HIGH);
      ssrOn = true;
    }
  }
}

void printStatus(){

  mySerial.print(state);
  mySerial.print(",");
  mySerial.print(currentMillis/1000);
  mySerial.print(","); 
  mySerial.print(smokerTemp);
  mySerial.print(",");
  mySerial.print((int)controllerTemp);
  mySerial.print(",");
  mySerial.print((int)outsideTemp);
  mySerial.print(",");
  mySerial.print((int)(Output/tpcWindowSize*100));
  mySerial.print(",");
  mySerial.print((int)Setpoint);
  mySerial.print(","); 
  mySerial.print((int)Kp);
  mySerial.print(",");
  mySerial.print((int)Ki);
  mySerial.print(",");
  mySerial.print((int)Kd);
  mySerial.println("");
}

void emergencyShutOff(){
  mySerial.println("");
  mySerial.println("EMERGENCY SHUTDOWN TEMPERATURE REACHED: ");
  mySerial.print("smokerEmergencyShutoffTemp: ");
  mySerial.println(smokerEmergencyShutoffTemp, DEC);
  mySerial.print("controllerEmergencyShutoffTemp: ");
  mySerial.println(controllerEmergencyShutoffTemp, DEC);
  mySerial.println("SHUTTING DOWN IMMEDIATELY");
  digitalWrite(ledpin, LOW);  // turn on the LED, representing the relay closed/power on
  digitalWrite(ssrpin, LOW);  // turn on the actual SSR to power the appliance
  inEmergencyShutdown = true;
  state = "EMERGENCY SHUTDOWN";
}

void processCommand(){

  /* This routine checks for any input waiting on the serial line. If any is
   * available it is read in and added to a 128 character buffer. It sends back
   * an error should the buffer overflow, and starts overwriting the buffer
   * at that point. It only reads one character per call. If it receives a
   * newline character is then runs the parseAndExecuteCommand() routine.
   */
  int inbyte;
  static char incomingBuffer[13];
  static char bufPosition=0;
  boolean goodCommand = true;
  double tempSetpoint, tempKp, tempKi, tempKd ;

  if(mySerial.available()>0) {
    // Read only one character per call
    inbyte = mySerial.read();
    switch (inbyte){

    case '\n' :
      break; // ignore new line characters

    case '\r' :
      break; // ignore carriage returns

    case '@': // @ is the command to prime the command buffer
      incomingBuffer[bufPosition]='\0'; // NULL terminate the string
      bufPosition=0; // Prepare for next command
      mySerial.println("submit command");
      mySerial.println("SSSPPPIIIDDD 0-999 values. '#' to submit command");
      break;

    case 35:  // '#' = int 35, is the end of the command sequence
      //if(inbyte == 35) { // '#'

      incomingBuffer[bufPosition]='\0'; // NULL terminate the string
      bufPosition=0; // Prepare for next command
      goodCommand = true;
      // Supply a separate routine for parsing the command. This will
      // vary depending on the task.
      //mySerial.println(String(incomingBuffer));
      /*
      double Setpoint, Input, Output;
       double Kp = 100, Ki = 0, Kd = 0;    
       */
      tempSetpoint = 1.0 * (((incomingBuffer[0]-48)*100)+((incomingBuffer[1]-48)*10)+(incomingBuffer[2]-48)); // setpoint
      tempKp =  1.0 * (((incomingBuffer[3]-48)*100)+((incomingBuffer[4]-48)*10)+(incomingBuffer[5]-48)); // Kp
      tempKi =  1.0 *(((incomingBuffer[6]-48)*100)+((incomingBuffer[7]-48)*10)+(incomingBuffer[8]-48)); // Ki
      tempKd = 1.0 * (((incomingBuffer[9]-48)*100)+((incomingBuffer[10]-48)*10)+(incomingBuffer[11]-48)); // Kd
      mySerial.println(tempSetpoint);
      mySerial.println(tempKp);
      mySerial.println(tempKi);
      mySerial.println(tempKd);

      if (0.0 <= tempSetpoint && tempSetpoint < smokerEmergencyShutoffTemp)
      { 
        Setpoint = tempSetpoint; 
      } 
      else 
      { 
        mySerial.print("Error: Setpoint ");
        mySerial.println(tempSetpoint);
        mySerial.print("Setpoint must be 0 to ");
        mySerial.print(smokerEmergencyShutoffTemp);
        mySerial.println(" degrees F");
        goodCommand = false;
      }

      if (0.0 <= tempKp && tempKp < 1000.0)
      { 
        Kp = tempKp; 
      } 
      else 
      { 
        mySerial.print("Error: Kp "); 
        mySerial.println(tempKp);
        goodCommand = false;
      }

      if (0.0 <= tempKi && tempKi < 1000.0)
      { 
        Ki = tempKi; 
      } 
      else 
      { 
        mySerial.print("Error: Ki "); 
        mySerial.println(tempKi);
        goodCommand = false;
      }

      if (0.0 <= tempKd && tempKd < 1000.0)
      { 
        Kd = tempKd; 
      } 
      else 
      { 
        mySerial.print("Error: Kd "); 
        mySerial.println(tempKd);
        goodCommand = false;
      }

      if (goodCommand){
        mySerial.println("setting PID parameters");
        Setpoint = tempSetpoint; // setpoint
        Kp = tempKp; // Kp
        Ki = tempKi; // Ki
        Kd = tempKd; // Kd
        myPID.SetTunings(Kp, Ki, Kd);
        myPID.ClearITerm();
      }
      break;

      // }

    default:
      incomingBuffer[bufPosition]=(char)inbyte;
      bufPosition++;
      if(bufPosition==13) {
        mySerial.println("ERROR: Command Overflow.");
        mySerial.println("'@' to prime command processor");
        mySerial.println("'#' to submit command");
        bufPosition=0;
      }
    }
  }
}











