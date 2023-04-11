

//behaviour:
//- if pump stopped by off button or endstops, it doesn't remember where it got to so will restart pumping whatever volume is on screen
//- to confirm and select next value push rotarty encoder
//- to adjust values turn the rotary encode
//- ot start or stop pump use the rocker switch

//Notes:
//Grace's test NEMA 17 17HS08-1004S rated to 1.0 A so Vref of A4988 driver set too 1/2.5 = 0.4
//Motor wires: 2B=black=A+, 2A=green=A-, 1A=red=B+, 1B=blue=B-
//endstops lever switches 's' or 'normally on' pin goes to analogue pin and G goes to ground and V goes to 5V normally closed (these may need rewiring on the LEM pump because original version used 2 wire configuration(

//========================================================================================================

//IMPORTANT STUFF:
//When changing the syringe size please alter the MMperML value in the Syringe pump settings section

//BEFORE UPLOADING PROGRAM TO ARDUINO
//PLEASE CHECK that the pins match your wiring, and ALTER THE I2C address where indicated

//CALIBRATE WITH WATER FIRST BEFORE USING ON REACTOR WHEN:
// - first uploaded program
// -changed any paramters or code
// / changing syringe - with plastic syringes I also suggest you do a run with blank solvent.


//This code was written using the following online learning resources:
//https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/ [24/09/21]
//https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/ [24/09/21]
//If you are new to electronics and arduino please refer to these resources
//They explain how the components work, as well as the code

//I have tried to minimize dependancies on non-standard libraries so that this code is easier to maintain




//=========================Program========================================//
//Libraries
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
//==================================================
//inputs
#define DT 3//Rotary Encoder DT pin
#define CLK 2//Rotary Encoder CLK pin
#define SW 4//Rotary Encoder button
#define pumpSwitch 5//switch for turning pump on and off
#define FrontStop A3 //front stop switch
#define BackStop A2 //back stop switch this has been changed from A4 because A4 is used as SDL for UNO - slight re-wiring required on Paris pump
//===================================================
//outputs
#define en 12 // enable pin on A4988 stepper motor driver
#define stepPin 11 // step pin on A4988 stepper motor driver
#define dirPin 10 // direction pin stepper motor driver: HIGH = CW, LOW = CCW
#define testLED 13 // test LED pin

// Stepper motor settings
#define MMperML 10            //The number of mm the syringe must move to dispense 1 mL YOU NEED TO CHANGE THIS FOR YOUR SYRINGE!

//These settings came analytical chemistry pump paper
#define NOFMICROSTEPS 16      // The number of microsteps per step. This is hardwired on the stepper driver if you used my schematics.
#define NOFSTEPSPER360 200    // The number of steps per revolution. This is a standard value for most stepper motors.
//#define MAXRPM 240            
#define MMPER360 2            // Leadscrew pitch, mm per revolution. Check your lead screw
//Values specific to this code
#define MAXRPM 146             // Maximum RPM (rotations per minute)

//note on MAXRPM - the RS data sheet wasn't clear on minimum pulse duration
//the stepper motor speed is controlled by voltage pulses to the step pin on the driver per second.
//I have decreased the max speed to 146 rpm with a pulse duration of 0.5 ms and a minimum off time of >25 ms between pulses
//according to this blog post https://forum.arduino.cc/t/simple-stepper-program/268292/3
// the pulse duration is large and the the off time is just right - however I saw varying values and this may need adjusting
//However, this means that the syringe pump can move at 294 mm/min at max speed which is way too fast for what the chemical reactors need.
int pulseDuration = 500;//microseconds
int NumSteps;
long PulseDelay;

//LCD I2C address
hd44780_I2Cexp lcd(0x27); // CHANGE 0x27 to your Screen address
const int LCD_ROWS = 2;
const int LCD_COLS = 16;
//================================================
//Variables and constants
unsigned long lastButtonPress = 0;
int buttonPressed;
int buttonState;
int lastStateCLK;
int currentStateCLK;
String currentDir="";
int counter = 0;
int menuState;
float volume = 0.00;
float flowRate = 0.00;
float minFlowRate = 0.5;
int mode = 0; //controls pump direction
int pumpStatus = LOW;
int lastPumpStatus = LOW;
int lastIndex[2] = {4, 0};
int stopBack;
 int stopFront;
 int stopSwitch;
//=================================================================================
//Motor Functions
int calcModeNum(String Mode){
  if(Mode=="INJ"){
    return 1;}
  else{return 0;}
  }
int calcPulseDelay(float flowRate){
  float mmperml;
  float MMPERSEC;
  float MSperRev;
  float floatPulseDelay;
  if(flowRate>0){
    //flow rate isin mL per min
    mmperml = double(MMperML);
    MMPERSEC = (flowRate/mmperml)/60;
    Serial.print(" (");
    Serial.print(MMPERSEC);
    Serial.print(") ");
    MSperRev = (MMPER360/MMPERSEC)*1000;
    Serial.print(" (");
    Serial.print(MSperRev);
    Serial.print(") ");
    floatPulseDelay = (MSperRev/(NOFSTEPSPER360*NOFMICROSTEPS))*1000 - pulseDuration;
    return long(floatPulseDelay);
  }
  else{return 0;}
  }
int calcNumSteps(float volume){
  float distance;
  float floatSteps;
  if(volume>0){
    distance = volume*MMperML;
    floatSteps = distance/MMPER360;
    floatSteps = floatSteps*(NOFSTEPSPER360*NOFMICROSTEPS);
    return int(floatSteps);
    }
  else{return 0;}
  }
void runMotor(int NumSteps, long PulseDelay, int mode){
  digitalWrite(en, LOW);
  if (mode==1){
    digitalWrite(dirPin, HIGH);
    }
  else{digitalWrite(dirPin, LOW);}
  for(int x = 0; x < NumSteps; x++){
    stopBack = digitalRead(BackStop);
    stopFront = digitalRead(FrontStop);
    if(stopBack == LOW || stopFront == LOW){
      break;
      }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PulseDelay);
    stopSwitch = digitalRead(pumpSwitch);
    if(stopSwitch==LOW){break;}
    }
  digitalWrite(en, HIGH);
  }
//===================================================================
//Menu functions
int checkButton(){
  buttonState = digitalRead(SW);
  if (buttonState == LOW){
    if (millis() - lastButtonPress > 50){
      lastButtonPress = millis();
      Serial.print("Button Pressed");
      return 1;
      }
     else{return 0;}
    }
  else{return 0;}
  }
void updateDisplay(){
  lcd.setCursor(0,0);
  lcd.print("VOL");
  lcd.setCursor(4,0);
  lcd.print("       ");
  lcd.setCursor(4,0);
  lcd.print(volume);
  lcd.setCursor(8,0);
  lcd.print("ml");
  lcd.setCursor(11,0);
  if(mode==0){lcd.print("FILL");}
  else{lcd.print("INJT");}
  lcd.setCursor(0,1);
  lcd.print("RATE");
  lcd.setCursor(5,1);
  lcd.print("        ");
  lcd.setCursor(5,1);
  lcd.print(flowRate);
  lcd.setCursor(9,1);
  lcd.print("mpm");
  lcd.setCursor(13,1);
  if(pumpStatus==0){lcd.print("OFF");}
  else{lcd.print("ON");}
    }

void cursorIndex(){
  //made this function work by swapping switch for if - I don't think arduino C/C++ supports switch
    if(buttonPressed==1){
      if(lastIndex[0]==4){
        lastIndex[0] = 11;
        lastIndex[1] = 0;
        }
      else if(lastIndex[0]==11){
        lastIndex[0] = 5;
        lastIndex[1] = 1;
        }
      else if(lastIndex[0]==5){
        lastIndex[0] = 4;
        lastIndex[1] = 0;
        }
      else{
        lastIndex[0] = 4;
        lastIndex[1] = 0;
        }
    }
    }
void setValue(int lastIndex[2], int counter){
  if(counter !=0){
    if(lastIndex[0]==4){
      volume= volume + counter;
      }
    else if(lastIndex[0]==11){
      mode= mode + counter;
      if(mode > 1){mode = 1;}
      else if(mode < 0){mode = 0;}
      }
    else if(lastIndex[0]==5){
      flowRate = flowRate + (float(counter)*minFlowRate);
      }
    updateDisplay();
    }
  }
void updatePumpStatus(){
  if(pumpStatus!= lastPumpStatus){
    lcd.setCursor(13,1);
    if(pumpStatus==LOW){lcd.print("OFF");}
    else{lcd.print("ON ");}
  }
  }
//======================================================================
//main loops
//======================================================================
void setup() {
  // put your setup code here, to run once:
  //===================================
  //motor setup
  pinMode(FrontStop, INPUT);
  pinMode(BackStop, INPUT);
  //declare output pins
  pinMode(en, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(en, HIGH);
  flowRate =0.98; //flow rate or 0.0001 returns 0 delay while flow rate > 0.95 returns negative numbers may need LONG variable not int
  volume = 1.0;
  //===================================
  //menu setup
//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pumpSwitch, INPUT_PULLUP);
  pinMode(DT, INPUT);
  pinMode(CLK, INPUT);
  pinMode(SW, INPUT_PULLUP);//enabling internal pull-up on encoder button pin
  lastPumpStatus=LOW;
  //attaching interrupts to encoder pins
  //UNO has 2 interrupts:
  //Interrupt 0 is attached to pin 2 and interrupt 1 is attached to pin 3
  //following code tells arduino to call updateEncoder() when this pins change state
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  // setting up the LCD
  int status;
  status = lcd.begin(LCD_ROWS, LCD_COLS);
  if(status) //none zero means unsuccessful
  {
    hd44780::fatalError(status);
    //this calls a routine from the hd44780 library that blinks an LED if possible if status!=0
    }
   //the next code shows to start up display on the LCD screen
  lcd.clear();
  lcd.print("Hello World!");
  delay(2000);
  lcd.clear();         
  lcd.setCursor(0,0);   //Set cursor to character 0 on line 1
  lcd.print("Syringe Pump v1");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,1);   //Set cursor to character 0 on line 2
  lcd.print("Ready");
  delay(2000);

  //Setup Serial Monitor for code testing
  Serial.begin(9600);
  lcd.clear();
  updateDisplay();
  lcd.setCursor(lastIndex[0], lastIndex[1]);
  lcd.blink();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(pumpStatus == HIGH && lastPumpStatus == LOW){
    updatePumpStatus();
    lastPumpStatus = HIGH;
    NumSteps= calcNumSteps(volume);
    PulseDelay= calcPulseDelay(flowRate);
    Serial.print(NumSteps);
    Serial.print(" ");
    Serial.print(PulseDelay);
    Serial.print(", ");
    runMotor(NumSteps, PulseDelay, mode);
    delay(2000);
  }
  else{
    if(pumpStatus != lastPumpStatus){
      updatePumpStatus();
      }
    delay(45);
    setValue(lastIndex, counter);
    buttonPressed = checkButton();
    if(buttonPressed==1){
      //This if statement proves that buttonPressed Function is working and buttonPressed returns 1
      Serial.print(buttonPressed);}
    cursorIndex();
    lcd.setCursor(lastIndex[0], lastIndex[1]);
    lcd.blink();
    counter = 0;
    if(pumpStatus==HIGH){
      lastPumpStatus=HIGH;
      }
    else{lastPumpStatus=LOW;}
    pumpStatus = digitalRead(pumpSwitch);
  }
}
void updateEncoder(){
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
    }
//   Serial.print("Direction: ");
//   Serial.print(currentDir);
//   Serial.print(" | Counter: ");
//   Serial.println(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
  }
