//teensy 3 code for aftertouch glove
//tomash ghz
//www.tomashg.com

#include <Bounce.h>

//debugging will print out in serial instead of midi
boolean debugging=false;

// the MIDI channel number to send messages
const int midiChannel = 7;

Bounce button0 = Bounce(7, 5);
Bounce button1 = Bounce(8, 5); 
Bounce button2 = Bounce(9, 5); 
Bounce button3 = Bounce(10, 5); 

//Analog read pins
const int xPin = 0;  // LEFT RIGHT
const int yPin = 1;  // FORWARD BACKWARD
const int zPin = 2;  // UP DOWN
const int irPin = 8; // Infrared

// RGB LEDs
const int rLed = 5;
const int gLed = 3;
const int bLed = 4;

const int AVERAGE_COUNT = 20; // Iterations for smoothing out the values

// hold the value when reading is triggered
double xTrig; 
double yTrig;
double zTrig;
double irTrig;

double x[AVERAGE_COUNT]; // store readings every cycle
double xSmooth = 0; // store the smootherd out value
double xLast = 0; // store the last reading
double xSlope = 0; // store the slope ( rate of change )

int counter=0;

double y[AVERAGE_COUNT];
double ySmooth = 0;
double yLast = 0;
double ySlope = 0;

double z[AVERAGE_COUNT];
double zSmooth = 0;
double zLast = 0;
double zSlope = 0;

double slope=0;

double ir[AVERAGE_COUNT];
double irSmooth = 0;

// States flag
// ir  z  y  x
//  0  0  0  0 off
//  1  1  1  1 on

const int OFFSTATE = 0;
const int XSTATE = 1;
const int YSTATE = 2;
const int ZSTATE = 4;
const int IRSTATE = 8;
const int SLOPEXSTATE = 3;
const int SLOPEYSTATE = 6;
const int SLOPEZSTATE = 12;

int state = 15; // store button states

int triggerTime = 180; // minimun trigger time for sample launches
int lastTrigger = 99999; // last trigger

void setup() {
  
  //DEBUG
  if (debugging) {
    Serial.begin(9600);//open serail port
  }
  else {
    Serial.begin(31250);//open midi
    usbMIDI.setHandleControlChange(myCCIn);
  }
  
  pinMode(0, OUTPUT); // set pin 0 as GROUND
  digitalWrite(0,LOW); // just because I ran out of ground pins
  
  pinMode(13,OUTPUT); // on Led
  digitalWrite(13,HIGH);
  
  pinMode(7, INPUT_PULLUP); //buttons
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  
  // SETUP LEDS
  pinMode(rLed,OUTPUT);
  pinMode(gLed,OUTPUT);
  pinMode(bLed,OUTPUT);
  ledWrite(rLed,0);
  ledWrite(gLed,0);
  ledWrite(bLed,0);
  
}

void loop() {
  //recieve MIDI messages
  if (!debugging) {
    usbMIDI.read();
  }
  
  // read buttons
  button0.update();
  button1.update();
  button2.update();
  button3.update();
  
  // READ VALUES
  x[counter] = analogRead(xPin);
  y[counter] = analogRead(yPin);
  z[counter] = analogRead(zPin);
  ir[counter] = analogRead(irPin);

  // check the button states
  // if pressed
  if (button0.fallingEdge()) {
    midiNoteOnOff(true,60); // send button press note on
    state+=1; // update triggered state
    xTrig = analogRead(xPin); // save reading
    xTrig = constrain(xTrig,390,500); // probably you will have to change these values for your sensor
    ledWrite(rLed,0); // disable leds
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button1.fallingEdge()) {
    midiNoteOnOff(true,61);
    state+=2;
    yTrig = analogRead(xPin);
    yTrig = constrain(xTrig,460,570);
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button2.fallingEdge()) {
    midiNoteOnOff(true,62);
    state+=4;
    zTrig = analogRead(zPin);
    zTrig = constrain(zTrig,410,580);
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button3.fallingEdge()) {
    midiNoteOnOff(true,63);
    state+=8;
    irTrig = analogRead(irPin);
    irTrig = constrain(irTrig,10,900); // you almso might have to change this for your sensor
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }

  // released
  if (button0.risingEdge()) {
    midiNoteOnOff(false,60); // send note off
    state-=1; // update triggered state
    ledWrite(rLed,0); // disable leds
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button1.risingEdge()) {
    midiNoteOnOff(false,61);
    state-=2;
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button2.risingEdge()) {
    midiNoteOnOff(false,62);
    state-=4;
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  if (button3.risingEdge()) {
    midiNoteOnOff(false,63);
    state-=8;
    ledWrite(rLed,0);
    ledWrite(gLed,0);
    ledWrite(bLed,0);
  }
  
  // check triggered states
  switch (state) {
    case OFFSTATE: // nothing pressed, do nothing
      break;
      
    case XSTATE: // first button is pressed, rotating wrist inwards movement
                 // like checking the watch
      xSmooth = 0;
      for (int i=0; i<AVERAGE_COUNT; i++) // calculate the average of AVERAGE_COUNT readings
        xSmooth += x[i];                  // for a smooth value
      xSmooth = xSmooth / (AVERAGE_COUNT);
      xSmooth = constrain(xSmooth,xTrig,560); // constrain probably you might have to calibrate and change this values for your sensor
      ledWrite(bLed,map(xSmooth,xTrig,560,0,255)); // update the led
      
      midiCC(map(xSmooth,xTrig,560,0,127),1); // send out cc message
      
      break;
      
    case YSTATE: // second button, tilting wrist up and down movement
      
      ySmooth = 0;
      for (int i=0; i<AVERAGE_COUNT; i++) // get smooth value
        ySmooth += y[i];
      ySmooth = ySmooth / (AVERAGE_COUNT);
      ySmooth = constrain(ySmooth,yTrig-30,610);
      ledWrite(gLed,map(ySmooth,yTrig-30,610,0,255));
      
      midiCC(map(ySmooth,yTrig-30,610,0,127),2); // send cc
      
      break;
      
    case ZSTATE: // third botton, rotate wrist outwards, palm inwards
      
      zSmooth = 0;
      for (int i=0; i<AVERAGE_COUNT; i++)
        zSmooth += z[i];
      zSmooth = zSmooth / (AVERAGE_COUNT);
      zSmooth = constrain(zSmooth,zTrig,600); // constrain
      ledWrite(rLed,map(zSmooth,zTrig,600,0,255)); //update led colors
      ledWrite(bLed,map(zSmooth,zTrig,600,0,255));
      
      midiCC(map(zSmooth,zTrig,600,0,127),3); // send cc

      break;
      
    case IRSTATE: // forth button pressed, infrared sensor
    
      irSmooth = 0;
      for (int i=0; i<AVERAGE_COUNT; i++) // smooth value
        irSmooth += ir[i];
      irSmooth = irSmooth / (AVERAGE_COUNT);
      irSmooth = constrain(irSmooth,irTrig,970);
      ledWrite(gLed,map(irSmooth,irTrig,970,0,255)); // update leds
      ledWrite(rLed,map(irSmooth,irTrig,970,0,255));

      midiCC(map(irSmooth,irTrig,970,0,127),4); // send cc
      break;
     
    case SLOPEYSTATE: // second and third buttons pressed, send sample trigger
      midiNoteOnOff(false,61); // disable the effects
      midiNoteOnOff(false,62); //
      xSlope = abs(x[counter]-xLast); // calculate the rate of change
      ySlope = abs(y[counter]-yLast); // ie sudden accelerations
      zSlope = abs(z[counter]-zLast);
      
      slope = (xSlope+ySlope+zSlope)/3; // for all 3 axis
      
      ledWrite(bLed,map(slope,0,1024,0,255));
      ledWrite(gLed,map(slope,0,1024,0,255));
      if (slope>45) { // if accelerated over this value and longer than trigger time
        if ((millis()-lastTrigger)>triggerTime){
          midiNoteOnOff(true,5); // send out note on
          ledWrite(bLed,255); // blink led
          ledWrite(gLed,255);
          lastTrigger=millis(); // idate last trigger time
        }
        else{
          midiNoteOnOff(false,5); // note off
        }
      }
      break; 
      
    case SLOPEZSTATE: // third and forth buttons pressed, send sammple trigger
      midiNoteOnOff(false,63);
      midiNoteOnOff(false,62);
      xSlope = abs(x[counter]-xLast); // calculate the rate of change for x axis
      ledWrite(rLed,map(xSlope,0,1024,0,255));
      if (xSlope>58){ // if over threshold send note on
        if ((millis()-lastTrigger)>triggerTime){
          //Serial.println(xSlope);
          midiNoteOnOff(true,6);
          ledWrite(rLed,255);
          lastTrigger=millis();
        }
        else{
          midiNoteOnOff(false,6);
        }
      }
      break;
  }
  
  xLast=x[counter]; // update last reading values used for finding the slope
  yLast=y[counter];
  zLast=z[counter];
  counter = (counter+1)%AVERAGE_COUNT; // update counter
  
  delay(5);
}

// helper function for common anode leds (255 = 0)
void ledWrite( int pin, int value){
  analogWrite(pin, map(value, 0, 255, 255, 0));
}

// function to handle noteon outgoing messages
void midiNoteOnOff(boolean s, int n) {

  if (s) {
    if (debugging) {//debbuging enabled
      Serial.print("Button ");
      Serial.print(n);
      Serial.println(" pressed.");
    }
    else {
      usbMIDI.sendNoteOn(n, 127, midiChannel);
    }
  }
  else {
    if (debugging) {//debbuging enabled
      Serial.print("Button ");
      Serial.print(n);
      Serial.println(" released.");
    }
    else {
      usbMIDI.sendNoteOff(n, 0, midiChannel);
    }
  }
}

// function to handle outgoing cc messages
void midiCC(int v, int n) {
  if (debugging) {//debbuging enabled
    Serial.print("P:");
    Serial.print(n);
    for (int i=0;i<map(v,0,127,0,210); i++)
      Serial.print("|");
    Serial.println("");
  }
  else {
    usbMIDI.sendControlChange(n, v, midiChannel);
  }
}

// function to handle incoming cc messages
void myCCIn(byte channel, byte control, byte value) {
    if ((channel==midiChannel)&&(control==0)&&(state==0)) { // correct channel and cc message set all leds to value
        ledWrite(rLed,map(value,0,127,0,255));
        ledWrite(gLed,map(value,0,127,0,255));
        ledWrite(bLed,map(value,0,127,0,255));
    }
}
