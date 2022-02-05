#include <MaxMatrix.h>
#include <avr/pgmspace.h>

// Ultrasonic HC-SR04 unit interface
// Uses serial port at 115200 baud for communication
// use trig pin for output, echo pin for input
// pulse out (10us) on trig initiates a cycle
// pulse width returned on echo is proportional to distance
// specs say 38ms = no return (beyond limit), but 90ms and more have been seen // set utimeout when calling usonic (routine will take longer for longer returns)
 
// higher timeout measures further, but can take longer if no echo
// if return >= utimeout, no valid pulse received
// if return < ~100 unit is faulty/disconnected (routine is timing out waiting for start of return)
// if return == 0 then unit is still sending return from last ping (or is faulty)
// maximum nominal range is 5m => utimeout = 29000 us
// call usonicsetup() during setup
// call usonic(timeout) to
// divide result of usonic
//define pins here

#define TRIG 10
#define ECHO 4
#define USMAX 3000

int data = 7;    // DIN pin of MAX7219 module
int load = 5;    // CS pin of MAX7219 module
int clock = 6;  // CLK pin of MAX7219 module

int maxInUse = 1;    //change this variable to set how many MAX7219's you'll use

int d; // measured distance
int sample=200; // sample rate delay;
int goal=45; // desired distance (cm) from sensor
int dtogo; // distance remaining to goal
int progress;
int j=1;
int k=0;

int LIGHTpin=A0; //photoresistor to activate
int bright;
int brightON=240; //adjust this variable to set the brightness at which the sensor should activate - will depend upon yuor hardware and ambient light

int tm=50; // ms delay 

MaxMatrix m(data, load, clock, maxInUse); // define module

byte buffer[10];

void setup() {
  m.init(); // module initialize
  m.setIntensity(6); // dot matix intensity 0-15

  Serial.begin(115200);
  usonicsetup();

pinMode(LIGHTpin, INPUT);
}
void loop() {
  bright=analogRead(LIGHTpin);

while (bright>brightON){
  m.clear();
// Serial.print(bright);
// Serial.println(" sensor off");
  delay(tm);
  bright=analogRead(LIGHTpin);
}

    d=usonic(11600)/58; //11600 = 2 metres, 17400 = 3 metres, 23200 = 4 metres, 29000 = 5 metres
    delay(sample);

   while(d==0){
        d=usonic(11600)/58;  
     delay(sample);
   }

        dtogo=d-goal;
    progress=dtogo/10;
    
// Serial.print(bright); //These lines can be used to test the setup
//    Serial.print(" ");
//    Serial.print(d);
//    Serial.print(" ");
//    Serial.print(dtogo);
//    Serial.print(" ");
//    Serial.println(progress);

   if (progress>=8){
  m.clear();
m.setDot(0,7,true);
m.setDot(7,0,true);
m.setDot(0,0,true);
m.setDot(7,7,true);

}

    if(progress<8 && dtogo>0){
  m.clear();
while (k<8){
  m.setDot(0, k, true);
  m.setDot(7, k, true);
  k++;
}
k=0;

while (j<7){
  m.setDot(j, progress, true);
if(progress>0){
  m.setDot(j, progress-1, false);
}
if(progress<7){
m.setDot(j, progress+1, false);  
}
  j++;
}
j=1;
    }

    if(dtogo<=0){
  m.clear();
m.setDot(0,4,true);
m.setDot(1,5,true);      
m.setDot(2,6,true);
m.setDot(3,5,true);
m.setDot(4,4,true);
m.setDot(5,3,true);
m.setDot(6,2,true);
m.setDot(7,1,true);
    }
}

void usonicsetup(void){
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
}

long usonic(long utimeout){
  long b;
//get return time in microseconds by 58 to get range in cm
//open serial port
//set up ultrasonic sensor
//variable to store distance
//distance in cm, time out at 11600us or 2m maximum range //print distance in cm
//wait a bit so we don't overload the serial port
//utimeout is maximum time to wait for return in us

if(digitalRead(ECHO)==HIGH){return 0;} //if echo line is still low from last result, return 0;
digitalWrite(TRIG, HIGH); //send trigger pulse
delay(1);
digitalWrite(TRIG, LOW);
long utimer=micros();
while((digitalRead(ECHO)==LOW)&&((micros()-utimer)<1000)){} //wait for pin state to change-
//return starts after 460us typically or timeout (eg if not connected)
utimer=micros();
while((digitalRead(ECHO)==HIGH)&&((micros()-utimer)<utimeout)){} //wait for pin state to
//change 
b=micros()-utimer; 
if(b==0){b=utimeout;} 
return b;
}
