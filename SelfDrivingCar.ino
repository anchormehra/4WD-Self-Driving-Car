/*
 * Vikash Mehra
 */
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include "Coordinates.h"

//GPS
TinyGPSPlus gps;
SoftwareSerial ss(D7,D8); //GPS Pins on Node MCU

//Compass
MechaQMC5883 qmc;

//Compass Variables
int x, y, z;
int azimuth;


//GPS Variables
double latitude , longitude;
String lat_str , lng_str; //for debugging
int couarseChange;
int coarseToDestination;
double distanceToDestination;
double prevDistanceToDestination;

//motor controller pins
int in1 = 16;
int in2 = 5;
int in3 = 4;
int in4 = 0;


int getAzimuth(){
  
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z,&azimuth);
  azimuth = qmc.azimuth(&y,&x);
  return 360-azimuth;
}

void turnRight(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
void turnLeft(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
void goForward(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); 
    digitalWrite(in4, HIGH);
    delay(5000);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
void goBackward(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void setup(){
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    
    Serial.begin(115200);
    ss.begin(9600);
    
    Wire.begin(12,14); // Magnetic compass pins SDA D6; SCL D5
    qmc.init(); //compass sensor init
}
void loop(){
  Goola();
  
}
void Goola(){
Serial.println("Entered Goola");
Serial.println(gps.satellites.value());
while (ss.available() > 0){
    if (gps.encode(ss.read()))
    {
      if (gps.location.isValid())
      {
        
azimuth = getAzimuth();
Serial.print("Azimuth : ");
Serial.println(azimuth);
//Serial.print("Course to : ");
//Serial.println(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), 33.676772, -117.913715));
//Serial.print("Distance to : ");
//Serial.println(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),33.676772, -117.913715));

for (int i=0; i<wayPointsSize; i++){

do{
    while (ss.available() > 0){
    if (gps.encode(ss.read()))
    {
      if (gps.location.isValid())
      {


do{
  distanceToDestination=TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),wayPoints[i][0], wayPoints[i][1]);
}while(distanceToDestination!=prevDistanceToDestination);
//do{

coarseToDestination=TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), wayPoints[i][0], wayPoints[i][1]);
couarseChange=coarseToDestination-azimuth;

if(couarseChange<0)
  couarseChange+=360;

if(couarseChange>5 && couarseChange<180){
  turnRight();
  Serial.println("Turning Right");
  Serial.println(i);
}
else if(couarseChange>179 && couarseChange<355){
  turnLeft();
  Serial.println("Turning Left");
  Serial.println(i);
}

else{
  goForward();
  Serial.println("Go Forward");
  Serial.println(i);
}

azimuth = getAzimuth();

//}while(abs(couarseChange<5));

//goForward();
//Serial.println("Going Forward");
if(distanceToDestination<2.5)
  i++;
  if (i>wayPointsSize-1)
    i=0;
Serial.print("Azimuth : ");
Serial.println(azimuth);
Serial.print("Distance to Destination : ");
Serial.println(distanceToDestination);
Serial.print("Coarse to Destination : ");
Serial.println(coarseToDestination);
Serial.print("Coarse change: ");
Serial.println(couarseChange);
Serial.println(gps.location.lat());
Serial.println(gps.location.lng());
delay(500);
      }
      }
      }
}while(distanceToDestination>2.5);
}    
}
}
}
prevDistanceToDestination=distanceToDestination;
delay(500);
}
