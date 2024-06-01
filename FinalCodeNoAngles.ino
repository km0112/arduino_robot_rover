#include <Servo.h>
#include <Math.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include "24s_PubSubClient.h"
#include "24s_WiFiManager.h"
#include <Ultrasonic.h>
#include "SSD1306.h" //double check if this last one is correct 

#define motor1pin D0
#define motor2pin D2 
SSD1306 display(0x3C,D14,D15); 


//MQTT Communication associated variables 
char payload_global[100];
boolean flag_payload;

const char* mqtt_server= "192.168.0.140";               //MQTT Broker(Server) Address
const char* MQusername = "user";               //MQTT username
const char* MQpassword = "Stevens1870";               //MQTT password
const char* MQtopic    = "arena2";               //MQTT Topic (Arena I/II)
const int mqtt_port    = 1883   ;          //MQTT TCP/IP port number
//WiFi Setting variables
const char* ssid     = "TP-Link_4DB1";                 //Wi-Fi SSID (Service Set IDentifier)  
const char* password = "01747331";                 //Wi-Fi Password

//wifi define
WiFiClient espClient; 
PubSubClient client(espClient); 


Ultrasonic ultrasonic_one(D11,D10); //front (set in printed part)
Ultrasonic ultrasonic_two(D9,D12); //left (set in printed part)
Ultrasonic ultrasonic_three(D14,D15); //right (set in printed part)

Servo motor1;
Servo motor2;

int dist1; //front 
int dist2; //left 
int dist3; //right 

int prevX = 0;  
int prevY = 0;  //previous x and y coords 
int distX, distY; //distance horizontally to target and vertically 
int x,y; //current x and y coords 
int changeX,changeY;  

int targets [][2] = {{100,650 }, {1600 ,150 }, { 2000,700 }, {110 ,150}, {700,130 }}; //points the robot is supposed to go to
//first coord is starting point 
int totDist; 
int targetIndex = 1; //index of target in target list (which target we are going to rn)
int crossProd; 
float angle;  

void setup_wifi() 
{
 delay(10);
 // We start by connecting to a Stevens WiFi network
 WiFi.begin(ssid, password);          
 while (WiFi.status() != WL_CONNECTED) 
 {
   delay(500);
   Serial.print(".");                       
 }
 randomSeed(micros());                      
}


void callback(char* topic, byte* payload, unsigned int length) 
{
 for (int i = 0; i < length; i++) 
 {
   payload_global[i] = (char)payload[i];
 }
 payload_global[length] = '\0';             
 flag_payload = true;                       
}


void reconnect() 
{                                                               
 // Loop until we're reconnected
 while (!client.connected()) 
 {
   // Create a random client ID
   String clientId = "ESP8266Client-";      
   clientId += String(random(0xffff), HEX); 
   // Attempt to connect                    
   if (client.connect(clientId.c_str(),MQusername,MQpassword)) 
   {
     client.subscribe(MQtopic);            
   } 

   else 
   {
     // Wait 5 seconds before retrying
delay(5000);
   }
 }
}




void setup() 
{
// put your setup code here, to run once:
     Serial.begin(115200);
//LiDAR code
     setup_wifi();                              
     delay(3000);
     Serial.println("Wemos POWERING UP ......... ");
     client.setServer(mqtt_server, mqtt_port); //This 1883 is a TCP/IP port number for MQTT
 client.setCallback(callback);
//motor code 
     motor1.attach(D0);
     motor2.attach(D2); //left side is D2
//Oled code 
     display.init();
     display.flipScreenVertically();
     display.display(); 
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    Serial.print("...");
    reconnect();
  }
  client.loop(); 

 String payload(payload_global);             
 int testCollector[10];                     
 int count = 0;
 int prevIndex, delimIndex;
  
 prevIndex = payload.indexOf('[');          
 while( (delimIndex = payload.indexOf(',', prevIndex +1) ) != -1){
   testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
   prevIndex = delimIndex;
 }
 delimIndex = payload.indexOf(']');
 testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
 
 int x, y;
 //Robot location x,y from MQTT subscription variable testCollector
 x = testCollector[0];
 y = testCollector[1];

  dist1 = ultrasonic_one.read(CM); 
  delay(500); 
  dist2 = ultrasonic_two.read(CM); 
  delay(500); 
  dist3 = ultrasonic_three.read(CM); 
  motor1.write(0);
  motor2.write(0); 

while(targetIndex<5) //while there are still targets 
{ 
  double distX;
  double distY; 
  double totDist; 
  double vectorMags; 
  double vectorRX;
  double vectorRY; 
  double vectorTX;
  double vectorTY; 


while(totDist>5)
{
  dist1 = ultrasonic_one.read(CM); 
  delay(500); 
  dist2 = ultrasonic_two.read(CM); 
  delay(500); 
  dist3 = ultrasonic_three.read(CM);

   distX = targets[targetIndex][0] - x; //distance away from target in x dir 
   distY = targets[targetIndex][1] - y; //distance away from target in y dir 
   totDist = sqrt(pow(distX, 2)+pow(distY,2)); //tot dist to target
   
   vectorRX = x-prevX;
   vectorRY = y-prevY;
   vectorTX = targets[targetIndex][0]-x;
   vectorTY = targets[targetIndex][1]-y; 

  vectorMags = sqrt(pow(vectorRX,2)+pow(vectorTX,2))*sqrt(pow(vectorRY,2)+pow(vectorTY,2));
  double acosIn = vectorRX*vectorTX+vectorRY*vectorTY/vectorMags; 
  angle =  acos(acosIn); 
  crossProd =  vectorMags*sin(angle); 
  if(dist1 <=5) //something near front of robot
  {
    motor1.write(180);
    motor2.write(180); 
  }
delay(500); 
  if(dist2 <= 5)
  {
    motor1.write(180);
    delay(250);
    motor1.write(0); 
  }

  if(dist3 <= 5)
  {
    motor2.write(180); 
    delay(250);
    motor2.write(0); 
  }

  if(angle <70 && angle > 5)
  {
    if(crossProd>0)
    {
      motor1.write(0);
      motor2.write(20); //left motor goes slightly slower so it turns left 
    }
    else //cross prod negative 
    {
      motor1.write(20);
      motor2.write(0); //right motor slightly slower so it turns right 
    }
  }

  if(angle >70 && angle <110)
  {
    if(crossProd>0) //left turn 
    {
      motor1.write(0);
      motor2.write(180); 
      delay(500); //set delay to account for 1/4 turn 
      motor2.write(0); 
    }
    if(crossProd<0) //right turn
    {
      motor2.write(0);
      motor1.write(180); 
      delay(500); //set delay to account for 1/4 turn 
      motor1.write(0); 
    }
  }

  if(angle>=110)
  {
    //turn 180 degrees 
    motor1.write(0);
    motor2.write(180); 
    delay(1000);
    motor2.write(0); 
  }

  else //angle <5
  {
    motor1.write(0);
    motor2.write(0); 
  }
}
  if(totDist<= 5) //totdist<= 5 cm stop for a sec, target reached
  {
    motor1.write(90);
    motor2.write(90); 
    delay(1000);
    targetIndex++; //shifts goal to next target 
  }

}
  

  

}
