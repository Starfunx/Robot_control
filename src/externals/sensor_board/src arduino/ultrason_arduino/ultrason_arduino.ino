//www.elegoo.com
//2016.12.08
#include "Ultrasonic.h"

#include "pin_configuration.h"
#include "gcode_interpreter.h"

#define FREQ 40 //Hz

// TimeOut = Max.Distance(mm) * 5,88
int TimeOut = 2000 * 5.8; // 1 m de distance max

// Changer le nombre de capteur et les ajouter dans la liste suivante
#define NUMBER_SENSOR 2
Ultrasonic sensor[NUMBER_SENSOR] = {Ultrasonic(TRIG_PIN1, ECHO_PIN1, TimeOut), 
                                    Ultrasonic(TRIG_PIN2, ECHO_PIN2, TimeOut)};


// Début du programme de récupération des distances
double distance[NUMBER_SENSOR] = {0.0 };

void setup() {
   Serial.begin(115200);
}

int x = 0;
String inputString = "";     // a String to hold incoming data
bool stringComplete = false; // whether the string is complete

void loop() {
   // Save sensor data in a table
   distance[x] = sensor[x].Distance();
   x = (x + 1) % NUMBER_SENSOR;

   // Serial read
   if (stringComplete)
   {
      print_gcode2serial(inputString, distance, NUMBER_SENSOR);
      // clear the string:
      inputString = "";
      stringComplete = false;
      Serial.flush();
   }
   // Delay for all sensor
   delay(1 / (NUMBER_SENSOR * FREQ));
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent()
{
   while (Serial.available())
   {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      if (isPrintable(inChar))
         inputString += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '\n')
      {
         stringComplete = true;
      }
   }
}