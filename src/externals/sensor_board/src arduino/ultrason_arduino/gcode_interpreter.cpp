#include "gcode_interpreter.h"

void print_gcode2serial(String inputString, double *distance, int NUMBER_SENSOR)
{
    // Serial.println(inputString);
    if (inputString[0] == 'S')
    {
        if (inputString[1] == 'A')
        {
            // Print all data
            for (int x = 0; x < NUMBER_SENSOR; x++)
            {
                Serial.print(distance[x]);
                if (x != NUMBER_SENSOR - 1)
                    Serial.print("; "); // Delete ; at end of the line
            }
            Serial.println();
        }
        else
        {
            int index = inputString.substring(1).toInt();
            if (-1 < index && index < NUMBER_SENSOR)
            {
                Serial.println(distance[index]);
            }
        }
    }
}