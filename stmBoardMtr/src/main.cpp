
#include <Arduino.h>

float target = 0.0;

void serialLoop()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      target = received_chars.toFloat();
      Serial.print("Target = ");
      Serial.println(target);
      received_chars = "";
    }
  }
}

void setup()
{

  Serial.begin(115200);
  
  delay(1000);

}
void loop()
{
  serialLoop();
}