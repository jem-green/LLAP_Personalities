/*
 Built with Arduino 0022 and PJRC OneWire 2.0 library
 http://www.pjrc.com/teensy/td_libs_OneWire.html
 
 created by Eren BALCI <erenbalci(at)gmail(dot)com>
 December 2012 
 */

#include <OneWire.h>

OneWire ds(2);                    // OneWire bus on digital pin 6

void setup()
{
  Serial.begin (9600);
  delay(250);
}

void loop()
{
  byte i;           // This is for the for loops
  boolean present;  // device present var
  byte data[8];     // container for the data from device
  byte crc_calc;    //calculated CRC
  byte crc_byte;    //actual CRC as sent by DS2401

  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  present = ds.reset();
  
  if (present == TRUE)
  {
    Serial.println("---------- Device present ----------");
    ds.write(0x33);  //Send Read data command
    
    data[0] = ds.read();
    Serial.print("Family code: 0x");
    PrintTwoDigitHex (data[0], 1);
    
    Serial.print("Hex ROM data: ");
    for (i = 1; i <= 6; i++)
    {
      data[i] = ds.read(); //store each byte in different position in array
      PrintTwoDigitHex (data[i], 0);
      Serial.print(" ");
    }
    Serial.println();
 
    crc_byte = ds.read(); //read CRC, this is the last byte
    crc_calc = OneWire::crc8(data, 7); //calculate CRC of the data

    Serial.print("Calculated CRC: 0x");
    PrintTwoDigitHex (crc_calc, 1);
    Serial.print("Actual CRC: 0x");
    PrintTwoDigitHex (crc_byte, 1);
  }
  else //Nothing is connected in the bus
  {
    Serial.println("xxxxx Nothing connected xxxxx");
  }
  
  delay(3000);
}

void PrintTwoDigitHex (byte b, boolean newline)
{
  Serial.print(b/16, HEX);
  Serial.print(b%16, HEX);
  if (newline == true){
    Serial.println();
  }
}

