//Meridian SD write read test

#include <SD.h>
#include <SPI.h>

File myFile;

const int chipSelect = 9;

void setup()
{
 //UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD:
 SPI.setMOSI(11); // Teensy4.0 with Meridian Board
 SPI.setSCK(13);  // Teensy4.0 with Meridian Board
 SPI.setMISO(12); // Teensy4.0 with Meridian Board

 // Open serial communications and wait for port to open:
  Serial.begin(6000000);
   while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. 
  myFile = SD.open("test.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("Meridian Board read write test.");
  // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("Opening test.txt...");
    
    // read from the file until there's nothing else in it:
        Serial.println("Reading texts in test.txt:");

    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop()
{
  // nothing happens after setup
}
