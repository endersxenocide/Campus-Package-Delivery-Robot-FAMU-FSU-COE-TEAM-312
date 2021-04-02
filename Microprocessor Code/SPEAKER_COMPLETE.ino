#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;
char command;
int pausa = 0;

void setup() {

  // Init USB serial port for debugging
  Serial.begin(9600);
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
   Serial.println("OK");

    // Set volume to maximum (0 to 30).
    player.volume(30);
    // Play the first MP3 file on the SD card
    player.play(1);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void loop() {
  
  while (Serial.available() > 0)
  {
    command = Serial.read();

    if (command == 's')
    {
      player.stop();
      Serial.println("Music Stopped!");
      menu_opcoes();
    }
    //Decreases volume
    if (command == '-')
    {
      player.volumeDown();
      Serial.print("Current Volume:");
      Serial.println(player.readVolume());
      menu_opcoes();
    }
    if (command == '+')
    {
      player.volumeUp();
      Serial.print("Current volume:");
      Serial.println(player.readVolume());
      menu_opcoes();
    }
    if (command == 'p')
    {
      pausa = !pausa;
      if (pausa == 0)
      {
      Serial.println("Continue...");
      player.start();
      }
      
      if (pausa == 1)
      {
      Serial.println("Music Paused!");
      player.pause();
      }
    }
  }
  
  
}

void menu_opcoes()
{
  Serial.println();
  Serial.println(F("=================================================================================================================================="));
  Serial.println(F("Commands:"));
  Serial.println(F(" [s] stopping reproduction"));
  Serial.println(F(" [p] pause/continue music"));
  Serial.println(F(" [+ or -] increases or decreases the volume"));
  Serial.println();
  Serial.println(F("================================================================================================================================="));
}
