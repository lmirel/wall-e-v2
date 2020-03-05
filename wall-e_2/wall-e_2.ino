
/*
 BUTTONS
        Analog pin 5
           |
Ground--1K--|--------|--------|-------|-------|
           |        |        |       |       |
          btn1     btn2     btn3    btn4    btn5
           |        |        |       |       |
        220 Ohm  390 Ohm  680 Ohm   2.2K    4.7K
           |--------|--------|-------|-------|-- +5V

*/

/***************************************************
 DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>
 
 ***************************************************
 This example shows the all the function of library for DFPlayer.
 
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
<https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

#define DEBUG 0
#define debug_println(...) \
            do { if (DEBUG) Serial.println(__VA_ARGS__); } while (0)
#define debug_print(...) \
            do { if (DEBUG) Serial.print(__VA_ARGS__); } while (0)

SoftwareSerial mySoftwareSerial (A0, A1); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

int can_walk = 0;
int can_talk = 0;

int fknt = 0;
Servo srvL, srvR;
#define SRVLM 90
#define SRVRM 90

#define LSRV_PIN  11
#define RSRV_PIN  10

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  if (DEBUG)
    Serial.begin(115200);
  //
  mySoftwareSerial.begin(9600);
  //
  srvL.attach(LSRV_PIN);
  srvR.attach(RSRV_PIN);
  srvL.write(SRVLM);  // set servo to mid-point
  srvR.write(SRVRM);  // set servo to mid-point
  srvL.detach();
  srvR.detach();
  //
  debug_println();
  debug_println(F("DFRobot DFPlayer Mini Demo"));
  debug_println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) 
  {  //Use softwareSerial to communicate with mp3.
    debug_println(F("Unable to begin:"));
    debug_println(F("1.Please recheck the connection!"));
    debug_println(F("2.Please insert the SD card!"));
    while(true);
  }
  debug_println(F("DFPlayer Mini online."));
  
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  
  //----Set volume----
  myDFPlayer.volume(30);  //Set volume value (0~30).
  myDFPlayer.volumeDown(); //Volume Down
  myDFPlayer.volumeUp(); //Volume Up
  
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
//  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  
  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);
  
  //----Mp3 control----
//  myDFPlayer.sleep();     //sleep
//  myDFPlayer.reset();     //Reset the module
//  myDFPlayer.enableDAC();  //Enable On-chip DAC
//  myDFPlayer.disableDAC();  //Disable On-chip DAC
//  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
#if 1
  //----Read information----
  Serial.print (F(" state: "));
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.print (F("volume: "));
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.print (F("    eq: "));
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.print (F("file k: "));
  fknt = myDFPlayer.readFileCounts();
  Serial.println(fknt); //read all file counts in SD card
  Serial.print (F("c file: "));
  Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  Serial.print (F(" dir k: "));
  Serial.println(myDFPlayer.readFileCountsInFolder(1)); //read fill counts in folder SD:/03
#endif  
  //myDFPlayer.play(1);  //Play the first mp3
  //delay(1000);
#if 1
  //----Read information----
  Serial.print (F(" state: "));
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.print (F("volume: "));
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.print (F("    eq: "));
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.print (F("file k: "));
  Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  Serial.print (F("c file: "));
  Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  Serial.print (F(" dir k: "));
  Serial.println(myDFPlayer.readFileCountsInFolder(1)); //read fill counts in folder SD:/03
#endif  
#if 0
  //----Mp3 play----
  myDFPlayer.next();  //Play next mp3
  delay(1000);
  myDFPlayer.previous();  //Play previous mp3
  delay(1000);
  myDFPlayer.loop(1);  //Loop the first mp3
  delay(1000);
  myDFPlayer.pause();  //pause the mp3
  delay(1000);
  myDFPlayer.start();  //start the mp3 from the pause
  delay(1000);
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  delay(1000);
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  delay(1000);
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  delay(1000);
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.stopAdvertise(); //stop advertise
  delay(1000);
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  delay(1000);
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  delay(1000);
  myDFPlayer.randomAll(); //Random play all the mp3.
  delay(1000);
  myDFPlayer.enableLoop(); //enable loop.
  delay(1000);
  myDFPlayer.disableLoop(); //disable loop.
  delay(1000);
#endif
  digitalWrite(LED_BUILTIN, LOW);
}

int cf = 0;
const int pingPin = 13; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 13; // Echo Pin of Ultrasonic Sensor
int go_move = 0;

int go_stop()
{
  if (go_move == 0)
    return 1;
  srvL.write(SRVLM);  // set servo to mid-point
  srvR.write(SRVRM);  // set servo to mid-point
  srvL.detach();
  srvR.detach();
  go_move = 0;
  return 0;
}

int go_back ()
{
  if (!can_walk)
    return 1;
  if (go_move == -1)
    return 1;
  srvL.attach(LSRV_PIN);
  srvR.attach(RSRV_PIN);
  srvL.write(80);  // set servo to mid-point
  srvR.write(100);  // set servo to mid-point
  go_move = -1;
  delay(1000);
  return 0;
}

int go_forth ()
{
  if (!can_walk)
    return 1;
  if (go_move == 1)
    return 1;
  srvL.attach(LSRV_PIN);
  srvR.attach(RSRV_PIN);
  srvL.write(100);  // set servo to mid-point
  srvR.write(80);  // set servo to mid-point
  go_move = 1;
  return 0;
}

int go_left ()
{
  if (!can_walk)
    return 1;
  srvL.attach(LSRV_PIN);
  srvR.attach(RSRV_PIN);
  srvL.write(80);  // set servo to mid-point
  srvR.write(80);  // set servo to mid-point
  delay(1000);
  return 0;
}

int go_right ()
{
  if (!can_walk)
    return 1;
  srvL.attach(LSRV_PIN);
  srvR.attach(RSRV_PIN);
  srvL.write(100);  // set servo to mid-point
  srvR.write(100);  // set servo to mid-point
  delay(1000);
  return 0;
}

int walle_talk ()
{
  if (!can_talk)
    return 1;
  //
  int pstate = myDFPlayer.readState(); //read mp3 state
  debug_print (F(" state: "));
  debug_println (pstate);
  if (pstate != 513)
  {
    cf ++;
    if (cf > fknt)
      cf = 1;
    myDFPlayer.play (cf);  //Play next mp3 every 3 second.
    debug_print (F("next :> c file: "));
    debug_println (myDFPlayer.readCurrentFileNumber()); //read current play file number
  }
  return 0;
}

int get_cm()
{
   long duration;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   return microsecondsToCentimeters(duration);
}

#define BTN01 0x02          //..839
#define BTN01V  839
#define BTN02 0x04          //..177
#define BTN02V  177
#define BTN12 (0x02|0x04)   //..846
int lbtn = 0;
int cbtn = 0;

int get_abutton ()
{
  int ret = 0;
  //
  if (lbtn != 0 && cbtn == 0)
  {
    //lbtn was released
    if (lbtn >= BTN01V-5 && lbtn <= BTN01V+5)
      ret |= BTN01;
    if (lbtn >= BTN02V-5 && lbtn <= BTN02V+5)
      ret |= BTN02;
    if (0)
    {
      debug_print(" pbtn: ");
      debug_print(ret);
      debug_println();
    }
  }
  //
  return ret;
}

int scan_buttons ()
{
  static char scnk = 0;
  cbtn = analogRead (A2);
  //
  if (cbtn)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
  //
  if (cbtn != 0)
  {
    lbtn = cbtn;
    scnk = 1;
  }
  if (lbtn != 0 && cbtn == 0)
  {
    if (scnk)
      scnk--;
    else
      lbtn = 0;
  }
  //
  if (0)
  {
    debug_print("cbtn: ");
    debug_print(cbtn);
    debug_print(" lbtn: ");
    debug_print(lbtn);
    debug_println();
  }
  //
  return 0;
}

int too_close = 0;

void loop()
{
  scan_buttons ();
  //
  long cm = get_cm ();
  if (get_abutton() & BTN01)
    can_walk = ~can_walk;
  if (get_abutton() & BTN02)
    can_talk = ~can_talk;
  //
  if (0 == can_walk)
    go_stop ();
  //
  if (1)
  {
    debug_print(millis()%1000);
    debug_print("ms> ");
    debug_print(cm);
    debug_print("cm ");
    debug_print("walk: ");
    debug_print(can_walk);
    debug_print(" talk: ");
    debug_print(can_talk);
    debug_println();
  }
  //
  #if 1
  if (cm < 20 && cm > 0) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (!too_close)
    {
      too_close = 1;
      go_stop ();
      walle_talk ();
      go_back ();
      go_right();
      //turn around
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
  //
  if (cm > 20)
  {
    too_close = 0;
    go_forth();
  }
  #endif
  //delay (100);
  //if (myDFPlayer.available()) 
  //{
  //  printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  //}
}

long microsecondsToCentimeters(long microseconds)
{
   return microseconds / 29 / 2;
}

void printDetail(uint8_t type, int value)
{
  switch (type) 
  {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
