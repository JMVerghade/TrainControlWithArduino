/********************
* Jean-Marie Verghade
* Date : 2014 feb 3td
* Creates a minimum DCC command station with cmdrArduino from Railstars http://railstars.com/software/cmdrarduino/
* The DCC waveform is output on Pin 9, and is suitable for connection to an LMD18200-based booster directly,
* or to a single-ended-to-differential driver, to connect with most other kinds of boosters.
* The pin 10 is also used, but not for the LMD18200 HBridge
* Manage also a 2x16 Text display and an 38kHz Infrared receiver to supply a user freindly man machine interface
* Serial communication is also implemented in order to connect the DCC central to a computer thanks to a USB-RS232 serial link
* and the Modbus protocol. Specific modbus functions are added to the standard protocol to control the central
********************/

#include <DCCPacket.h>
#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include <IRremote.h>
/*
  LiquidCrystal Library - Hello World
 
 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.
 
 This sketch prints "Hello World!" to the LCD
 and shows the time.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 8
 * LCD D4 pin to digital pin 7
 * LCD D5 pin to digital pin 6
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 4
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 
 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

// include the library code:
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 8, 7, 6, 5, 4);

// Infrared receiver 
int RECV_PIN = 13;
decode_results results;
decode_results last_results;
IRrecv irrecv(RECV_PIN);

// FOR Locomanagement with remote IR 
byte IR_CurrentLoco = 0;

DCCPacketScheduler dps;
//Setup the brewtrollers register bank
//All of the data accumulated will be stored here
modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

byte count = 0;

boolean ESD = false;

#define LOCO_MAX_SPEED 127  //max speed for loco
#define LOCO_NULL_SPEED 1 //Speed = 0 means emergency stop
#define LOCO_DIR_FORWARD 0
#define LOCO_DIR_BACKWARD 1
#define LOCO_MAX_NUMBER 10 // beware 20 locos avoid the program to run normally
#define LOCO_MAX_FUNCTIONS 10 // beware 16 functions bugs - to be calculated with LOCO_MAX_NUMBER

#define BOOSTER_PWM 3 // enable booster
//#define MMI_ESD 3 // Emergency Stop
//#define START_LED   13  // on when start, off when stop
#define BOOSTER_OVER_LED    11  // on if current too high
//#define DEBUG_LED 12


boolean debug_led_state = false;
char a;

class clsLoco
{
  public:
  byte m_bAdress;
  int m_iCurrentSpeed;
  byte m_bCurrentDir;
  int m_bLocoFunction[LOCO_MAX_FUNCTIONS];
};

clsLoco Locos[LOCO_MAX_NUMBER];

void LocosArraySetup()
{
  byte i;
  for (i=0;i<LOCO_MAX_NUMBER;i++)
  {
    Locos[i].m_bAdress=i+1;
    Locos[i].m_iCurrentSpeed=LOCO_NULL_SPEED;
    Locos[i].m_bCurrentDir=LOCO_DIR_FORWARD;
    byte j;
    for (j=0;j<LOCO_MAX_FUNCTIONS;j++) Locos[i].m_bLocoFunction[j]=0;
  }
}// LocosArraySetup
/*************************************************
**  Setup procedure
*************************************************/
void setup() {
  // initialize the library with the numbers of the interface pins
  
    // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Centrale DCC starting !");
  // Initialize Locos array
  LocosArraySetup();
  // Setup the DCC packet scheduler
  dps.setup();
  // Start the infrared receiver
  irrecv.enableIRIn(); 
  
  //set up booster enable pin
  pinMode(BOOSTER_PWM, OUTPUT);
  digitalWrite(BOOSTER_PWM, LOW); //disable booster
  
  //pinMode(START_LED,OUTPUT);
  //digitalWrite(START_LED,HIGH);
  
  //set up a LED to inform booster status
  pinMode(BOOSTER_OVER_LED,OUTPUT);
  digitalWrite(BOOSTER_OVER_LED,HIGH);
  
  //pinMode(DEBUG_LED,OUTPUT);
  //digitalWrite(DEBUG_LED,HIGH);
  
  //pinMode(MMI_ESD,INPUT);
  //digitalWrite(MMI_ESD,1);
  
  //Assign the modbus device ID.  
  regBank.setId(1);
  regBank.add(30001);  
  regBank.add(30002);
/*
Assign the modbus device object to the protocol handler
This is where the protocol handler will look to read and write
register data.  Currently, a modbus slave protocol handler may
only have one device assigned to it.
*/
  slave._device = &regBank;  

// Initialize the serial port for coms at 9600 baud  
  slave.setBaud(19200);   
  
  slave.LocoThrottleRegister(&(ThrottleManage));
  slave.LocoFunctionRegister(&(FunctionManage));
  slave.LocoESDRegister(&(ESDManage));
}

void FunctionManage(byte adr, byte function, byte functionvalue)
// this function is called by the Modbus driver if it has been registered first
{
  // seek the loco slot regarding adress
  byte i=0;
  while ((i<LOCO_MAX_NUMBER) && (Locos[i].m_bAdress!=adr))
  {
    i++;
  }
  //Serial.print("Loco slot found=");
  //Serial.print(i);
    if(functionvalue==1) {Locos[i].m_bLocoFunction[function]=1;} else {Locos[i].m_bLocoFunction[function]=0;}
    if (function<=4) {
      debug_led_state=!debug_led_state;
      
      dps.setFunctions0to4(adr,DCC_SHORT_ADDRESS,Locos[i].m_bLocoFunction[0] | Locos[i].m_bLocoFunction[1]<<1 | Locos[i].m_bLocoFunction[2]<<2 | Locos[i].m_bLocoFunction[3] <<3 | Locos[i].m_bLocoFunction[4]<<4); 
    }
    else
    if (function <=8)
    {
      dps.setFunctions5to8(adr,DCC_SHORT_ADDRESS,Locos[i].m_bLocoFunction[5] | Locos[i].m_bLocoFunction[6]<<1 | Locos[i].m_bLocoFunction[7]<<2 | Locos[i].m_bLocoFunction[3] <<3 | Locos[i].m_bLocoFunction[8]<<4); 
    }
    else
    if (function <=12)
    {
      dps.setFunctions9to12(adr,DCC_SHORT_ADDRESS,Locos[i].m_bLocoFunction[9] | Locos[i].m_bLocoFunction[10]<<1 | Locos[i].m_bLocoFunction[11]<<2 | Locos[i].m_bLocoFunction[3] <<3 | Locos[i].m_bLocoFunction[12]<<4); 
    }
}

void ThrottleManage(byte adr, byte dir, int locospeed)
// this function is called by the Modbus driver if it has been registered during Setup
{
    //if(locospeed == 0) //this would be treated as an e-stop!
    //{
    //  locospeed = 1;
    //}
    if (dir==1) {locospeed = -locospeed;}
    debug_led_state=!debug_led_state;
    dps.setSpeed128(adr,DCC_SHORT_ADDRESS,locospeed);
    regBank.set(30001,adr);
    regBank.set(30002,locospeed);

}

void ESDManage(byte adr)
{
  debug_led_state=!debug_led_state;
  dps.setSpeed128(adr,DCC_SHORT_ADDRESS,0);
  //ESD=true;
}

void LCDManage()
// Display current loco informations
{
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0,0);
   lcd.print("LOC VIT F");
   lcd.setCursor(0, 1);
   lcd.print(Locos[IR_CurrentLoco].m_bAdress);
   lcd.print(" ");
   lcd.setCursor(4, 1);
   lcd.print(Locos[IR_CurrentLoco].m_iCurrentSpeed);
   lcd.print(" ");
   lcd.setCursor(8,0);
   for (byte i=0;i<8;i++) { if (Locos[IR_CurrentLoco].m_bLocoFunction[i]==1) lcd.print("1"); else lcd.print("0"); }
   lcd.setCursor(8,1);
   for (byte i=0;i<LOCO_MAX_FUNCTIONS-8;i++) { if (Locos[IR_CurrentLoco].m_bLocoFunction[i+8]==1) lcd.print("1"); else lcd.print("0"); }
  // print the number of seconds since reset:
  //lcd.print(millis()/1000);
}// LCDManage

void loop() {
  //if (digitalRead(MMI_ESD)) { ESD=!ESD; }
  if (false)
  {
    digitalWrite(BOOSTER_OVER_LED,HIGH);   // then stop LED ON
    digitalWrite(BOOSTER_PWM,LOW);     // and stop booster
    //digitalWrite(START_LED,LOW);
    } 
    else {                       // else RUN position
     //digitalWrite(START_LED,HIGH);
     digitalWrite(BOOSTER_PWM,HIGH); // enable Booster (if it was disable above)
     digitalWrite(BOOSTER_OVER_LED,debug_led_state);  // overled OFF - STOP LED OFF
   
  }
// Modbus slave   
 slave.run(); 
  
  //dps.setFunctions0to4(dcc_address,DCC_SHORT_ADDRESS,bLocoLight | bLocoSound<<1 | bLocoF2<<2 | bLocoF3 <<3 | bLocoF4<<4); 
 
  //digitalWrite(BOOSTER_OVER_LED,HIGH);
  dps.update();
   
   //digitalWrite(DEBUG_LED,debug_led_state);
   //digitalWrite(BOOSTER_OVER_LED,LOW); 

   LCDManage();
   
  
if (irrecv.decode(&results)) {
    //Serial.print(results.value, HEX);
    //Serial.print(" : ");
    if((results.value==0xFFFFFFFF)&&(last_results.value!=0)) {IR_CommandTreat(last_results);} else { IR_CommandTreat(results);}
    //IR_CommandTreat(results);
    //irrecv.resume(); // Receive the next value
  }
}

void IR_Accelerate()
{
  if (Locos[IR_CurrentLoco].m_iCurrentSpeed<LOCO_MAX_SPEED) { Locos[IR_CurrentLoco].m_iCurrentSpeed++; if (Locos[IR_CurrentLoco].m_iCurrentSpeed==0) Locos[IR_CurrentLoco].m_iCurrentSpeed=LOCO_NULL_SPEED; }
  if (Locos[IR_CurrentLoco].m_iCurrentSpeed>0) Locos[IR_CurrentLoco].m_bCurrentDir=LOCO_DIR_FORWARD; else Locos[IR_CurrentLoco].m_bCurrentDir = LOCO_DIR_BACKWARD;
  ThrottleManage(Locos[IR_CurrentLoco].m_bAdress,Locos[IR_CurrentLoco].m_bCurrentDir,abs(Locos[IR_CurrentLoco].m_iCurrentSpeed));
}//IR_Accelerate

void IR_Stop()
{
  Locos[IR_CurrentLoco].m_iCurrentSpeed=LOCO_NULL_SPEED;
  Locos[IR_CurrentLoco].m_bCurrentDir=LOCO_DIR_FORWARD;
  ThrottleManage(Locos[IR_CurrentLoco].m_bAdress,Locos[IR_CurrentLoco].m_bCurrentDir,abs(Locos[IR_CurrentLoco].m_iCurrentSpeed));
}//IR_Stop

void IR_Decelerate()
{
  if (Locos[IR_CurrentLoco].m_iCurrentSpeed>-LOCO_MAX_SPEED) { Locos[IR_CurrentLoco].m_iCurrentSpeed--; if (Locos[IR_CurrentLoco].m_iCurrentSpeed==0) Locos[IR_CurrentLoco].m_iCurrentSpeed=-LOCO_NULL_SPEED; }
  if (Locos[IR_CurrentLoco].m_iCurrentSpeed>0) Locos[IR_CurrentLoco].m_bCurrentDir=LOCO_DIR_FORWARD; else Locos[IR_CurrentLoco].m_bCurrentDir = LOCO_DIR_BACKWARD;
  ThrottleManage(Locos[IR_CurrentLoco].m_bAdress,Locos[IR_CurrentLoco].m_bCurrentDir,abs(Locos[IR_CurrentLoco].m_iCurrentSpeed));
}//IR_Decelerate

void LocoFunctionSwitch(byte FuncNb)
{
  Locos[IR_CurrentLoco].m_bLocoFunction[FuncNb]=abs(Locos[IR_CurrentLoco].m_bLocoFunction[FuncNb]-1);
  FunctionManage(Locos[IR_CurrentLoco].m_bAdress, FuncNb, (byte)Locos[IR_CurrentLoco].m_bLocoFunction[FuncNb]);
}

void IR_CommandTreat(decode_results lResults)
{
      
    switch (lResults.value) {
    case 0xFD00FF: Serial.println("STOP"); debug_led_state=!debug_led_state; break; // 0xB2EEDF3D || 
    case 0xFD807F: Serial.println("VOL+"); break; // 0xEC76CB20 ||
    case 0xFD40BF: Serial.println("FUNC/STOP"); break; //0x2C87261 || 
    case 0xFD20DF: Serial.println("|<<"); IR_Decelerate(); last_results = lResults; break;
    case 0xFDA05F: Serial.println(">||"); IR_Stop(); last_results = lResults; break;
    case 0xFD609F: Serial.println(">>|"); IR_Accelerate(); last_results = lResults; break;
    case 0xFD10EF: Serial.println("\\/"); if (IR_CurrentLoco>0) IR_CurrentLoco--; last_results.value=0;break;
    case 0xFD906F: Serial.println("VOL-"); break;
    case 0xFD50AF: Serial.println("/\\"); if (IR_CurrentLoco<LOCO_MAX_NUMBER) IR_CurrentLoco++; last_results.value=0; break;
    case 0xFD30CF: Serial.println("0"); LocoFunctionSwitch(0); last_results.value=0; break;
    case 0xFDB04F: Serial.println("EQ"); break;
    case 0xFD708F: Serial.println("ST/REPT"); break;
    case 0xFD08F7: Serial.println("1"); LocoFunctionSwitch(1); last_results.value=0; break;
    case 0xFD8877: Serial.println("2"); LocoFunctionSwitch(2); last_results.value=0;break;
    case 0xFD48B7: Serial.println("3"); LocoFunctionSwitch(3); last_results.value=0;break;
    case 0xFD28D7: Serial.println("4"); LocoFunctionSwitch(4); last_results.value=0;break;
    case 0xFDA857: Serial.println("5"); LocoFunctionSwitch(5); last_results.value=0;break;
    case 0xFD6897: Serial.println("6"); LocoFunctionSwitch(6); last_results.value=0;break;
    case 0xFD18E7: Serial.println("7"); LocoFunctionSwitch(7); last_results.value=0;break;
    case 0xFD9867: Serial.println("8"); LocoFunctionSwitch(8); last_results.value=0;break;
    case 0xFD58A7: Serial.println("9"); LocoFunctionSwitch(9); last_results.value=0;break;
    case 0xFFFFFFFF: Serial.print("."); break;
    default:
      Serial.print(lResults.value, HEX);
      Serial.println(" Non reconnu !");
    }
    
    irrecv.resume(); // Receive the next value

}// IR_CommandTreat



