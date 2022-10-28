#include "max6675.h"
#include <AutoPID.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

#define NextionSerial Serial1
#define DebugSerial Serial

#define OUTPUT_PINHTR1 2
#define OUTPUT_PINHTR2 3
#define OUTPUT_PINHTR3 4

// Eeprom Addresses
int TCA1 = 0;
int TCA2 = 2;
int TCA3 = 4;
int TCA4 = 6;

//Time left intergers
int MAXTemp = 0;
int RampTime = 0;// Will be set after each Speed set
int TotalRampTime = 0;
int TimeSum = 0;

bool TTS = false;

bool Temprecieved = false;
bool Timerecieved = false;

bool FileWithName = false;
bool FileNameCheck = false;

bool TimeLock = false;
bool Dataprocessing = false;
bool ThermoProces = false;

bool SendData = false;

int thermoDO = 38;
int thermoCLK = 40;

//SD card Reader
bool check = false;
File myFile; // create file for writing data to SD Card shield.
const int chipSelect = 4;
char NameType[8];

//thermocouples
int thermoCS = 22;
int thermoCS1 = 24;
int thermoCS2 = 26;
int thermoCS3 = 28;
int thermoCS4 = 30;
int thermoCS5 = 32;
int thermoCS6 = 34;
int thermoCS7 = 36;

//Read Nextion command temp and time
int ReconW = 4;
int FileName = 3;
int TotalChar = 0;

int SetTemp = 0;
unsigned long Time = 0;
int TempHeatUpState = 0;

//Time setting, multiplier
int Minutes = 1;
int MultiplierTime = Minutes * 60;

String readString; //crucial for the decoding of the nextion data
String Name = ""; // string used to save the name of a product, used for the SD card datalog

//time intervals
int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;
int T5 = 0;
int T6 = 0;
int T7 = 0;
int T8 = 0;
int T9 = 0;
int T10 = 0;
int T11 = 0;
int T12 = 0;
int T13 = 0;
int T14 = 0;


//Temp intervals
int Temp1 = 0;
int Temp2 = 0;
int Temp3 = 0;
int Temp4 = 0;
int Temp5 = 0;
int Temp6 = 0;
int Temp7 = 0;
int Temp8 = 0;
int Temp9 = 0;
int Temp10 = 0;
int Temp11 = 0;
int Temp12 = 0;
int Temp13 = 0;
int Temp14 = 0;

String CSF;
String NTSF;
String TLF;

//Boolean
boolean TEMPR = false;
boolean TIMR = false;

//Speed setting
int SpeedSet = 1;

// String Just in Case
String str = "";

//RunningState
int RunningState = 0;
int ThermoCheck = 0;
int Currentstate = 0;

//Delta Time for waiting time heater
unsigned long deltaTime = 0;
unsigned long WriteSDDelta = 0;
unsigned long SendTemp = 0;
unsigned long UpdateThermo = 0;

unsigned long ToTimeLeft = 0;

//Time Left Function
int TimeLeftRun = 0;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);
MAX6675 thermocouple4(thermoCLK, thermoCS4, thermoDO);
MAX6675 thermocouple5(thermoCLK, thermoCS5, thermoDO);
MAX6675 thermocouple6(thermoCLK, thermoCS6, thermoDO);
MAX6675 thermocouple7(thermoCLK, thermoCS7, thermoDO);

const int RunningAverageCount = 8;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

// Thermocouple Calibration
int ThermoVal1 = 0;
int ThermoVal2 = 0;
int ThermoVal3 = 0;
int ThermoVal4 = 0;

#define KP 1.3
#define KI 0.2// was 0,3
#define KD 1

double temperature = 20;
double outputVal = 10;
double bangOn = 6;
double bangOff = 1;
double Setpoint = 0;

//thermocouples corrections
double ThermoCorrection = 0;
double ThermoCorrection1 = 0;
double ThermoCorrection2 = 0;
double ThermoCorrection3 = 0;
float ThermoCorrection4 = 0;
float ThermoCorrection5 = 0;
float ThermoCorrection6 = 0;
float ThermoCorrection7 = 0;

double outputMin = 0;
double outputMax = 80;

//Time left
long TL = 0;

int TimeaddC = 0;
int TAC = 0;

//State Nextion
int NTS = 0;
int CS = 0;

AutoPID myPID(&temperature, &Setpoint, &outputVal, outputMin, outputMax , KP, KI, KD);

String b;

void sendCmd(String cmd)
{
  NextionSerial.print(cmd);
  NextionSerial.write("\xFF\xFF\xFF");
}

void setup()
{

  analogWrite(OUTPUT_PINHTR1, 0);
  analogWrite(OUTPUT_PINHTR2, 0);
  analogWrite(OUTPUT_PINHTR3, 0);

  //Start serial ports
  DebugSerial.begin(9600);
  NextionSerial.begin(115200);

  // Load Thermocouple Offsets form EEPROM including negative values
  ThermoVal1 = (int8_t)EEPROM.read(TCA1);
  ThermoVal2 = (int8_t)EEPROM.read(TCA2);
  ThermoVal3 = (int8_t)EEPROM.read(TCA3);
  ThermoVal4 = (int8_t)EEPROM.read(TCA4);

  ThermoCorrection4 = ThermoVal1 * 0.25;
  ThermoCorrection5 = ThermoVal2 * 0.25;
  ThermoCorrection6 = ThermoVal3 * 0.25;
  ThermoCorrection7 = ThermoVal4 * 0.25;

  DebugSerial.println("OffSet form EEPROM for Thermocouple 4: ");
  DebugSerial.println(ThermoVal1);
  DebugSerial.println(ThermoCorrection4);
  DebugSerial.println("OffSet form EEPROM for Thermocouple 5: ");
  DebugSerial.println(ThermoVal2);
  DebugSerial.println(ThermoCorrection5);
  DebugSerial.println("OffSet form EEPROM for Thermocouple 6: ");
  DebugSerial.println(ThermoVal3);
  DebugSerial.println(ThermoCorrection6);
  DebugSerial.println("OffSet form EEPROM for Thermocouple 7: ");
  DebugSerial.println(ThermoVal4);
  DebugSerial.println(ThermoCorrection7);

  //Setup PID Bang Bang
  myPID.setBangBang(bangOn, bangOff);
  //set PID update interval to 250ms
  myPID.setTimeStep(250);

  pinMode(53, OUTPUT); //To check if the micro SD is installed

  DebugSerial.print("Initializing SD card...");

 

  while (!NextionSerial)
  {
    DebugSerial.println("Waiting on Nextion Screen");
  }

  sendCmd("ArduinoV.txt = \"Oven\"");
  DebugSerial.println("Sending start up to Nextion Screen");
  delay(500);
  sendCmd("Trigger.val=1");
  sendCmd(""); // clear the buffer
  sendCmd("LiveTemp.txt=\"Initializing\"");
  sendCmd(""); // clear the buffer
  delay(100);

  DebugSerial.println("Awaiting Command of the Nextion Screen or DebugSerial");

  //Nextion recieve data Temp and Time
  TotalChar = ReconW + FileName;
   if (SD.begin())
  {
    DebugSerial.println("SD card is ready to use.");
    sendCmd("page1.SDcardState.txt=\"SD card istalled\"");
  }
  else
  {
    DebugSerial.println("SD card initialization failed");
    sendCmd("page1.SDcardState.txt = \"No SD card istalled\"");
    FileNameCheck = true;
  }

}
void loop()
{
  unsigned long currentMillis = millis();//Start Timer

  String mytext;
  String myvalue;
  String TempvalueAVR;
  String Tempvalue;
  String Tempvalue1;
  String Tempvalue2;
  String Tempvalue3;
  String Tempvalue4;
  String Tempvalue5;
  String Tempvalue6;
  String Tempvalue7;
  String Tempvalue8;

  String Tempval4;
  String Tempval5;
  String Tempval6;
  String Tempval7;

  String TL;
  String NTS;
  String CS;
  String message;
  String Processstate;

  // Thermocouple offset
  String TempOffSet1;
  String TempOffSet2;
  String TempOffSet3;
  String TempOffSet4;

  String OffSetN1;
  String OffSetN2;
  String OffSetN3;
  String OffSetN4;

  String THC4;
  String THC5;
  String THC6;
  String THC7;

  if (NextionSerial.available() > 0) {

    DebugSerial.print("Dataproccesing state is: ");
    DebugSerial.println(Dataprocessing);
    DebugSerial.print("FileNameCheck state is: ");
    DebugSerial.println(FileNameCheck);


    /*  DebugSerial.println("OffSet form EEPROM for Thermocouples: ");
      DebugSerial.println(ThermoCorrection4);
      DebugSerial.println(ThermoCorrection5);
      DebugSerial.println(ThermoCorrection6);
      DebugSerial.println(ThermoCorrection7); */

    DebugSerial.println("Incoming Data form nextion: ");
    DebugSerial.println(readString);

    if (Dataprocessing == true && ThermoProces == false) {
      sendCmd("t16.txt=\"Processing Data\"");
    }

    char c = NextionSerial.read();  //gets one byte from serial buffer

    if (c == '.') {
      if ('Z') {
        DebugSerial.println("Start Thermocouple Calibration process");
        ThermoProces = true;
        DebugSerial.print("Thermoproces state: ");
        DebugSerial.println(ThermoProces);
      }
    }

    if (c == '$') {
      String Trash = readString;
      DebugSerial.print("The recieved Trash: ");
      DebugSerial.println(Trash);
      Trash = "";
      
      //if (NextionSerial.available() == 0)//
     
        DebugSerial.println("Nextion Serial Buffer is empty");
        sendCmd("SendTime.val=1");
        sendCmd("t16.txt=\"Recieved Data\"");
        Dataprocessing = true;
  
      readString += c; //makes the string readString
    }

    if (c == '!') {
      String Trash = readString;
      DebugSerial.print("The recieved Trash: ");
      DebugSerial.println(Trash);
      Trash = "";
      readString = ""; //clears variable for new input
    }

    if (c == 'x') {
      DebugSerial.println("No File Name has been given");
      FileNameCheck = true;
      //Dataprocessing = false;
      //sendCmd("StopSender.val=0");
      readString = ""; //clears variable for new input
    }

    if (c == '>') {
      DebugSerial.println("Recieved Nexttion Data request");
      SendData = true;
      readString = ""; //clears variable for new input
    }

    if (c == '(') {
      FileWithName = false;
      DebugSerial.print("The up comming Run doesn't require a file name ");
    }

    if (c == ')') {
      FileWithName = true;
      DebugSerial.print("The up comming Run requires a File Name: ");
      DebugSerial.print(FileWithName);
    }

    if (c == '@') {
      DebugSerial.println("A Stop command has been isued");
      sendCmd("RunningState.txt=\"Stop\""); //Send to Nextion the running state
      sendCmd("StopSender.val=0"); //Send to Nextion the running state
      readString = ""; //clears variable for new input
      if (check == true) {
        myFile.println("Programm Has been stopped");
        myFile.close(); // if There is a file open it will close it
        check = false;
      }
      sendCmd("t16.txt=\" \"");
      TempHeatUpState = 15; //Stop command
    }

    if (c == ',') {
      Dataprocessing = true;
      while (readString.length() > 1) {
        DebugSerial.print("The Following Data will be processed: ");
        DebugSerial.println(readString); //prints string to serial port out
        int n = readString.toInt();  //convert readString into a number
        DebugSerial.print("Writing Recieved data: ");
        DebugSerial.println(n);
        if (TimeLock == false); {
          if (readString.indexOf('a') > 0) T1 = n;
          if (readString.indexOf('b') > 0) T2 = n;
          if (readString.indexOf('c') > 0) T3 = n;
          if (readString.indexOf('d') > 0) T4 = n;
          if (readString.indexOf('e') > 0) T5 = n;
          if (readString.indexOf('f') > 0) T6 = n;
          if (readString.indexOf('g') > 0) T7 = n;
          if (readString.indexOf('h') > 0) T8 = n;
          if (readString.indexOf('i') > 0) T9 = n;
          if (readString.indexOf('j') > 0) T10 = n;
          if (readString.indexOf('k') > 0) T11 = n;
          if (readString.indexOf('l') > 0) T12 = n;
          if (readString.indexOf('m') > 0) T13 = n;
          if (readString.indexOf('n') > 0) {
            Timerecieved = true;
            T14 = n;
            sendCmd("SendT.val=1");
            sendCmd("SendTime.val=0");
            TimeLock = false;
          }
        }
        if (readString.indexOf('A') > 0) Temp1 = n;
        if (readString.indexOf('B') > 0) Temp2 = n;
        if (readString.indexOf('C') > 0) Temp3 = n;
        if (readString.indexOf('D') > 0) Temp4 = n;
        if (readString.indexOf('E') > 0) Temp5 = n;
        if (readString.indexOf('F') > 0) Temp6 = n;
        if (readString.indexOf('G') > 0) Temp7 = n;
        if (readString.indexOf('H') > 0) Temp8 = n;
        if (readString.indexOf('I') > 0) Temp9 = n;
        if (readString.indexOf('J') > 0) Temp10 = n;
        if (readString.indexOf('K') > 0) Temp11 = n;
        if (readString.indexOf('L') > 0) Temp12 = n;
        if (readString.indexOf('M') > 0) Temp13 = n;
        if (readString.indexOf('N') > 0) {
          Temprecieved = true;
          Temp14 = n;
          sendCmd("vis b5,1");
          sendCmd("t16.txt=\" \"");
          Dataprocessing = false;
          sendCmd("StopSend.val=0");
          
        }

        if (readString.indexOf(':') > 0) {
          SpeedSet = n;
          DebugSerial.print("SpeedSet: ");
          DebugSerial.println(SpeedSet);
          Dataprocessing = false;
        }

        if (readString.indexOf('#') > 0 && FileWithName == true) {
          if (readString.indexOf('#')) Name = readString;
          DebugSerial.print("The recieved name is: ");
          DebugSerial.println(Name);
          int LastIndex = Name.length() - 1;
          Name.remove(LastIndex);
          Name.concat(".TXT");
          DebugSerial.println(Name);

          Name.toCharArray(NameType, Name.length() + 1);
          if (SD.exists(Name)) {// Checking if The Name of the File is already occupied
            sendCmd("page NameConf"); // Opens error page on Nextion
            DebugSerial.println("File name exists.");
            FileNameCheck = false;
          }

          else {
            DebugSerial.println("File name doesn't exist.");
            DebugSerial.println(NameType);
            myFile = SD.open(Name, FILE_WRITE);
            myFile.print("This file originatie from this file name: ");
            myFile.println(Name);

            FileNameCheck = true;
            check = true;

            readString = ""; //clears variable for new input
          }

          Dataprocessing = false;
        }
        readString = ""; //clears variable for new input
      }
    }
    else {
      readString += c; //makes the string readString
    }
  }

  while (ThermoProces == true && Dataprocessing == false) {

    char c = NextionSerial.read();

    //TempOffSet1 = int(thermocouple4.readCelsius() - ThermoCorrection4);

    TempOffSet1 = int(thermocouple4.readCelsius() + ThermoCorrection4);
    TempOffSet2 = int(thermocouple5.readCelsius() + ThermoCorrection5);
    TempOffSet3 = int(thermocouple6.readCelsius() + ThermoCorrection6);
    TempOffSet4 = int(thermocouple7.readCelsius() + ThermoCorrection7);

    THC4 = ThermoVal1 * 0.25;
    THC5 = ThermoVal2 * 0.25;
    THC6 = ThermoVal3 * 0.25;
    THC7 = ThermoVal4 * 0.25;

    OffSetN1 = "\"" + THC4 + "\"";
    OffSetN2 = "\"" + THC5 + "\"";
    OffSetN3 = "\"" + THC6 + "\"";
    OffSetN4 = "\"" + THC7 + "\"";

    // current temperature with compensation

      sendCmd("OffSet1.txt=" + OffSetN1);
      sendCmd("OffSet2.txt=" + OffSetN2);
      sendCmd("OffSet3.txt=" + OffSetN3);
      sendCmd("OffSet4.txt=" + OffSetN4);
  
      sendCmd("THCC1.val=" + TempOffSet1);
      sendCmd("THCC2.val=" + TempOffSet2);
      sendCmd("THCC3.val=" + TempOffSet3);
      sendCmd("THCC4.val=" + TempOffSet4);

    if (c == 'A') {
      ThermoVal1 ++;
      DebugSerial.println("Increasing First Thermocouple");
      ThermoCorrection4 = ThermoVal1 * 0.25;
      sendCmd("OffSet1.txt=" + OffSetN1);
    }
    if (c == 'a') {
      ThermoVal1 --;
      DebugSerial.println("Decreasing First Thermocouple");
      ThermoCorrection4 = ThermoVal1 * 0.25;
      sendCmd("OffSet1.txt=" + OffSetN1);
    }
    if (c == 'B') {
      ThermoVal2 ++;
      DebugSerial.println("Increasing Second Thermocouple");
      ThermoCorrection5 = ThermoVal2 * 0.25;
      sendCmd("OffSet2.txt=" + OffSetN2);
    }
    if (c == 'b') {
      ThermoVal2 --;
      DebugSerial.println("Decreasing Second Thermocouple");
      ThermoCorrection5 = ThermoVal2 * 0.25;
      sendCmd("OffSet2.txt=" + OffSetN2);
    }
    if (c == 'C') {
      ThermoVal3 ++;
      DebugSerial.println("Increasing First Thermocouple");
      ThermoCorrection6 = ThermoVal3 * 0.25;
      sendCmd("OffSet3.txt=" + OffSetN3);
    }
    if (c == 'c') {
      ThermoVal3 --;
      DebugSerial.println("Decreasing First Thermocouple");
      ThermoCorrection6 = ThermoVal3 * 0.25;
      sendCmd("OffSet3.txt=" + OffSetN3);
    }
    if (c == 'D') {
      ThermoVal4 ++;
      DebugSerial.println("Increasing First Thermocouple");
      ThermoCorrection7 = ThermoVal4 * 0.25;
      sendCmd("OffSet4.txt=" + OffSetN4);
    }
    if (c == 'd') {
      ThermoVal4 --;
      DebugSerial.println("Decreasing First Thermocouple");
      ThermoCorrection7 = ThermoVal4 * 0.25;
      sendCmd("OffSet4.txt=" + OffSetN4);
    }
    // Back button
    if (c == 'X') {
      DebugSerial.println("Back Button has been pressed");

      EEPROM.update(TCA1, ThermoVal1);
      EEPROM.update(TCA2, ThermoVal2);
      EEPROM.update(TCA3, ThermoVal3);
      EEPROM.update(TCA4, ThermoVal4);

      DebugSerial.println("Data Has been Saved in the EEPROM");
      ThermoProces = false;
      return;
    }

    readString = ""; //clears variable for new input
  }



  if (Dataprocessing == false && ThermoProces == false) {

    if (Timerecieved == true && Temprecieved == true) {

      DebugSerial.println("Data has been recieved");
      DebugSerial.println(T1);
      DebugSerial.println(T2);
      DebugSerial.println(T3);
      DebugSerial.println(T4);
      DebugSerial.println(T5);
      DebugSerial.println(T6);
      DebugSerial.println(T7);
      DebugSerial.println(T8);
      DebugSerial.println(T9);
      DebugSerial.println(T10);
      DebugSerial.println(T11);
      DebugSerial.println(T12);
      DebugSerial.println(T13);
      DebugSerial.println(T14);
      DebugSerial.println("Recieved data temperature: ");
      DebugSerial.println(Temp1);
      DebugSerial.println(Temp2);
      DebugSerial.println(Temp3);
      DebugSerial.println(Temp4);
      DebugSerial.println(Temp5);
      DebugSerial.println(Temp6);
      DebugSerial.println(Temp7);
      DebugSerial.println(Temp8);
      DebugSerial.println(Temp9);
      DebugSerial.println(Temp10);
      DebugSerial.println(Temp11);
      DebugSerial.println(Temp12);
      DebugSerial.println(Temp13);
      DebugSerial.println(Temp14);

      TIMR = true;
      TEMPR = true;

      Timerecieved = false;
      Temprecieved = false;

      if (check == true && FileNameCheck == true) {
        DebugSerial.println("Writing Run setting to Micro SD card");
        myFile.println("This run has been excudeted with the following setting: ");
        myFile.print("Temperature");
        myFile.print(":");
        myFile.println("Time");

        myFile.print(Temp1);
        myFile.print(":");
        myFile.println(T1);
        myFile.print(Temp2);
        myFile.print(":");
        myFile.println(T2);
        myFile.print(Temp3);
        myFile.print(":");
        myFile.println(T3);
        myFile.print(Temp4);
        myFile.print(":");
        myFile.println(T4);
        myFile.print(Temp5);
        myFile.print(":");
        myFile.println(T5);
        myFile.print(Temp6);
        myFile.print(":");
        myFile.println(T6);
        myFile.print(Temp7);
        myFile.print(":");
        myFile.println(T7);
        myFile.print(Temp8);
        myFile.print(":");
        myFile.println(T8);
        myFile.print(Temp9);
        myFile.print(":");
        myFile.println(T9);
        myFile.print(Temp10);
        myFile.print(":");
        myFile.println(T10);
        myFile.print(Temp11);
        myFile.print(":");
        myFile.println(T11);
        myFile.print(Temp12);
        myFile.print(":");
        myFile.println(T12);
        myFile.print(Temp13);
        myFile.print(":");
        myFile.println(T13);
        myFile.print(Temp14);
        myFile.print(":");
        myFile.println(T14);

      }
    }

    float RawTemperature = (((thermocouple.readCelsius() + thermocouple1.readCelsius() + thermocouple2.readCelsius() + thermocouple3.readCelsius()) / 4));

    RunningAverageBuffer[NextRunningAverage++] = RawTemperature;
    if (NextRunningAverage >= RunningAverageCount)
    {
      NextRunningAverage = 0;
    }
    float RunningAverageTemperature = 0;
    for (int i = 0; i < RunningAverageCount; ++i)
    {
      RunningAverageTemperature += RunningAverageBuffer[i];
    }
    RunningAverageTemperature /= RunningAverageCount;

    DebugSerial.print("Avarage: ");
    DebugSerial.println(RunningAverageTemperature);

    DebugSerial.print("Raw Value Th 1: ");
    DebugSerial.println(thermocouple.readCelsius());

    DebugSerial.print("Raw Value Th 2: ");
    DebugSerial.println(thermocouple1.readCelsius());

    DebugSerial.print("Raw Value Th 3: ");
    DebugSerial.println(thermocouple2.readCelsius());

    DebugSerial.print("Raw Value Th 4: ");
    DebugSerial.println(thermocouple3.readCelsius());

    DebugSerial.print("Raw Value Th 5: ");
    DebugSerial.println(thermocouple4.readCelsius() + ThermoCorrection4);

    DebugSerial.print("Raw Value Th 6: ");
    DebugSerial.println(thermocouple5.readCelsius() + ThermoCorrection5);

    DebugSerial.print("Raw Value Th 7: ");
    DebugSerial.println(thermocouple6.readCelsius() + ThermoCorrection6);

    DebugSerial.print("Raw Value Th 8: ");
    DebugSerial.println(thermocouple7.readCelsius() + ThermoCorrection7);

    if (TIMR == true && TEMPR == true && FileNameCheck == true) {
      if (T1 >= 1)TimeaddC ++;
      if (T2 >= 1)TimeaddC ++;
      if (T3 >= 1)TimeaddC ++;
      if (T4 >= 1)TimeaddC ++;
      if (T5 >= 1)TimeaddC ++;
      if (T6 >= 1)TimeaddC ++;
      if (T7 >= 1)TimeaddC ++;
      if (T8 >= 1)TimeaddC ++;
      if (T9 >= 1)TimeaddC ++;
      if (T10 >= 1)TimeaddC ++;
      if (T11 >= 1)TimeaddC ++;
      if (T12 >= 1)TimeaddC ++;
      if (T13 >= 1)TimeaddC ++;
      if (T14 >= 1)TimeaddC ++;

      DebugSerial.print("TimeaddC: ");
      DebugSerial.println(TimeaddC);

      RunningState = 1;

      TIMR = false;
      TEMPR = false;

      if (SpeedSet == 1) {
        RampTime = 1;
      }
      if (SpeedSet == 2) {
        RampTime = 2;
      }
      if (SpeedSet == 3) {
        RampTime = 4;
      }

      MAXTemp = max(Temp14, max(Temp13, max(Temp12, max(Temp11, max(Temp10, max(Temp9, max(Temp8, max(Temp7, max(Temp6, max(Temp5, max(Temp4, max(Temp3, max(Temp2, Temp1)))))))))))));
      Serial.print("Maximum value is: ");
      Serial.println(MAXTemp);

      TotalRampTime = (MAXTemp - 20) / RampTime;
      DebugSerial.print("Total Ramp Time left: ");
      DebugSerial.println(TotalRampTime);

      TimeSum = T1 + T2 + T3 + T4 + T5 + T6 + T7 + T8 + T9 + T10 + T11 + T12 + T13 + T14;

      ToTimeLeft = currentMillis;

      if (thermocouple.readCelsius() + ThermoCorrection1 >= 0 && thermocouple1.readCelsius() + ThermoCorrection2 >= 0 && thermocouple2.readCelsius() + ThermoCorrection2 >= 0 && thermocouple3.readCelsius() + ThermoCorrection3 >= 0)
      {
        ThermoCheck = 1;
      }

      if (thermocouple.readCelsius() + ThermoCorrection <= 0 || thermocouple1.readCelsius() + ThermoCorrection1 <= 0 || thermocouple2.readCelsius() + ThermoCorrection2 <= 0 || thermocouple3.readCelsius() + ThermoCorrection3 <= 0)// can be subsituded for a else statment
      {
        ThermoCheck = 0;
        sendCmd("RunningState.txt=\"Thermocouple Problem\""); //Send to Nextion the running state
        myFile.close();

      }

      DebugSerial.println("There is a Thermocouple problem");

    }


    if (RunningState == 1 && ThermoCheck == 1 && SpeedSet != 0) {
      DebugSerial.println("Heating proces in Progress");

      if (currentMillis - WriteSDDelta >= 5000 && check == true) {

        DebugSerial.print("Writing Data to SD Card");
        DebugSerial.println(Name);

        myFile.print(thermocouple4.readCelsius() + ThermoCorrection4);
        myFile.print(":");
        myFile.print(thermocouple5.readCelsius() + ThermoCorrection5);
        myFile.print(":");
        myFile.print(thermocouple6.readCelsius() + ThermoCorrection6);
        myFile.print(":");
        myFile.println(thermocouple7.readCelsius() + ThermoCorrection7);

        WriteSDDelta = currentMillis;
      }

      /*if(RunningAverageTemperature>=120)
        TempHeatUpState = 14;
        sendCmd("RunningState.txt=\"OVERHEATING\""); //Send to Nextion the OVERHEATING state
      */

      sendCmd("RunningState.txt=\"Running\""); //Send to Nextion the running state

      switch (TempHeatUpState)
      {

        case 0:
          SetTemp = Temp1;
          Time = T1;
          Currentstate = 1;
          break;

        case 1:
          SetTemp = Temp2;
          Time = T2;
          Currentstate = 2;
          break;

        case 2:
          SetTemp = Temp3;
          Time = T3;
          Currentstate = 3;
          break;

        case 3:
          SetTemp = Temp4;
          Time = T4;
          Currentstate = 4;
          break;

        case 4:
          SetTemp = Temp5;
          Time = T5;
          Currentstate = 5;
          break;

        case 5:
          SetTemp = Temp6;
          Time = T6;
          Currentstate = 6;
          break;

        case 6:
          SetTemp = Temp7;
          Time = T7;
          Currentstate = 7;
          break;

        case 7:
          SetTemp = Temp8;
          Time = T8;
          Currentstate = 8;
          break;

        case 8:
          SetTemp = Temp9;
          Time = T9;
          Currentstate = 9;
          break;

        case 9:
          SetTemp = Temp10;
          Time = T10;
          Currentstate = 10;
          break;

        case 10:
          SetTemp = Temp11;
          Time = T11;
          Currentstate = 11;
          break;

        case 11:
          SetTemp = Temp12;
          Time = T12;
          Currentstate = 12;
          break;

        case 12:
          SetTemp = Temp13;
          Time = T13;
          Currentstate = 13;
          break;

        case 13:
          SetTemp = Temp14;
          Time = T14;
          Currentstate = 14;
          break;

        case 14:
          DebugSerial.println("Done with the Curing Cycle");
          sendCmd("RunningState.txt=\"Finished\"");
          SetTemp = 0;
          Setpoint = 0;
          myFile.println("Finished cycle");
          myFile.close(); // if There is a file open it will close it
          sendCmd("page Finished"); // Opens Finished Page
          RunningState ++;
          break;

        case 15:
          sendCmd("RunningState.txt=\"Done\""); //Send to Nextion the running state
          TIMR = false;
          TEMPR = false;
          SetTemp = 0;
          Setpoint = 0;
          RunningState = 0;
          TempHeatUpState = 0;
          sendCmd("TLN.val=0");
          sendCmd("NTSN.val=0");
          sendCmd("CSN.val=0");
          sendCmd("page1.SelcSet.txt =\"\"");
          sendCmd("page3.SelcSet.txt =\"\"");
          Currentstate = 0;
          deltaTime = 0;
          return;
          break;
      }

      DebugSerial.print("Switch state: ");
      DebugSerial.println(TempHeatUpState);

      DebugSerial.println("Case Status: ");
      DebugSerial.print("Case Set Temperture: ");
      DebugSerial.println(SetTemp);
      DebugSerial.print("Case Set Time: ");
      DebugSerial.println(Time);
      DebugSerial.print("Setpoint: ");
      DebugSerial.println(Setpoint);
//((thermocouple.readCelsius() + ThermoCorrection) + (thermocouple1.readCelsius() + ThermoCorrection1) + (thermocouple2.readCelsius() + ThermoCorrection2) + (thermocouple3.readCelsius() + ThermoCorrection3) / 4) >= SetTemp + 12 || 
      if ((((thermocouple4.readCelsius() + ThermoCorrection4) + (thermocouple5.readCelsius() + ThermoCorrection5) + (thermocouple6.readCelsius() + ThermoCorrection6) + (thermocouple7.readCelsius() + ThermoCorrection7)) / 4) >= SetTemp - 1) Setpoint = SetTemp;

      //if(((thermocouple.readCelsius()+ThermoCorrection)||(thermocouple1.readCelsius()+ThermoCorrection1)||(thermocouple2.readCelsius()+ThermoCorrection2)||(thermocouple3.readCelsius()+ThermoCorrection3) >= SetTemp) Setpoint = SetTemp;

      else   Setpoint = SetTemp + 6;

      if ((((thermocouple4.readCelsius() + ThermoCorrection4) + (thermocouple5.readCelsius() + ThermoCorrection5) + (thermocouple6.readCelsius() + ThermoCorrection6) + (thermocouple7.readCelsius() + ThermoCorrection7)) / 4) >= SetTemp - 3) {
        if (TTS == false)
        {
          Setpoint = SetTemp;
          deltaTime = currentMillis;
          TTS = true;
        }


        // if((unsigned long)(currentMillis - deltaTime) >= (Time*MultiplierTime*1000))

        if ((unsigned long)(currentMillis - deltaTime) >= (Time * MultiplierTime * 1000))
        {
          TempHeatUpState++;
          TTS = false;
        }

      }

      if (Time >= 0 && TTS == true)
      {
        TL = (Time - ((currentMillis - deltaTime) / 60000));
        if (TL != TLF) // Only send time left data when the TL changes
        {
          sendCmd("TLN.val=" + TL);
          TLF = TL;
        }
        DebugSerial.print("Time Left till Next stage: ");
        DebugSerial.println(TL);
        /*
                TAC = (TimeaddC * 30) + (T1+T2+T3+T4+T5+T6+T7+T8+T9+T10+T11+T12+T13+T14);
                DebugSerial.print("Total Time est: ");
                DebugSerial.println(TAC); */
      }

      else
      {
        sendCmd("TLN.val=0");
      }

    }
    if (SpeedSet == 1) {
      RampTime = 1;
      if (Setpoint >= 110) myPID.setOutputRange(0, 35);
      if (Setpoint >= 100) myPID.setOutputRange(0, 25);
      if (Setpoint >= 90) myPID.setOutputRange(0, 20);
      if (Setpoint >= 70) myPID.setOutputRange(0, 15);
      else myPID.setOutputRange(0, 10);
    }
    if (SpeedSet == 2) {
      RampTime = 2;
      if (Setpoint >= 110) myPID.setOutputRange(0, 40);
      if (Setpoint >= 100) myPID.setOutputRange(0, 35);
      if (Setpoint >= 90) myPID.setOutputRange(0, 30);
      if (Setpoint >= 70) myPID.setOutputRange(0, 25);
      else myPID.setOutputRange(0, 20);
    }
    if (SpeedSet == 3) {
      RampTime = 4;
      if (Setpoint >= 110) myPID.setOutputRange(0, 45);
      if (Setpoint >= 100) myPID.setOutputRange(0, 40);
      if (Setpoint >= 90) myPID.setOutputRange(0, 35);
      if (Setpoint >= 70) myPID.setOutputRange(0, 30);
      else myPID.setOutputRange(0, 25);
    }

    else {
      DebugSerial.println("No SpeedSet Selected");
    }

    temperature = RunningAverageTemperature;
    myPID.run();
    analogWrite(OUTPUT_PINHTR1, outputVal);
    analogWrite(OUTPUT_PINHTR2, outputVal);
    analogWrite(OUTPUT_PINHTR3, outputVal);

    DebugSerial.print("outputVal: ");
    DebugSerial.println(outputVal);

    delay(200);

    //if(((thermocouple1.readCelsius()+thermocouple2.readCelsius())/2)>= (SetTemp + 0,25*SetTemp)); {//may cause problems!!!
    //  outputVal = 0;
    //}
    message = RunningAverageTemperature;
    TempvalueAVR = int(RunningAverageTemperature);
    Tempvalue =  int(thermocouple.readCelsius() + ThermoCorrection);
    Tempvalue1 = int(thermocouple1.readCelsius() + ThermoCorrection1);
    Tempvalue2 = int(thermocouple2.readCelsius() + ThermoCorrection2);
    Tempvalue3 = int(thermocouple3.readCelsius() + ThermoCorrection3);
    Tempvalue4 = thermocouple4.readCelsius() + ThermoCorrection4;
    Tempvalue5 = thermocouple5.readCelsius() + ThermoCorrection5;
    Tempvalue6 = thermocouple6.readCelsius() + ThermoCorrection6;
    Tempvalue7 = thermocouple7.readCelsius() + ThermoCorrection7;

    Tempval4 = "\"" + Tempvalue4 + "\"";
    Tempval5 = "\"" + Tempvalue5 + "\"";
    Tempval6 = "\"" + Tempvalue6 + "\"";
    Tempval7 = "\"" + Tempvalue7 + "\"";
    NTS = SetTemp;
    CS = Currentstate;

    mytext = "\"" + message + "\"";
    //myvalue = "\"" + Tempvalue + "\"";

    if (SendData == true && currentMillis - SendTemp >= 1500)
    {
      TimeLeftRun = (TotalRampTime + TimeSum) - ((currentMillis - ToTimeLeft) / 60000);

      if (TimeLeftRun <= 0) TimeLeftRun = 0;

      DebugSerial.print("Total Time Left: ");
      DebugSerial.println(TimeLeftRun);

      DebugSerial.println("Writing data to Nextion");
      sendCmd("LiveTemp.txt=" + mytext);
      sendCmd("TempverAVR.val=" + TempvalueAVR);
      sendCmd("Tempver.val=" + Tempvalue);
      sendCmd("Tempver1.val=" + Tempvalue1);
      sendCmd("Tempver2.val=" + Tempvalue2);
      sendCmd("Tempver3.val=" + Tempvalue3);
      sendCmd("Tempver4.txt=" + Tempval4);// The graph can only handle 4 channels
      sendCmd("Tempver5.txt=" + Tempval5);
      sendCmd("Tempver6.txt=" + Tempval6);
      sendCmd("Tempver7.txt=" + Tempval7);
      sendCmd("NTSN.val=" + NTS);
      sendCmd("CSN.val=" + CS);

      sendCmd("TotalTL.val=" + TimeLeftRun);// Send Total time left

      sendCmd("SendCheck.val=1");//Send to the Nextion that new data can be send

      SendTemp = currentMillis;

      readString = ""; //clears variable for new input
      SendData = false;
      sendCmd(""); // clear the buffer
    }
    readString = ""; //clears variable for new input
  }
}
