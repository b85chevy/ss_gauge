// ***************************************** //
// Chevrolet SS Sedan Gauge Cluster
// Revision -
// November 2, 2021
// ***************************************** //

#include <EEPROM.h>
#include <math.h>

// display defines
#include "Goldelox_Serial_4DLib.h"
#include "Goldelox_Const4D.h"

// Boost pressure gauge display defines
#define boostpressure_resetPin 4 // Digital output pin for display reset
#define boostpressureDisplaySerial Serial1  // serial port 1
#include "boostpressure_gaugeConst.h"
Goldelox_Serial_4DLib boostpressureDisplay(&boostpressureDisplaySerial);

// Fuel pressure gauge display defines
#define fuelpressure_resetPin 5 // Digital output pin for display reset
#define fuelpressureDisplaySerial Serial2  // serial port 2
#include "fuelpressure_gaugeConst.h"
Goldelox_Serial_4DLib fuelpressureDisplay(&fuelpressureDisplaySerial);

// Oil pressure gauge display defines
#define oilpressure_resetPin 6 // Digital output pin for display reset
#define oilpressureDisplaySerial Serial3  // serial port 3
#include "oilpressure_gaugeConst.h"
Goldelox_Serial_4DLib oilpressureDisplay(&oilpressureDisplaySerial);

// serial can and obdii defines
// 11-bit can bus, obdii
// utilizes soft serial
#include <Serial_CAN_Module.h>
#include <SoftwareSerial.h>

Serial_CAN can;

#define can_tx                      11          // tx of serial can module connect to digital pin
#define can_rx                      10          // rx of serial can module connect to digital pin
#define CAN_ID_PID                  0x7DF       // can broadcast address

#define PID_ABS_BARO_PRESSURE       0x33
#define PID_MANIFOLD_ABS_PRESSURE   0x0B
#define PID_FUEL_PRESSURE           0x0A
#define PID_OIL_PRESSURE_A          0x70        // first byte of 0x1470 or 5232
#define PID_OIL_PRESSURE_B          0x14        // second byte of 0x1470 or 5232

#define CAN_SERVICE_01              0x01        // 0x01
#define CAN_SERVICE_22              0x22        // 0x22
#define CAN_BITS_02                 0x02        // additional data bits
#define CAN_BITS_03                 0x03        // additional data bits
#define CAN_BITS_04                 0x04        // additional data bits
#define CAN_BITS_06                 0x06        // additional data bits

unsigned char getPID = 0;
unsigned char numPID = 0;

// send delay
unsigned long sendTime = 0;                     // last time the sequence was entered
unsigned long send_historyTime = 0;             // previous time the sequence was entered
unsigned long sendDelay = 100;                  // delay between send PID
unsigned long receiveTime = 0;                  // last time the sequence was entered
unsigned long receive_historyTime = 0;          // previous time the sequence was entered
unsigned long receiveDelay = 50;                // delay between receive PID

// MCP2515 mask and filter configuration
// 111 1111 1111 = 0x7FF
// mask: 111 1111 1111 00 0000 0000 0000 0000 -or- 0x7FF
// filter: 111 1110 1000 00 0000 0000 0000 0000 -or- 0x7E8
// above settings will only accept data from the 0x7E8 identifier

unsigned long mask[4] =
{
  0, 0x7FF,                // ext, maks 0   // set which bits are checked against the filters
  0, 0x7FF,                // ext, mask 1
};

unsigned long filt[12] =
{
  0, 0x7E8,                // ext, filt 0     // set the filter criteria
  0, 0x7E8,                // ext, filt 1
  0, 0x7E8,                // ext, filt 2
  0, 0x7E8,                // ext, filt 3
  0, 0x7E8,                // ext, filt 4
  0, 0x7E8,                // ext, filt 5
};

// ********** define inputs and outputs ********** //
// boost pressure gauge
int boostpressurePin = A0;                      // select the input pin for the potentiometer
int boostpressureValue = 0;                     // store the value coming from the sensor
float boostpressure_filterAlpha = 0.1;          // initialize Exponential Moving Average (EMA) alpha, low alpha is slow/more filter and high alpha is fast/less filter
float boostpressure_filterValue = 0;            // initialize EMA S filtered value
float boostpressure_voltage = 0;                // store voltage
float boostpressure_psi = 0;                    // convert to psi using manufacturer transfer function
int boostpressureScaled = 0;                    // scaled boost for display inpus
int boostpressureHistory = 0;                   // store previous data

// obdii baro pressure sensor data
unsigned int absbaro_pressure_raw = 0;
float absbaro_pressure_kpa = 101.325;           // sea level

// obdii map pressure sensor data
unsigned int map_pressure_raw = 0;
float map_pressure_filterAlpha = 0.1;           // initialize Exponential Moving Average (EMA) alpha, low alpha is slow/more filter and high alpha is fast/less filter
float map_pressure_filterValue = 0;             // initialize EMA S filtered value
float map_pressure_kpa = 0;
float mapboost_pressure_kpa = 0;
float mapboost_pressure_psi = 0;                // boost pressure calculated from map
int mapboostpressureScaled = 0;                 // scaled boost pressure for display inpus
int mapboostpressureHistory = 0;                // store previous data

// obdii fuel pressure sensor data
unsigned int fuel_pressure_raw = 0;
float fuel_pressure_filterAlpha = 0.1;          // initialize Exponential Moving Average (EMA) alpha, low alpha is slow/more filter and high alpha is fast/less filter
float fuel_pressure_filterValue = 0;            // initialize EMA S filtered value
float fuel_pressure_kpa = 0;
float fuel_pressure_psi = 0;
int fuelpressureScaled = 0;                     // scaled fuel for display inpus
int fuelpressureHistory = 0;                    // store previous data

// obdii oil pressure sensor data
unsigned int oil_pressure_raw = 0;
float oil_pressure_filterAlpha = 0.1;           // initialize Exponential Moving Average (EMA) alpha, low alpha is slow/more filter and high alpha is fast/less filter
float oil_pressure_filterValue = 0;             // initialize EMA S filtered value
float oil_pressure_kpa = 0;
float oil_pressure_psi = 0;
int oilpressureScaled = 0;                      // scaled fuel for display inpus
int oilpressureHistory = 0;                     // store previous data

// general
float rampMultiplier= 0.5;                      // multiplier to adjust ramp speed
int initialboostRead = 0;                       // store first gauge read
int initialfuelRead = 0;                        // store first gauge read
int initialoilRead = 0;                         // store first gauge read
int testToggle = 0;                             // test data toggle
int boostsensorType = 1;                        // 0 = analog and 1 = map calculated

// bezel button
int buttonPin = 3;                              // pushbutton connected to digital pin 3
int buttonValue = HIGH;                         // store the value coming from the button
int buttonHistory = HIGH;                       // store the previous button value
int contrastHistory = 15;                       // store the previous contrast level
int contrastDirection = -1;                     // store contrast direction where dim = -1 and bright = 1
unsigned long buttonTime = 0;                   // last time the output pin was toggled
unsigned long debounceDelay = 50;               // debounce time
int contrastHistoryaddr = 0;                    // eeprom address

// fast sequence
unsigned long fastTime = 0;                     // last time the sequence was entered
unsigned long fast_historyTime = 0;             // previous time the sequence was entered
unsigned long fastDelay = 10;                   // delay between entries

// slow sequence
unsigned long slowTime = 0;                     // last time the sequence was entered
unsigned long slow_historyTime = 0;             // previous time the sequence was entered
unsigned long slowDelay = 200;                  // delay between entries

// *********************************************** //

void set_mask_filt()
{
    // set mask
    if(can.setMask(mask)) {
       //Serial.println("set mask ok");
    }
    else {
       //Serial.println("set mask fail");
    }

    // set filter
    if(can.setFilt(filt)) {
       //Serial.println("set filt ok");
    }
    else {
       //Serial.println("set filt fail");
    }
}

void setup()
{
  // start serial debug
  //Serial.begin(9600);  // serial debug window

  // ********** setup and start can bus ********** //
  can.begin(can_tx, can_rx, 57600);      // 9600 does not work
  //Serial.println("initialize can bus");
  // set baudrate of CAN Bus to 500Kb/s
  //if(can.canRate(CAN_RATE_500)) {
  //    Serial.println("set can rate ok");
  //}
  //else {
  //    Serial.println("set can rate fail");
  //}

  set_mask_filt();
  //Serial.println("can bus ready");
  // ********************************************* //

  // ********** setup and start displays ********** //
  pinMode(boostpressure_resetPin, OUTPUT);          // Display reset pin, reset Display simuating open collector or ground/open output, gnd
  pinMode(fuelpressure_resetPin, OUTPUT);           // Display reset pin, reset Display simuating open collector or ground/open output, gnd
  pinMode(oilpressure_resetPin, OUTPUT);            // Display reset pin, reset Display simuating open collector or ground/open output, gnd
  delay(100);                                       // wait for it to be recognised
  pinMode(boostpressure_resetPin, INPUT);           // Display reset pin, reset Display simuating open collector or open/ground output, high impedance                                    // wait for it to be recognised
  pinMode(fuelpressure_resetPin, INPUT);            // Display reset pin, reset Display simuating open collector or open/ground output, high impedance
  pinMode(oilpressure_resetPin, INPUT);             // Display reset pin, reset Display simuating open collector or open/ground output, high impedance

  delay(2000) ;                                     // give display time to startup (This is important)

  // now start display as Serial lines should have 'stabilised'
  // boost pressure display
  boostpressureDisplaySerial.begin(115200) ;        // Hardware serial to Display, same as SPE on display is set to
  boostpressureDisplay.TimeLimit4D = 5000 ;         // 5 second timeout on all commands

  // fuel pressure display
  fuelpressureDisplaySerial.begin(115200) ;         // Hardware serial to Display, same as SPE on display is set to
  fuelpressureDisplay.TimeLimit4D = 5000 ;          // 5 second timeout on all commands

  // oil pressure display
  oilpressureDisplaySerial.begin(115200) ;          // Hardware serial to Display, same as SPE on display is set to
  oilpressureDisplay.TimeLimit4D = 5000 ;           // 5 second timeout on all commands

  delay(1000) ;

  // read display contrast from eerpom and set display contrast
  contrastHistory = EEPROM.read(contrastHistoryaddr);
  boostpressureDisplay.gfx_Contrast(contrastHistory);
  fuelpressureDisplay.gfx_Contrast(contrastHistory);
  oilpressureDisplay.gfx_Contrast(contrastHistory);

  // set other display parameters
  boostpressureDisplay.gfx_ScreenMode(LANDSCAPE) ; // change manually if orientation change
  fuelpressureDisplay.gfx_ScreenMode(LANDSCAPE) ; // change manually if orientation change
  oilpressureDisplay.gfx_ScreenMode(LANDSCAPE) ; // change manually if orientation change

  boostpressureDisplay.SSTimeout(0) ; // 0 disables screen scrolling
  fuelpressureDisplay.SSTimeout(0) ; // 0 disables screen scrolling
  oilpressureDisplay.SSTimeout(0) ; // 0 disables screen scrolling

  if (!boostpressureDisplay.media_Init()) {
    boostpressureDisplay.gfx_Cls();
    boostpressureDisplay.putstr("Drive not mounted...");
    delay(1000);
    boostpressureDisplay.gfx_Cls();
  }
  if (!fuelpressureDisplay.media_Init()) {
    fuelpressureDisplay.gfx_Cls();
    fuelpressureDisplay.putstr("Drive not mounted...");
    delay(1000);
    fuelpressureDisplay.gfx_Cls();
  }
  if (!oilpressureDisplay.media_Init()) {
    oilpressureDisplay.gfx_Cls();
    oilpressureDisplay.putstr("Drive not mounted...");
    delay(1000);
    oilpressureDisplay.gfx_Cls();
  }
  // put your setup code here, to run once:

  // initialize display
  // display logo
  boostpressureDisplay.media_SetAdd(iImage1H, iImage1L) ;     // point to the Image1 Holden logo image
  fuelpressureDisplay.media_SetAdd(iImage1H, iImage1L) ;      // point to the Image1 Holden logo image
  oilpressureDisplay.media_SetAdd(iImage1H, iImage1L) ;       // point to the Image1 Holden logo image
  boostpressureDisplay.media_Image(0, 0) ;                    // show image
  fuelpressureDisplay.media_Image(0, 0) ;                     // show image
  oilpressureDisplay.media_Image(0, 0) ;                      // show image

  delay (2000);

  // clear gauges
  boostpressureDisplay.gfx_Cls();
  fuelpressureDisplay.gfx_Cls();
  oilpressureDisplay.gfx_Cls();

  // displaygauges
  boostpressureDisplay.media_SetAdd(iSmartGauge1H, iSmartGauge1L) ;      // point to the SmartGauge1 boostpressure_gauge image
  boostpressureDisplay.media_VideoFrame(0, 0, 0) ;
  fuelpressureDisplay.media_SetAdd(iSmartGauge2H, iSmartGauge2L) ;       // point to the SmartGauge2 boostpressure_gauge image
  fuelpressureDisplay.media_VideoFrame(0, 0, 0) ;
  oilpressureDisplay.media_SetAdd(iSmartGauge3H, iSmartGauge3L) ;        // point to the SmartGauge2 boostpressure_gauge image
  oilpressureDisplay.media_VideoFrame(0, 0, 0) ;
//***************************************************//

  // initialize gpio
  pinMode(buttonPin, INPUT_PULLUP);    // sets the digital pin as input
}

void sendPID(unsigned char __bits, unsigned char __service, unsigned char __pid_c, unsigned char __pid_b, unsigned char __pid_a)
{
    unsigned char tmp[8] = {__bits, __service, __pid_c, __pid_b, __pid_a, 0, 0, 0};
   //Serial.println("\r\n------------------------------------------------------------------");
   //Serial.print("SEND PID: 0x");
   //Serial.print(__pid_b, HEX);
    if(__pid_a != 0) {
     //Serial.println(__pid_a, HEX);
    }
   //Serial.println();
    can.send(CAN_ID_PID, 0, 0, 8, tmp);   // 11-bit can bus
}

void receivePID(unsigned char (& receive_buffer)[8])
{
    unsigned long id  = 0;
   //Serial.print("receive");
    if(can.recv(&id, receive_buffer))                // check if get data
    {
       Serial.print("Get Data From id: 0x");
       Serial.println(id, HEX);
        for(int i = 0; i < 8; i++)          // print the data
        {
           Serial.print("0x");
           Serial.print(receive_buffer[i], HEX);
           Serial.print("\t");
           //Serial.print(receive_buffer[i], DEC);
           //Serial.print("\t");
        }
       Serial.println();
    }
    /*
    // send test data
    if (testToggle == 0) {
      testToggle = 1;
      // boost and fuel test data only
      receive_buffer [0] = 3;
      receive_buffer [1] = 65;
      receive_buffer [2] = 51;
      receive_buffer [3] = random(101,101);
      receive_buffer [4] = 11;
      receive_buffer [5] = random(140,150);
      receive_buffer [6] = 10;
      receive_buffer [7] = random(140,150);
    }
    else if (testToggle == 1) {
      testToggle = 0;
      // oil test data only
      receive_buffer [0] = 4;
      receive_buffer [1] = 98;
      receive_buffer [2] = 20;
      receive_buffer [3] = 112;
      receive_buffer [4] = random(60,70);
    }
    */
}

void loop()
{
  // put your main code here, to run repeatedly:

  // ********** start of fast sequence ********** //
  fastTime = millis();  // record time sequence is entered, this is used to delay read frequency
  if(fastTime - fast_historyTime > fastDelay) {
    // save the last time the sensor was read
    fast_historyTime = fastTime;

    // ********** read boost sensor and update display ********** //
    // read the sensor value
    boostpressureValue = analogRead(boostpressurePin);
    // calculate the filtered value:
    //boostpressure_filterValue = (boostpressure_filterAlpha * float(boostpressureValue)) + ((1 - boostpressure_filterAlpha) * boostpressure_filterValue);
    // convert to voltage
    boostpressure_voltage = float(boostpressureValue) * (5.0 / 1023.0);
    // convert to psi
    boostpressure_psi = (3.7529 * boostpressure_voltage) - 1.8765;
    // scale sensor value to gauge
    boostpressureScaled = round(float(boostpressure_psi) / (8.0/234.0));  // psi range is 0 to 15, gauge range is 0 to 8 or 234 degrees
    if (boostpressureScaled < 0) {
      boostpressureScaled = 0;
      }
    else if (boostpressureScaled > 234) {
      boostpressureScaled = 234;
      }
    // ********************************************************* //

    // ********** can and obdii operation ********** //
    // ********** send request ********** //
      if (getPID == 0) {
        sendTime = millis();  // record time sequence is entered, this is used to delay read frequency
        if(sendTime - send_historyTime > sendDelay) {
          // save the last time PID request was sent
          send_historyTime = sendTime;
          if (numPID == 0) {
            sendPID(CAN_BITS_04, CAN_SERVICE_01, PID_ABS_BARO_PRESSURE, PID_MANIFOLD_ABS_PRESSURE, PID_FUEL_PRESSURE);
            numPID = 1;
            }
          else if (numPID == 1) {
            sendPID(CAN_BITS_03, CAN_SERVICE_22, PID_OIL_PRESSURE_B, PID_OIL_PRESSURE_A, 0);
            numPID = 0;
          }
          getPID = 1;
        }
      }
      // ********************************************************* //
      // ********** receive request ********** //
      if (getPID == 1) {
        receiveTime = millis();  // record time sequence is entered, this is used to delay read frequency
        if(receiveTime - receive_historyTime > receiveDelay) {
          // save the last time PID request was sent
          receive_historyTime = receiveTime;
          unsigned char receive_buffer[8];
          receivePID(receive_buffer);
          // baro pressure
          if (receive_buffer[2] == PID_ABS_BARO_PRESSURE) {
            absbaro_pressure_raw = receive_buffer[3];
            absbaro_pressure_kpa = float(absbaro_pressure_raw);
            }
          // map pressure and calculate boost pressure
          if (receive_buffer[4] == PID_MANIFOLD_ABS_PRESSURE) {
            map_pressure_raw = receive_buffer[5];
            // calculate the filtered value:
            //map_pressure_filterValue = (map_pressure_filterAlpha * float(map_pressure_raw)) + ((1 - map_pressure_filterAlpha) * map_pressure_filterValue);
            map_pressure_kpa = float(map_pressure_raw);
            // calculate boost pressure
            mapboost_pressure_kpa = float(map_pressure_kpa) - float(absbaro_pressure_kpa);
            mapboost_pressure_psi = mapboost_pressure_kpa/6.8947572932;
            // scale sensor value to gauge
            mapboostpressureScaled = round(float(mapboost_pressure_psi) / (8.0/234.0));  // analog range is 0 to 8, gauge range is 800 degrees
            if (mapboostpressureScaled > 234) {
              mapboostpressureScaled = 234;
            }
            else if (mapboostpressureScaled < 0) {
              mapboostpressureScaled = 0;
            }
            //Serial.print("MAP Boost Pressure: ");
            //Serial.println(absbaro_pressure_kpa);
            //Serial.println(map_pressure_raw);
            //Serial.println(map_pressure_kpa);
            //Serial.println(mapboost_pressure_kpa);
            //Serial.println(mapboost_pressure_psi);
            //Serial.println(mapboostpressureScaled);
            //Serial.print(" PSI ");
          }

          // fuel pressure
          if (receive_buffer[6] == PID_FUEL_PRESSURE) {
            fuel_pressure_raw = receive_buffer[7];
            // calculate the filtered value:
            //fuel_pressure_filterValue = (fuel_pressure_filterAlpha * float(fuel_pressure_raw)) + ((1 - fuel_pressure_filterAlpha) * fuel_pressure_filterValue);
            //fuel_pressure_kpa = float(fuel_pressure_filterValue) * 3.0;
            fuel_pressure_kpa = float(fuel_pressure_raw) * 3.0;
            fuel_pressure_psi = fuel_pressure_kpa/6.8947572932;
            // scale sensor value to gauge
            fuelpressureScaled = round(float(fuel_pressure_psi) * 10.0);  // analog range is 0 to 80, gauge range is 800 degrees
            if (fuelpressureScaled > 800) {
              fuelpressureScaled = 800;
            }
           //Serial.print("Fuel Pressure: ");
           //Serial.print(fuel_pressure_psi, 1);
           //Serial.print(" PSI ");
          }

          // oil pressure
          if (receive_buffer[2] == PID_OIL_PRESSURE_B and receive_buffer[3] == PID_OIL_PRESSURE_A) {
            oil_pressure_raw = receive_buffer[4];
            // calculate the filtered value:
            //oil_pressure_filterValue = (oil_pressure_filterAlpha * float(oil_pressure_raw)) + ((1 - oil_pressure_filterAlpha) * oil_pressure_filterValue);
            oil_pressure_psi = float(oil_pressure_raw) * 0.578;
            // scale sensor value to gauge
            oilpressureScaled = round(float(oil_pressure_psi) * 10.0);  // analog range is 0 to 80, gauge range is 800 degrees
            if (oilpressureScaled > 800) {
              oilpressureScaled = 800;
              }
            //Serial.print("Oil Pressure: ");
            //Serial.print(oil_pressure_psi, 1);
            //Serial.print(" PSI ");
            //Serial.println("Oil Pressure scaled: ");
            //Serial.print(oilpressureScaled, 1);
            //Serial.print(" PSI ");
          }
        }
        getPID = 0;
      }

    // force data to be correct boost gauge type, analog or calculated from map
    if (boostsensorType == 1) {
      boostpressureScaled = mapboostpressureScaled;
    }
    // ********** ramp gauges ********** //
    // initial ramp boost gauge
    if (initialboostRead == 0 and boostpressureScaled > 0) {
      initialboostRead = 1;
      for (boostpressureHistory = 1; boostpressureHistory < boostpressureScaled; boostpressureHistory = boostpressureHistory + round((boostpressureScaled - boostpressureHistory) * rampMultiplier)) {
        boostpressureDisplay.media_VideoFrame(0, 0, boostpressureHistory) ;
        }
      }

    // initial ramp fuel gauge
    if (initialfuelRead == 0 and fuelpressureScaled > 0) {
      initialfuelRead = 1;
      for (fuelpressureHistory = 1; fuelpressureHistory < fuelpressureScaled; fuelpressureHistory = fuelpressureHistory + round((fuelpressureScaled - fuelpressureHistory) * rampMultiplier)) {
        fuelpressureDisplay.media_VideoFrame(0, 0, fuelpressureHistory) ;
        }
      }

    // initial ramp oil gauge
    if (initialoilRead == 0 and oilpressureScaled > 0) {
      initialoilRead = 1;
      for (oilpressureHistory = 1; oilpressureHistory < oilpressureScaled; oilpressureHistory = oilpressureHistory + round((oilpressureScaled - oilpressureHistory) * rampMultiplier)) {
        oilpressureDisplay.media_VideoFrame(0, 0, oilpressureHistory) ;
        }
      }
    // ********************************************************* //
    // ********** update gauges ********** //
    if (initialboostRead == 1) {
      // smooth transition boost gauge
      boostpressureHistory = round((boostpressure_filterAlpha * float(boostpressureScaled)) + ((1 - boostpressure_filterAlpha) * boostpressureHistory));
      boostpressureDisplay.media_VideoFrame(0, 0, boostpressureHistory);
      }
    if (initialfuelRead == 1) {
      // smooth transition fuel gauge
      fuelpressureHistory = round((fuel_pressure_filterAlpha * float(fuelpressureScaled)) + ((1 - fuel_pressure_filterAlpha) * fuelpressureHistory));
      fuelpressureDisplay.media_VideoFrame(0, 0, fuelpressureHistory) ;
      }
    if (initialoilRead == 1) {
      // smooth transition oil gauge
      oilpressureHistory = round((oil_pressure_filterAlpha * float(oilpressureScaled)) + ((1 - oil_pressure_filterAlpha) * oilpressureHistory));
      oilpressureDisplay.media_VideoFrame(0, 0, oilpressureHistory) ;
    }
    // ********************************************* //

  }
  // ********** end of fast sequence ********** //

  // ********** start of slow sequence ********** //
  slowTime = millis();  // record time sequence is entered, this is used to delay read frequency
  if(slowTime - slow_historyTime > slowDelay) {
    // save the last time the sensor was read
    slow_historyTime = slowTime;

    // ********** bezel button dimming operation ********** //
    buttonValue = digitalRead(buttonPin);   // read the input pin
    if (buttonValue != buttonHistory) {
      // reset the debouncing timer
      buttonTime = millis();
      }

    if ((millis() - buttonTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

        // only toggle if the new button state is LOW
        if (buttonValue == LOW) {
          if ((contrastHistory > 0) && (contrastDirection == -1)) {
            contrastHistory = contrastHistory + 1 * contrastDirection;
            }
          else if ((contrastHistory < 15) && (contrastDirection == 1)) {
            contrastHistory = contrastHistory + 1 * contrastDirection;
            }
          else {
            contrastDirection = contrastDirection * -1;
            contrastHistory = contrastHistory + 1 * contrastDirection;
            }
          boostpressureDisplay.gfx_Contrast(contrastHistory);
          // correct brightness of replacement fuel display
          if ((contrastHistory != 0) && (contrastHistory != 15)) {
              fuelpressureDisplay.gfx_Contrast(contrastHistory + 1);
            }
          else if ((contrastHistory >= 1) && (contrastHistory <= 4)) {
              fuelpressureDisplay.gfx_Contrast(contrastHistory + 1);
            }
          else {
             fuelpressureDisplay.gfx_Contrast(contrastHistory);
             }
          oilpressureDisplay.gfx_Contrast(contrastHistory);
          EEPROM.write(contrastHistoryaddr, contrastHistory);
        }
    }
    buttonHistory = buttonValue;
  // **************************************************** //
  }
  // ********** end of slow sequence ********** //
}
