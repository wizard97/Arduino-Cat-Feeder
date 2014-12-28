/* 


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>


Aaron Wisner's Arduino program for controlling peltier cooled automatic cat feeder. Two doors controlled 
 by servos open and close at user set times. A temperature sensor controlls state of peltier cooler, which 
 keeps wet food from spoiling. Program requires a TMP36 temp sensor to monitor temperature inside cat feeder 
 along with a mosfet to control the peltier cooler and the fan that cools the heatsink on the peltier via a pwm single from the AVR.
 This program will not run on an arduino using an ATMEGA168, due to ram limitations.
 ****in the code indicate varaibles the user should adjust to fit their needs*******
 written on 6/25/13 By: Aaron Wisner
 
 
 Please note the required libraries
 
 
 */

//necessary librarys
#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>
#include <Wire.h> 
#include <Servo.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
Servo feed1Servo;
Servo feed2Servo;

//assigning devices to pins
int scaledpeltierPower = 5;  //peltier mosfet connected to pin 5
int sensePin = A0;  //temp36 temp sensor center pin hooked to analog pin 0
int alarm = 8;  //feeding alarm on pin 8
int lcdMode = A1; //keyboard mode button
int alarmset = 6; //keyboard button
int timeadjust = 11; //keyboard button
int servoPin1 = 9;  //servo for 1st feed on pin 9
int servoPin2 = 10;  //servo for second feed on pin 10
LiquidCrystal lcd(13, 12, 7, 4, 3, 2);

//variables
int peltierPower = 255;  //adjusts peltier power duty cycle between 0-255 (8 bits of resolution)
const float arefvoltage = 1.1;
const int samples = 50;
float peltierpercent = 0;
float lasttemp = 0;
int alarmsetnumber = 0;
const int starlocation[] = {
  3, 9, 3, 9}; //array for inserting arrow for adjusting alarm times
int servo1close = 1; //***degrees servo1 close***
int servo2close = 171; //***degress servo2 close***
int servo1open = 92; //***degrees servo 1 open***
int servo2open = 80; //***degrees servo 2 open***
int dooropencounter = 0;
int alarmsetnumberminus = 3;
int buttonpresscounter = 0;
int cycletime = 0;
int delay1days = 0;
int delay2days = 0;
int delayrow = 0;
int settempF = 37;
boolean previousbuttonstate = false;
boolean lastbuttonstate = true;
boolean timechange1open = false;
boolean timechange1close = false;
boolean timechange2open = false;
boolean timechange2close = false;
boolean feed1enable = true;
boolean feed2enable = true;
boolean delaychangeday1 = false;
boolean delaychangeday2 = false;
boolean door1fired = false;
boolean door2fired = false;
boolean tempchange = false;
unsigned long milliseconds = 0;

//assign alarm ID's
AlarmID_t feed1openalarm = 0;
AlarmID_t feed1closealarm = 1;
AlarmID_t feed2openalarm = 2;
AlarmID_t feed2closealarm = 3;


//feed1 Start
int feed1openhour = 12;
int feed1openmin = 30;

//feed1 end
int feed1closehour = 13;
int feed1closemin = 0;

//feed2 Start
int feed2openhour = 18;
int feed2openmin = 45;

//feed2 end
int feed2closehour = 19;
int feed2closemin = 15;

byte arrowchar[8] = {
  B10000,
  B11000,
  B11100,
  B11110,
  B11110,
  B11100,
  B11000,
  B10000
};



void setup()
{
  //serial monitor
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.createChar(0, arrowchar);
  lcd.setCursor(0, 0);
  Serial.println("Succesfully established connection at 9600 baud");
  lcd.print("Welcome to the");
  lcd.setCursor(0, 1);  
  lcd.print("Kitty Feeder 2K");
  Alarm.delay(2000);
  lcd.clear();
  lcd.print("  Created By:");
  lcd.setCursor(0, 1);
  lcd.print("  Aaron Wisner  ");
  Alarm.delay(2000);
  lcd.clear();
  Serial.println("Attempting to sync time");
  lcd.print("Syncing Time...");
  lcd.setCursor(0, 1);
  setSyncProvider(RTC.get); // get time from RTC
  Alarm.delay(1000);
  if(timeStatus()== timeSet) 
  {
    Serial.println("Succesfully updated system time from RTC");
    lcd.print("  Time Synced");
  }
  else
  {
    Serial.println("Unable to sync with the RTC, resetting to defaults");
    lcd.clear();
    lcd.print("Unable to Sync");
    lcd.setCursor(0, 1);
    lcd.print("Defaulting");
    setSyncInterval(86400);
  }


  //assigning pin mode
  pinMode(scaledpeltierPower, OUTPUT);
  pinMode(alarm, OUTPUT);
  pinMode(lcdMode, INPUT);
  pinMode(alarmset, INPUT);
  pinMode(timeadjust, INPUT);
  analogReference(INTERNAL);
  feed1Servo.attach(servoPin1);
  feed2Servo.attach(servoPin2);
  feed1Servo.write(servo1close); 
  feed2Servo.write(servo2close); 

  //setting alarms
  eepromreadalarms();

  feed1openhour = constrain(feed1openhour, 0, 23);
  feed2openhour = constrain(feed2openhour, 0, 23);
  feed1closehour = constrain(feed1closehour, 0, 23);
  feed2closehour = constrain(feed2closehour, 0, 23);
  feed1openmin = constrain(feed1openmin, 0, 59);
  feed2openmin = constrain(feed2openmin, 0, 59);
  feed1closemin = constrain(feed1closemin, 0, 59);
  feed2closemin = constrain(feed2closemin, 0, 59);

  time_t feed1opentime = AlarmHMS(feed1openhour,feed1openmin,0);
  time_t feed1closetime = AlarmHMS(feed1closehour,feed1closemin,0);
  time_t feed2opentime = AlarmHMS(feed2openhour,feed2openmin,0);
  time_t feed2closetime = AlarmHMS(feed2closehour,feed2closemin,0);  

  AlarmID_t feed1openalarm = Alarm.alarmRepeat(feed1opentime, feed1Open);  //setting up event handler
  AlarmID_t feed1closealarm = Alarm.alarmRepeat(feed1closetime, feed1Close);  
  AlarmID_t feed2openalarm =  Alarm.alarmRepeat(feed2opentime, feed2Open);  
  AlarmID_t feed2closealarm = Alarm.alarmRepeat(feed2closetime, feed2Close);
  Alarm.timerRepeat(3, serialmonitor);

}




//looped system tasks
void  loop()
{
  peltierCooler(); //monitor refrigeration status
  lcdmonitor(); //render lcd display
  Alarm.delay(0);  //service alarms

}




// feeding functions called by alarm event handler
void feed1Open()
{
  if ((feed1enable) && (delay1days == 0))
  {
    Serial.println("Feed 1 initaited");
    lcd.clear();
    lcd.print("Feed 1 Initiated");
    lcd.setCursor(0,1);
    lcd.print("  Enjoy! -___-");
    digitalWrite(alarm, HIGH);
    opendoor1();
    Alarm.delay(5000);  //****alarm time needed to get you pets attention****
    digitalWrite(alarm, LOW);
    door1fired = true;
  }
  else if ((delay1days >= 1) && (feed1enable))
  {
    delay1days--;
    door1fired = false;
    delaychangeday1 = true;
    setsavealarms();
  }
  else if (delay1days == 0 && !feed1enable)
  {
    lcd.clear();
    lcd.print("Feed 1 is Off");
    lcd.setCursor(0,1);
    lcd.print("Re-enable!");
    door1fired = false;
    Alarm.delay(2000);
  }


}

void feed1Close()
{
  if ((feed1enable) && door1fired)
  {
    digitalWrite(alarm, HIGH);
    lcd.clear();
    lcd.print("Feed 1 Complete");
    lcd.setCursor(0,1);
    lcd.print("   Times Up!");
    Alarm.delay(3000);  //****alarm time needed to get you pets attention****
    digitalWrite(alarm, LOW);
    closedoor1();
    Serial.println("Feed 1 complete");  
    feed1enable = false;
    door1fired = false;
  } 
  else if (delay1days == 0 && !feed1enable)
  {
    lcd.clear();
    lcd.print("Feed 1 is Off");
    lcd.setCursor(0,1);
    lcd.print("Re-enable!");
    door1fired = false;
    Alarm.delay(2000);
  }

}

void feed2Open()
{
  if ((feed2enable) && (delay2days == 0))
  {
    Serial.println("Feed 2 initated");
    lcd.clear();
    lcd.print("Feed 2 Initiated"); 
    lcd.setCursor(0,1);
    lcd.print("  Enjoy! -___-");
    digitalWrite(alarm, HIGH);
    opendoor2();
    Alarm.delay(5000);  //****alarm time needed to get you pets attention****
    digitalWrite(alarm, LOW);
    door2fired = true;
  }
  else if ((delay2days >= 1) && (feed2enable))
  {
    delay2days--;
    door2fired = false;
    delaychangeday2 = true;
    setsavealarms();
  }
  else if (delay2days == 0 && !feed2enable)
  {
    lcd.clear();
    lcd.print("Feed 2 is Off");
    lcd.setCursor(0,1);
    lcd.print("Re-enable!");
    door2fired = false;
    Alarm.delay(2000);
  }

}

void feed2Close()
{
  if ((feed2enable) && door2fired)
  {
    digitalWrite(alarm, HIGH);
    lcd.clear();
    lcd.print("Feed 2 Complete");
    lcd.setCursor(0,1);
    lcd.print("   Times Up!");
    Alarm.delay(3000);  //****alarm time needed to get you pets attention****
    digitalWrite(alarm, LOW);
    closedoor2();
    Serial.println("Feed 2 complete");
    feed2enable = false;
    door2fired = false;
  }
  else if (delay2days == 0 && !feed2enable)
  {
    lcd.clear();
    lcd.print("Feed 2 is Off");
    lcd.setCursor(0,1);
    lcd.print("Re-enable!");
    door2fired = false;
    Alarm.delay(2000);
  }
}  

void peltierCooler()  //function for controlling peltier pwm  
{
  float deltaT = (getTemp() - float(settempF));  //****set target temp here****
  if (deltaT <= -.5 || settempF >= 51 || (!feed1enable && !feed2enable))
  {
    peltierPower = 0;
  }
  else if (deltaT > -0.5 && (deltaT <= 1.0))  //****set the deltaT you want to pwm peltier****
  {
    peltierPower = map(int(100*deltaT), -50, 100, 50, 255);
  }  //****set your pwm min and max out of 255****
  else {
    peltierPower = 255;
  }
  analogWrite(scaledpeltierPower, peltierPower);
  peltierpercent = (peltierPower*100/255);
  Alarm.delay(0);
}

void lcdmonitor()
{
  Alarm.delay(0);
  if (digitalRead(lcdMode) || digitalRead(timeadjust) || digitalRead(alarmset))
  {
    buttonpresscounter++;
  }
  else buttonpresscounter = 0;

  if (buttonpresscounter >= 3)
  {
    Alarm.delay(0);
    if (digitalRead(timeadjust) && digitalRead(alarmset) && !digitalRead(lcdMode))
    {
      manualdooropen();
    }


    else if (digitalRead(lcdMode) && !digitalRead(timeadjust) && !digitalRead(alarmset))
    {
      Alarm.delay(0);
      lcd.clear();
      lcd.print(" Please Select");
      lcd.setCursor(0,1);
      lcd.print("Open/Close Times");
      Alarm.delay(2000);
      lcd.clear();
      lcd.print("#1 Open #1 Close");
      lcd.setCursor(0,1);
      lcd.print("#2 Open #2 Close");
      Alarm.delay(2000);
      lcd.clear();
      alarmsetnumber = 0;
      milliseconds = millis();
      page2printline1();
      page2printline2();
      Alarm.delay(0);

      while ((milliseconds + 60000) >= millis() && alarmsetnumber <= 3)
      {

        arrowlocation(); 
        Alarm.delay(50);
        while (alarmsetnumber == 0 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          Alarm.delay(0);
          if (digitalRead(timeadjust))
          {
            feed1openmin = feed1openmin + 15;
            if (feed1openmin >= 60)
            {
              feed1openmin = 0;
              ++feed1openhour;
            }
            if (feed1openhour >= 24)
            {
              feed1openhour = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            feed1openmin = feed1openmin - 15;
            if (feed1openmin < 0)
            {
              feed1openmin = 45;
              --feed1openhour;
            }
            if (feed1openhour < 0)
            {
              feed1openhour = 24;
            } 
          }
          page2printline1();
          arrowlocation();
          timechange1open = true; 
          Alarm.delay(200);
        }

        while (alarmsetnumber == 1 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          if (digitalRead(timeadjust))
          {
            feed1closemin = feed1closemin + 15;
            if (feed1closemin >= 60)
            {
              feed1closemin = 0;
              ++feed1closehour;
            }
            if (feed1closehour >= 24)
            {
              feed1closehour = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            feed1closemin = feed1closemin - 15;
            if (feed1closemin < 0)
            {
              feed1closemin = 45;
              --feed1closehour;
            }
            if (feed1closehour < 0)
            {
              feed1closehour = 24;
            } 
          }
          page2printline1();
          arrowlocation();
          timechange1close = true;
          Alarm.delay(200);
        }

        while (alarmsetnumber == 2 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          Alarm.delay(0);
          if (digitalRead(timeadjust))
          {
            feed2openmin = feed2openmin + 15;
            if (feed2openmin >= 60)
            {
              feed2openmin = 0;
              ++feed2openhour;
            }
            if (feed2openhour >= 24)
            {
              feed2openhour = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            feed2openmin = feed2openmin - 15;
            if (feed2openmin < 0)
            {
              feed2openmin = 45;
              --feed2openhour;
            }
            if (feed2openhour < 0)
            {
              feed2openhour = 24;
            } 
          }
          page2printline2();
          arrowlocation();
          timechange2open = true;
          Alarm.delay(200);
        }

        while (alarmsetnumber == 3 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          Alarm.delay(0);
          if (digitalRead(timeadjust))
          {
            feed2closemin = feed2closemin + 15;
            if (feed2closemin >= 60)
            {
              feed2closemin = 0;
              ++feed2closehour;
            }
            if (feed2closehour >= 24)
            {
              feed2closehour = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            feed2closemin = feed2closemin - 15;
            if (feed2closemin < 0)
            {
              feed2closemin = 45;
              --feed2closehour;
            }
            if (feed2closehour < 0)
            {
              feed2closehour = 24;
            } 
          }

          page2printline2();
          arrowlocation();
          timechange2close = true;
          Alarm.delay(200);
        }
      }
      alarmsetnumber = 0;
      setsavealarms();
    }

    else if (digitalRead(alarmset) && !digitalRead(timeadjust) && !digitalRead(lcdMode))
    {
      Alarm.delay(0);
      lcd.clear();
      lcd.print(" Please Select");
      lcd.setCursor(0,1);
      lcd.print("On/Off & Delays");
      Alarm.delay(2000);


      lcd.clear();
      lcd.print("#1: ");
      if (feed1enable)
      {
        lcd.print("On  ");
      }
      else lcd.print("Off ");
      lcd.print(delay1days);
      lcd.print(" days");
      lcd.setCursor(0,1);
      lcd.print("#2: ");
      if (feed2enable)
      {
        lcd.print("On  ");
      }
      else lcd.print("Off ");
      lcd.print(delay2days);
      lcd.print(" days");
      Alarm.delay(0);


      lastbuttonstate = true;
      milliseconds = millis();
      while (((milliseconds + 60000) >= millis()) && delayrow <= 3)
      {
        arrowdelaylocation();
        Alarm.delay(50);

        while (delayrow == 0 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          if (feed1enable)
          {
            feed1enable = false;
          }
          else if (!feed1enable)
          {
            feed1enable = true;
          }
          lcd.clear();
          lcd.print("#1: ");
          if (feed1enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay1days);
          lcd.print(" days");
          lcd.setCursor(0,1);
          lcd.print("#2: ");
          if (feed2enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay2days);
          lcd.print(" days");
          Alarm.delay(200);

        }

        while (delayrow == 1 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          Alarm.delay(0);
          if (digitalRead(timeadjust))
          {
            delay1days++;
            if (delay1days > 5)
            {
              delay1days = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            delay1days--;
            if (delay1days < 0)
            {
              delay1days = 5;
            }  
          }
          lcd.clear();
          lcd.print("#1: ");
          if (feed1enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay1days);
          lcd.print(" days");
          lcd.setCursor(0,1);
          lcd.print("#2: ");
          if (feed2enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay2days);
          lcd.print(" days");
          Alarm.delay(0);

          arrowdelaylocation();
          delaychangeday1 = true;
          Alarm.delay(200);
        }

        while (delayrow == 2 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          if (feed2enable)
          {
            feed2enable = false;
          }
          else if (!feed2enable)
          {
            feed2enable = true;
          }
          lcd.clear();
          lcd.print("#1: ");
          if (feed1enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay1days);
          lcd.print(" days");
          lcd.setCursor(0,1);
          lcd.print("#2: ");
          if (feed2enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay2days);
          lcd.print(" days");
          Alarm.delay(200);

        }



        while (delayrow == 3 && (digitalRead(timeadjust) == HIGH || digitalRead(lcdMode)))
        {
          Alarm.delay(0);
          if (digitalRead(timeadjust))
          {
            delay2days++;
            if (delay2days > 5)
            {
              delay2days = 0;
            }
          }
          else if (digitalRead(lcdMode))
          {
            delay2days--;
            if (delay2days < 0)
            {
              delay2days = 5;
            }  
          }
          lcd.clear();
          lcd.print("#1: ");
          if (feed1enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay1days);
          lcd.print(" days");
          lcd.setCursor(0,1);
          lcd.print("#2: ");
          if (feed2enable)
          {
            lcd.print("On  ");
          }
          else lcd.print("Off ");
          lcd.print(delay2days);
          lcd.print(" days");
          Alarm.delay(0);

          arrowdelaylocation();
          delaychangeday2 = true;
          Alarm.delay(200);
        }
      }
      delayrow = 0;
      setsavealarms();
      Alarm.delay(0);
    }


    else if (!digitalRead(alarmset) && digitalRead(timeadjust) && !digitalRead(lcdMode))
    {
      Alarm.delay(0);
      lcd.clear();
      lcd.print(" Please Select");
      lcd.setCursor(0,1);
      lcd.print("Temperature");
      Alarm.delay(2000);
      lcd.clear();
      lcd.print("Set Temp: ");
      if (settempF <= 50)
      {
        lcd.print(settempF);
        lcd.write(B11011111);
        lcd.print("F");
      }
      else
      {
        lcd.print("Off");
      }
      milliseconds = millis();
      while ((milliseconds + 60000) >= millis())
      {
        Alarm.delay(0);
        lcd.clear();
        lcd.print("Set Temp: ");
        if (settempF <= 50)
        {
          lcd.print(settempF);
          lcd.write(B11011111);
          lcd.print("F");
        }
        else
        {
          lcd.print("Off");
        }

        if (digitalRead(timeadjust) && !previousbuttonstate)
        {
          settempF++;
          if (settempF > 51)
          {
            settempF = 51;
          }
          tempchange = true;
        }
        else if (digitalRead(lcdMode) && !previousbuttonstate)
        {
          settempF--;
          if (settempF < 32)
          {
            settempF = 32;
          }
          tempchange = true;
        }
        if (digitalRead(lcdMode) == HIGH || digitalRead(timeadjust) == HIGH)
        {
          previousbuttonstate = true;
        }
        else {
          previousbuttonstate = false;
        }
        if (digitalRead(alarmset)) break;

        Alarm.delay(100);
      }
      setsavealarms();
      Alarm.delay(0);

    }
    Alarm.delay(0);
  }



  if (cycletime <= 3)
  {
    Alarm.delay(0);
    lcddigitalclockdisplay();
  }
  else if (cycletime <= 6)
  {
    Alarm.delay(0);
    lcd.clear();
    page2printline1();
    page2printline2();
  }
  else if (cycletime <= 9)
  {
    Alarm.delay(0);
    lcd.clear();
    lcd.print("#1: ");
    if (feed1enable)
    {
      lcd.print("On  ");
    }
    else lcd.print("Off  ");
    lcd.print("Delay:");
    lcd.print(delay1days);
    lcd.setCursor(0, 1);
    lcd.print("#2: ");
    if (feed2enable)
    {
      lcd.print("On  ");
    }
    else lcd.print("Off  ");
    lcd.print("Delay:");
    lcd.print(delay2days);
  }
  else if (cycletime <= 12)
  {
    Alarm.delay(0);
    lcd.clear();
    lcd.print("System Uptime:");
    lcd.setCursor(0, 1);
    milliseconds = millis();
    lcd.print(milliseconds/1000);
    lcd.print(" Secs");
  }

  cycletime++;
  if (cycletime >= 13)
  {
    cycletime = 0;
  }
  Alarm.delay(0);

}

void serialmonitor()
{
  Alarm.delay(0);
  serialdigitalClockDisplay();
  Serial.print("Current Temp (F): ");
  Serial.print(lasttemp);
  Serial.println();
  if (peltierPower > 0)
  {
    Serial.print("Peltier cooler is running at ");
    Serial.print(peltierpercent);
    Serial.print("%");
    Serial.println();
  }
  else {
    Serial.println("Peltier cooler is off");
  }
}    


float getTemp()  //function for retrieving temp in degrees farenheight 
{
  Alarm.delay(0);
  unsigned int total = 0;
  for (int i = 0; i < samples; i++) 
  {
    total = total + analogRead(sensePin);
    Alarm.delay(1000/samples);
  }
  float average = total/samples;
  float scaledvoltage = average*arefvoltage;
  float voltage = scaledvoltage/1024.0; 
  float temperatureC = (voltage - 0.5)*100.0;
  float temperatureF = temperatureC*(9.0/5.0) + 32.0;
  lasttemp = temperatureF;
  return temperatureF;
}

//system stats
void serialdigitalClockDisplay()  //function for writing system time to serial monitor
{
  Alarm.delay(0);
  // digital clock display of the time
  Serial.print("Current System Time: ");
  Serial.print(hour());
  serialprintDigits(minute());
  serialprintDigits(second());
  Serial.println();
}

void lcddigitalclockdisplay()
{
  Alarm.delay(0);
  lcd.clear();
  lcd.print("Time: ");
  lcd.print(hour());
  lcdprintDigits(minute());
  lcdprintDigits(second());
  lcd.setCursor(0, 1);
  lcd.print(round(lasttemp));
  lcd.write(B11011111); 
  lcd.print("F ");
  lcd.print("Cooler:");
  lcd.print(round(peltierpercent));
  lcd.write(B00100101);
}

void serialprintDigits(int digits)  //function for deciding whether to use place marking zeros
{
  Alarm.delay(0);
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void lcdprintDigits(int digits)
{
  Alarm.delay(0);
  // utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}


void page2printline1()
{
  Alarm.delay(0);
  lcd.setCursor(0,0);
  lcd.write(B00100011);
  lcd.print("1: ");
  if (feed1openhour < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed1openhour);
  lcd.print(":");
  if (feed1openmin < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed1openmin);
  lcd.print(" ");
  if (feed1closehour < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed1closehour);
  lcd.print(":");
  if (feed1closemin < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed1closemin);
}

void page2printline2() 
{ 
  Alarm.delay(0);
  lcd.setCursor(0, 1);
  lcd.write(B00100011);
  lcd.print("2: ");
  if (feed2openhour < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed2openhour);
  lcd.print(":");
  if (feed2openmin < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed2openmin);
  lcd.print(" ");
  if (feed2closehour < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed2closehour);
  lcd.print(":");
  if (feed2closemin < 10)
  {
    lcd.print("0");
  }
  lcd.print(feed2closemin);
}

void arrowlocation()
{
  Alarm.delay(0);

  if (alarmsetnumber == 0 || alarmsetnumber == 1)
  {
    lcd.setCursor(starlocation[alarmsetnumber], 0);
    lcd.write(byte(0));
  }
  if (alarmsetnumber == 2 || alarmsetnumber == 3)
  {
    lcd.setCursor(starlocation[alarmsetnumber], 1);
    lcd.write(byte(0));
  }

  alarmsetnumberminus = alarmsetnumber - 1;
  if (alarmsetnumberminus <= -1)
  {
    alarmsetnumberminus = 3;
  }
  if (alarmsetnumberminus == 0 || alarmsetnumberminus == 1)
  {
    lcd.setCursor(starlocation[alarmsetnumberminus], 0);
    lcd.print(" ");
  }
  if (alarmsetnumberminus == 2 || alarmsetnumberminus == 3)
  {
    lcd.setCursor(starlocation[alarmsetnumberminus], 1);
    lcd.print(" ");
  }
  Alarm.delay(0);
  if (digitalRead(alarmset) == HIGH && previousbuttonstate == LOW)
  {
    alarmsetnumber = alarmsetnumber + 1;
  }

  previousbuttonstate = digitalRead(alarmset);
}



void arrowdelaylocation()
{
  Alarm.delay(0);
  if (delayrow == 0)
  {
    lcd.setCursor(3, 0);
    lcd.write(byte(0));
    lcd.setCursor(7, 1);
    lcd.print(" ");
  }
  else if (delayrow == 1)
  {
    lcd.setCursor(7, 0);
    lcd.write(byte(0));
    lcd.setCursor(3, 0);
    lcd.print(" ");
  }

  else if (delayrow == 2)
  {
    lcd.setCursor(3, 1);
    lcd.write(byte(0));
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }

  else if (delayrow == 3)
  {
    lcd.setCursor(7, 1);
    lcd.write(byte(0));
    lcd.setCursor(3, 1);
    lcd.print(" ");
  }
  Alarm.delay(0);
  if (digitalRead(alarmset) == HIGH && lastbuttonstate == LOW)
  {
    delayrow = delayrow + 1;
  }

  lastbuttonstate = digitalRead(alarmset);
  Alarm.delay(0);
}


void setsavealarms() //writes data to EEPROM
{
  if (timechange1open)
  {
    time_t newfeed1opentime = AlarmHMS(feed1openhour,feed1openmin,0);
    Alarm.free(feed1openalarm);
    AlarmID_t feed1openalarm = Alarm.alarmRepeat(newfeed1opentime, feed1Open);  
    Alarm.enable(feed1openalarm);
    Alarm.delay(0);
    EEPROM.write(0, feed1openhour);
    EEPROM.write(1, feed1openmin);
    timechange1open = false;

  }

  if (timechange1close)
  {
    Alarm.delay(0);
    time_t newfeed1closetime = AlarmHMS(feed1closehour,feed1closemin,0);
    Alarm.free(feed1closealarm); 
    AlarmID_t feed1closealarm = Alarm.alarmRepeat(newfeed1closetime, feed1Close); 
    Alarm.enable(feed1closealarm);
    Alarm.delay(0);
    EEPROM.write(2, feed1closehour);
    EEPROM.write(3, feed1closemin);
    timechange1close = false;
  }

  if (timechange2open)
  {
    Alarm.delay(0);

    time_t newfeed2opentime = AlarmHMS(feed2openhour,feed2openmin,0);
    Alarm.free(feed2openalarm);  
    AlarmID_t feed2openalarm =  Alarm.alarmRepeat(newfeed2opentime, feed2Open);  
    Alarm.enable(feed2openalarm);
    Alarm.delay(0);
    EEPROM.write(4, feed2openhour);
    EEPROM.write(5, feed2openmin);
    timechange2open = false;
  }

  if (timechange2close)
  {
    Alarm.delay(0);

    time_t newfeed2closetime = AlarmHMS(feed2closehour,feed2closemin,0);
    Alarm.free(feed2closealarm);
    AlarmID_t feed2closealarm = Alarm.alarmRepeat(newfeed2closetime, feed2Close);
    Alarm.enable(feed2closealarm);
    Alarm.delay(0);
    EEPROM.write(6, feed2closehour);
    EEPROM.write(7, feed2closemin);
    timechange2close = false;
  }

  if (delaychangeday1)
  { 
    Alarm.delay(0);
    EEPROM.write(8, delay1days);
    delaychangeday1 = false;
  }

  if (delaychangeday2)
  {
    Alarm.delay(0);
    EEPROM.write(9, delay2days);
    delaychangeday2 = false;

  }

  if (tempchange)
  {
    Alarm.delay(0);
    EEPROM.write(10, settempF);
    tempchange = false;
  }
  Alarm.delay(0);
}


void eepromreadalarms()
{
  Alarm.delay(0);
  feed1openhour = EEPROM.read(0);
  feed1openmin = EEPROM.read(1);
  feed1closehour = EEPROM.read(2);
  feed1closemin = EEPROM.read(3);
  feed2openhour = EEPROM.read(4);
  Alarm.delay(0);
  feed2openmin = EEPROM.read(5);
  feed2closehour = EEPROM.read(6);
  feed2closemin = EEPROM.read(7);
  delay1days = EEPROM.read(8);
  Alarm.delay(0);
  delay2days = EEPROM.read(9);
  settempF = EEPROM.read(10);
  Alarm.delay(0);
}


void manualdooropen()
{
  Alarm.delay(0);
  dooropencounter = 0;
  lcd.clear();
  lcd.print("Hold to Manually");
  lcd.setCursor(0,1);
  lcd.print("Open Doors");
  while (digitalRead(timeadjust) == HIGH && digitalRead(alarmset) == HIGH)
  {
    Alarm.delay(0);
    ++dooropencounter;
    if (dooropencounter >= 30)
    {
      Alarm.delay(0);
      opendoor1();
      opendoor2();

      for (int timeremaining = 15; timeremaining > -1; timeremaining--)
      {        
        lcd.clear();
        lcd.print("    You have");
        lcd.setCursor(0, 1);
        lcd.print("   ");
        lcd.print(timeremaining);
        lcd.print(" Seconds");
        Alarm.delay(1000);
      }
      lcd.clear();
      lcd.print("   Time is Up!");
      closedoor1();
      closedoor2();
    }  
    Alarm.delay(100);
  }
  Alarm.delay(0);
}




void opendoor1() //opens door 1 slowly
{
  for(int i = servo1close; i < servo1open; i++)
  {
    Alarm.delay(0);
    feed1Servo.write(i); 
    Alarm.delay(30);
  }
}

void opendoor2() //opens door two slowly
{
  for(int i = servo2close; i > servo2open; i--)
  {
    Alarm.delay(0);
    feed2Servo.write(i); 
    Alarm.delay(30);
  }
}

void closedoor1() //closes door 1 slowly
{
  for(int i = servo1open; i > servo1close; i--)
  {
    Alarm.delay(0);
    feed1Servo.write(i); 
    Alarm.delay(30);
  }
}

void closedoor2() //closes door 2 slowly
{
  for(int i = servo2open; i < servo2close; i++)
  {
    Alarm.delay(0);
    feed2Servo.write(i); 
    Alarm.delay(30);
  }
}






