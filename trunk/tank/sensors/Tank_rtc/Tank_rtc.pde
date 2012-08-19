#include <Wire.h>//I2C header file

// Defines
//#define DEBUG // Uncomment to turn on verbose mode
#define I2C_RTC 0x68 // 7 bit address (without last bit - look at the datasheet)
#define ERROR_LED 13

// Errors
#define ERROR_RTC_SET 1 // Unable to set RTC time and date
#define ERROR_RTC_GET 2 // Unable to get RTC time and date

// Global variables
byte result;
byte second;
byte second_old; // The code ask the RTC for data only when the previous value has changed
byte minute;
byte minute_old; // The code ask the RTC for data only when the previous value has changed
byte hour;
byte hour_old; // The code ask the RTC for data only when the previous value has changed
byte weekday;
byte day;
byte month;
byte year;
char* weekdayname[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Function prototypes
byte BcdToDec(byte);
byte DecToBcd(byte);
void SetError(int);

void setup()
{
  pinMode(ERROR_LED, OUTPUT); // Set error LED
  Wire.begin(); // Initiate the Wire library and join the I2C bus as a master
  Serial.begin(9600); // Initiate serial communication

// Set initial date and time
  second_old = second = 53; // Second (0-59)
  minute_old = minute = 59; // Minute (0-59)
  hour_old = hour = 23; // Hour (0-23)
  weekday = 1; // Day of the week (1-7)
  day = 31; // Day (1-31)
  month = 12; // Month (1-12)
  year = 11; // Year (0-99)
  Wire.beginTransmission(I2C_RTC); // Select RTC
  Wire.send(0);        // Start address
  Wire.send(DecToBcd(second));     // Second
  Wire.send(DecToBcd(minute));    // Minute
  Wire.send(DecToBcd(hour));    // Hour
  Wire.send(DecToBcd(weekday));    // Weekday
  Wire.send(DecToBcd(day));    // Day
  Wire.send(DecToBcd(month));     // Month (with century bit = 0)
  Wire.send(DecToBcd(year));    // Year
  Wire.send(0b10000);           // Output clock frequency enabled (1 Hz)
  Wire.endTransmission();
  result = Wire.endTransmission();

#ifdef DEBUG
  Serial.print("Result of setting date and time: ");
  Serial.println(result, DEC);
#endif

  if (result) SetError(ERROR_RTC_SET);
}

void loop()
{
  Wire.beginTransmission(I2C_RTC);
  Wire.send(0); // Start address
  result = Wire.endTransmission();
#ifdef DEBUG
  Serial.print("Result of asking for date and time: ");
  Serial.println(result, DEC);
#endif
  if (result) SetError(ERROR_RTC_GET);

  Wire.requestFrom(I2C_RTC, 1);
  second = BcdToDec(Wire.receive());
  if (second != second_old) // Cycle begins only when it has changed
  {
    second_old = second;
    if (second == 0) // If second is zero I need to ask for the minute
    {
      Wire.requestFrom(I2C_RTC, 1);
      minute = BcdToDec(Wire.receive());
      if (minute != minute_old) // Cycle begins only when it has changed
      {
        minute_old = minute;
        if (minute == 0) // If minute is zero I need to ask for the hour
        {
          Wire.requestFrom(I2C_RTC, 1);
          hour = BcdToDec(Wire.receive() & 0b111111);
          if (hour != hour_old) // Cycle begins only when it has changed
          {
            hour_old = hour;
            if (hour == 0) // If hour is zero I need to ask for other elements
            {
              Wire.requestFrom(I2C_RTC, 4);
              weekday = BcdToDec(Wire.receive());
              day = BcdToDec(Wire.receive());
              month = BcdToDec(Wire.receive());
              year = BcdToDec(Wire.receive());
            }
          }
        }
      }
    }
    Serial.print(weekdayname[weekday - 1]);
    Serial.print("(");
    Serial.print(weekday, DEC);
    Serial.print(") 20");
    if (year < 10) Serial.print("0");
    Serial.print(year, DEC);
    Serial.print("-");
    if (month < 10) Serial.print("0");
    Serial.print(month,DEC);
    Serial.print("-");
    if (day < 10) Serial.print("0");
    Serial.print(day, DEC);
    Serial.print(" ");
    if (hour < 10) Serial.print("0");
    Serial.print(hour,DEC);
    Serial.print(":");
    if (minute < 10) Serial.print("0");
    Serial.print(minute, DEC);
    Serial.print(":");
    if (second < 10) Serial.print("0");
    Serial.println(second, DEC);
  }
}

// Converts a BCD (binary coded decimal) to decimal
byte BcdToDec(byte value)
{
  return ((value / 16) * 10 + value % 16);
}

// Converts a decimal to BCD (binary coded decimal)
byte DecToBcd(byte value){
  return (value / 10 * 16 + value % 10);
}

void SetError(int error) // Blinks forever the error led a number of times corresponding to error number
{
  while(1) // Forever
  {
    for (byte index = 0; index < error; index++)
    {
      digitalWrite(ERROR_LED, HIGH);
      delay(500);
      digitalWrite(ERROR_LED, LOW);
      delay(500);
    }
    delay(1000);
  }
}
