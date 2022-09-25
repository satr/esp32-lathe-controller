//Library "LiquidCrystal_I2C_Hangul"  https://github.com/junwha0511/LiquidCrystal_I2C_Hangul
#include<LiquidCrystal_I2C_Hangul.h>
#include<Wire.h>
//Library "ESP32 ESP32S2 AnalogWrite" https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
#include <pwmWrite.h>

//First get LCD I2C address (here "0x27") with a scketch from https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
LiquidCrystal_I2C_Hangul lcd(0x27,16,2); 

int rpmRegulator = 34;         // select the input pin for the potentiometer
int ledPin = LED_BUILTIN;      // select the pin for the LED
int servoMotorSpeed = 14;      // select the pin for the servo motor speed control

int rpmRegulatorValue = 0;
int rpmLastRegulatorValue = 0;

Pwm pwm = Pwm();

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  lcd.init();
  lcd.backlight();

  pwm.writeResolution(servoMotorSpeed, 10);
  pwm.write(servoMotorSpeed, 341);
  pwm.writeFrequency(servoMotorSpeed, 100);

  applyRegulatorValue();
}

void loop() {
  applyRegulatorValue();

  delay(50);
}

void applyRegulatorValue() {
  rpmRegulatorValue = analogRead(rpmRegulator) / 4; //the analog value is in range 0 - 4096
                                                    //the rpmRegulatorValue is in range 0 - 1024
  if(abs(rpmLastRegulatorValue - rpmRegulatorValue) > 1) { //only update with change above noise
    rpmLastRegulatorValue = rpmRegulatorValue;
    pwm.writeFrequency(servoMotorSpeed, rpmRegulatorValue);
    displayRpm();
  }
}

void displayRpm() {
  lcd.setCursor(0, 0);
  lcd.printf("RPM:%-4d", rpmRegulatorValue);
}
