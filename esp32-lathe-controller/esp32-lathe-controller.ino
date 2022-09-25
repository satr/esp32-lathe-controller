#include<LiquidCrystal_I2C_Hangul.h>
#include<Wire.h>
#include <pwmWrite.h>

LiquidCrystal_I2C_Hangul lcd(0x27,16,2); 

int sensorPin = 34;    // select the input pin for the potentiometer
int ledPin = LED_BUILTIN;      // select the pin for the LED
int pwmOut = 14;
int rpmRegulatorValue = 0;
int rpmLastRegulatorValue = 0;
int brightness = 0;
int step = 1;

Pwm pwm = Pwm();

void setup() {
  Serial.begin(115200);
  // pinMode(ledPin, OUTPUT);
  lcd.init();
  lcd.backlight();
  displayRpm();
  pwm.writeResolution(pwmOut, 10);
  pwm.write(pwmOut, 341);
  pwm.writeFrequency(pwmOut, 100);
  readRegulatorValue();
}

void loop() {
  readRegulatorValue();

  if(abs(rpmLastRegulatorValue - rpmRegulatorValue) > 1) {
    rpmLastRegulatorValue = rpmRegulatorValue;
    pwm.writeFrequency(pwmOut, rpmRegulatorValue);
    displayRpm();
  }
  delay(50);
}

void readRegulatorValue() {
  rpmRegulatorValue = analogRead(sensorPin) / 4;
}

void displayRpm() {
  lcd.setCursor(0, 0);
  lcd.printf("RPM:%-4d", rpmRegulatorValue);
}
