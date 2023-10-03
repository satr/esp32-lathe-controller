//Library "LiquidCrystal_I2C_Hangul"  https://github.com/junwha0511/LiquidCrystal_I2C_Hangul
#include<LiquidCrystal_I2C_Hangul.h>
#include<Wire.h>
//Library "ESP32 ESP32S2 AnalogWrite" https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
#include <pwmWrite.h>

//First get LCD I2C address (here "0x27") with a scketch from https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
LiquidCrystal_I2C_Hangul lcd(0x27,16,2); 

int pulseRegulator = 34;         // select the input pin for the potentiometer
int ledPin = LED_BUILTIN;      // select the pin for the LED
int servoMotorSpeed = 14;      // select the pin for the servo motor speed control

float pulseRegulatorValue = 0;
float pulseLastRegulatorValue = 0;
int pulsesPerRevolution = 800;

Pwm pwm = Pwm();

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  lcd.init();
  lcd.backlight();

  pwm.writeResolution(servoMotorSpeed, 10);
  pwm.write(servoMotorSpeed, 341);
  pwm.writeFrequency(servoMotorSpeed, 100);

  displayPulseAndRpm();
  applyRegulatorValue();
}

void loop() {
  applyRegulatorValue();

  delay(50);
}

void applyRegulatorValue() {
  pulseRegulatorValue = analogRead(pulseRegulator) * 10; //the analog value is in range 0 - 4096
                                                    //the pulseRegulatorValue is in range 0 - 40960
  if(abs(pulseLastRegulatorValue - pulseRegulatorValue) > 1) { //only update with change above noise
    pulseLastRegulatorValue = pulseRegulatorValue;
    pwm.writeFrequency(servoMotorSpeed, pulseRegulatorValue);
    displayPulseAndRpm();
  }
}

void displayPulseAndRpm() {
  lcd.setCursor(0, 0);
  lcd.printf("Pulse:%-5g RPM:%-4.0f", pulseRegulatorValue, pulseRegulatorValue / 12);
}
