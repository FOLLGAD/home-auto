
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup_lcd() {
  lcd.begin(16, 2);
  char motd[] = "Hello Mycocrib!";
  lcd.print(motd);
}

void setup() {
    Serial.begin(9600);
    setup_lcd();
    setup_bme280();
    pinMode(LED_BUILTIN, OUTPUT);
}

void setup_bme280() {
    while(!Serial);
    Serial.println(F("BME280 test"));

    unsigned status;
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 1000;

    Serial.println();
}




struct TempVals {
  float temp;
  float pressure;
  float alt;
  float humidity;
};

struct TempVals getValues() {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure();
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  struct TempVals t = {
    temp, pressure, alt, humidity
  };
  return t;
}

void loop() {
    TempVals t = getValues();

    lcd.setCursor(0, 1);
    lcd.print(String(t.temp, 1));
    lcd.print("C. ");
    lcd.print(String(t.humidity, 1));
    lcd.print("%");

    Serial.println("HERE");

    delay(1000);
    
    manual_periph_ctrl(0);
    Deep_Sleep_Manual();
}



void resetWatchdog (){
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset, clear existing interrupt
  WDTCSR = bit (WDCE) | bit (WDE) | bit (WDIF);
  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  // pat the dog
  wdt_reset();  
}  // end of resetWatchdog

void i2c_switch_off(){
  // turn off I2C
  TWCR &= ~(bit(TWEN) | bit(TWIE) | bit(TWEA));

  // turn off I2C pull-ups
  digitalWrite (A4, LOW);
  digitalWrite (A5, LOW);
}


void manual_periph_ctrl(uint8_t selector){
  byte old_ADCSRA = ADCSRA;
  // disable ADC
  ADCSRA = 0; 
   
  if (selector == 0){
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();
    power_twi_disable();

    UCSR0B &= ~bit (RXEN0);  // disable receiver
    UCSR0B &= ~bit (TXEN0);  // disable transmitter
  } 
  if (selector >= 1){
    power_adc_enable();
    power_spi_enable();
    power_timer0_enable();
    power_timer1_enable();
    power_timer2_enable();
    power_twi_enable();
    
    UCSR0B |= bit (RXEN0);  // enable receiver
    UCSR0B |= bit (TXEN0);  // enable transmitter
  } 

  ADCSRA = old_ADCSRA;
}

void Deep_Sleep_Manual(){
  // CORE State
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;            // turn off ADC
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface

  // Interrupts are not counted as ADXL require
  noInterrupts ();       // timed sequence coming up
  resetWatchdog ();      // get watchdog ready
  sleep_enable ();       // ready to sleep
  // interrupts ();         // interrupts are required now
  sleep_cpu ();          // sleep                
  // sleep_disable ();      // precaution
  // power_all_enable ();   // power everything back on
}
