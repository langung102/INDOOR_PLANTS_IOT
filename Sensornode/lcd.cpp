#include "HardwareSerial.h"
#include "stdint.h"
#include "lcd.h"

int lcdColumns = 16;
int lcdRows = 2;

uint8_t customChar[] = {
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F
};

LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);  

void setup_lcd() {
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
}

void print_lcd() {
  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("T:");
  if (temp_value > 1000) {
    lcd.print("ERR");
  } else {
    lcd.print(temp_value);
  }
  lcd.setCursor(5, 0);
  lcd.print(" H:");
  if (humi_value > 1000) {
    lcd.print("ERR");
  } else {
    lcd.print(humi_value);
  }
  lcd.setCursor(11, 0);
  lcd.print(" W:");
  if (distance_value <= maxCapacity) {
    uint8_t tmp = (uint8_t)floor((distance_value*100/maxCapacity)/17)+1;
    
    Serial.print("index: ");
    Serial.println(tmp);

    for (int i=6; i>=1; i--) {
      if (i<=tmp) {
        customChar[i] = 0x11;
      } else {
        customChar[i] = 0x1F;
      }
    }
    lcd.createChar(0, customChar);
    lcd.setCursor(14, 0); 
    lcd.write(0);
  }

  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.print(soil_value);
  lcd.setCursor(5, 1);
  lcd.print(" L:");
  lcd.print(light_value);

  delay(500);
}