#include "lcd.h"

int lcdColumns = 16;
int lcdRows = 2;

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
  lcd.print(temp_value);
  lcd.setCursor(5, 0);
  lcd.print(" H:");
  lcd.print(humi_value);
  lcd.setCursor(11, 0);
  lcd.print(" W:");
  lcd.print("OK");

  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.print(soil_value);
  lcd.setCursor(5, 1);
  lcd.print(" L:");
  lcd.print(light_value);

  delay(500);
}