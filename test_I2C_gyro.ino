
#define HML5883L 0x1E
#define L3GD20 0b01101010

#include <LiquidCrystal.h>
#include <Wire.h>

LiquidCrystal lcd1(11, 10, 9, 8, 7, 6, 5, 4, 3, 2);
LiquidCrystal lcd2(11, 12, 9, 8, 7, 6, 5, 4, 3, 2);

extern void i2c_write(byte dev ,byte reg);
extern void i2c_write(byte dev ,byte reg, byte data);
extern byte i2c_read(byte dev);
extern byte i2c_read(byte dev ,byte reg);

void i2c_write(byte dev ,byte reg){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(true);
}

void i2c_write(byte dev ,byte reg, byte data){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

byte i2c_read(byte dev){
  Wire.requestFrom(dev, 1, true);
  byte val = Wire.read();
  return val;
}

byte i2c_read(byte dev ,byte reg){
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(dev, 1, true);
  byte val = Wire.read();
  return val;
}

double deg = 0.00875 / 20;

double gyro_ofst_x = 0;
double gyro_ofst_y = 0;
double gyro_ofst_z = 0;

void setup() {
  Wire.begin();
  lcd1.begin(40, 2);
  lcd2.begin(40, 2);
  lcd1.clear();
  lcd2.clear();

  // gyro
  lcd1.setCursor(0, 0);
  lcd1.print(i2c_read(L3GD20, 0x0F));
  lcd1.setCursor(0, 1);
  lcd1.print(i2c_read(HML5883L, 0x0A));
  
  i2c_write(L3GD20, 0x20, 0b10111111);
  //i2c_write(L3GD20, 0x20, 0b11001111);
  //i2c_write(L3GD20, 0x21, 0b00001001);
  i2c_write(L3GD20, 0x21, 0b00000000);
  i2c_write(L3GD20, 0x22, 0b00000000);
  i2c_write(L3GD20, 0x23, 0b00000000);
  i2c_write(L3GD20, 0x24, 0b00000000);
  i2c_write(L3GD20, 0x25, 0b00000000);
  
  //lcd1.print(50 - i2c_read(L3GD20, 0x26));

  // compass
  i2c_write(HML5883L, 0x00, 0b01110000);
  i2c_write(HML5883L, 0x01, 0b00100000);
  i2c_write(HML5883L, 0x02, 0b00000000);

  delay(500);

  gyro_ofst_x = -311.41;
  gyro_ofst_y = -71;
  gyro_ofst_z = 34.12;
  
  double gyro_ofst_x2 = 0;
  double gyro_ofst_y2 = 0;
  double gyro_ofst_z2 = 0;

  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;

  int times = 1000;
  
  for(int i = 0; i < times; i++){
    gyro_x = (i2c_read(L3GD20, 0x29) << 8) | i2c_read(L3GD20, 0x28);
    gyro_y = (i2c_read(L3GD20, 0x2B) << 8) | i2c_read(L3GD20, 0x2A);
    gyro_z = (i2c_read(L3GD20, 0x2D) << 8) | i2c_read(L3GD20, 0x2C);
    
    gyro_ofst_x2 += gyro_x;
    gyro_ofst_y2 += gyro_y;
    gyro_ofst_z2 += gyro_z;
  }
/*
  gyro_ofst_x = (-311.41 - gyro_ofst_x2/times)/2;
  gyro_ofst_y = (-71 - gyro_ofst_y2/times)/2;
  gyro_ofst_z = (34.12 - gyro_ofst_z2/times)/2;
  */

  gyro_ofst_x = - gyro_ofst_x2/times;
  gyro_ofst_y = - gyro_ofst_y2/times;
  gyro_ofst_z = - gyro_ofst_z2/times;
}

double ofs = 300;


void loop() {

  // gyro
  
  int16_t gyro_x = (i2c_read(L3GD20, 0x29) << 8) | i2c_read(L3GD20, 0x28);
  int16_t gyro_y = (i2c_read(L3GD20, 0x2B) << 8) | i2c_read(L3GD20, 0x2A);
  int16_t gyro_z = (i2c_read(L3GD20, 0x2D) << 8) | i2c_read(L3GD20, 0x2C);

  static double static_gyro_x = 0;
  static double static_gyro_y = 0;
  static double static_gyro_z = 0;
  
  
  if(abs(gyro_ofst_x + gyro_x) < ofs && abs(gyro_ofst_y + gyro_y) < ofs && abs(gyro_ofst_z + gyro_z) < ofs)  {
    gyro_ofst_x = (gyro_ofst_x * 9999 - gyro_x) / 10000;
    gyro_ofst_y = (gyro_ofst_y * 9999 - gyro_y) / 10000;
    gyro_ofst_z = (gyro_ofst_z * 9999 - gyro_z) / 10000;
  }
  
  static_gyro_x += gyro_x + gyro_ofst_x;
  static_gyro_y += gyro_y + gyro_ofst_y;
  static_gyro_z += gyro_z + gyro_ofst_z;
  
  
  lcd1.setCursor(0, 0);
  lcd1.print("X: ");
  lcd1.print(gyro_x);
  lcd1.setCursor(10, 0);
  //lcd1.print(static_gyro_x);
  lcd1.print((double)static_gyro_x * deg);
  
  lcd1.setCursor(20, 0);
  lcd1.print("Y: ");
  lcd1.print(gyro_y);
  lcd1.setCursor(30, 0);
  //lcd1.print(static_gyro_y);
  lcd1.print((double)static_gyro_y * deg);
  
  lcd1.setCursor(0, 1);
  lcd1.print("Z: ");
  lcd1.print(gyro_z);
  lcd1.setCursor(10, 1);
  //lcd1.print(static_gyro_z);
  lcd1.print((double)static_gyro_z * deg);
  
  lcd1.setCursor(16, 1);
  //lcd1.print(i2c_read(L3GD20, 0x27),BIN);
  
  lcd1.print(gyro_ofst_x);
  lcd1.print(",");
  lcd1.print(gyro_ofst_y);
  lcd1.print(",");
  lcd1.print(gyro_ofst_z);
  
  lcd1.setCursor(35, 1);


  // compass

  
  i2c_write(HML5883L, 0x02,0b00000001);
  
  int16_t cmps_x = (i2c_read(HML5883L, 0x03) << 8) | i2c_read(HML5883L, 0x04);
  int16_t cmps_y = (i2c_read(HML5883L, 0x07) << 8) | i2c_read(HML5883L, 0x08);
  int16_t cmps_z = (i2c_read(HML5883L, 0x05) << 8) | i2c_read(HML5883L, 0x06);
  
  double dbl_cmps_x = cmps_x;
  double dbl_cmps_y = cmps_y;
  double dbl_cmps_z = cmps_z;


  lcd2.setCursor(0, 0);
  lcd2.print("X: ");
  lcd2.print(cmps_x);
  
  lcd2.setCursor(20, 0);
  lcd2.print("Y: ");
  lcd2.print(cmps_y);
  
  lcd2.setCursor(0, 1);
  lcd2.print("Z: ");
  lcd2.print(cmps_z);
  
  lcd2.setCursor(10, 1);
  lcd2.print(sqrt(dbl_cmps_x * dbl_cmps_x + dbl_cmps_y * dbl_cmps_y + dbl_cmps_z * dbl_cmps_z));
  
  lcd2.setCursor(20, 1);
  int a = atan(dbl_cmps_y/dbl_cmps_x)/PI*360;

  if(a < -157)    lcd2.print("S");
  else if(a < -112)          lcd2.print("SW");
  else if(a < -67)          lcd2.print("W");
  else if(a < -22)          lcd2.print("NW");
  else if(a < 23)          lcd2.print("N");
  else if(a < 68)    lcd2.print("NE");
  else if(a < 113)    lcd2.print("E");
  else if(a < 158)    lcd2.print("SE");
  else if(a < 180)    lcd2.print("S");
  
  lcd2.print(a);
  
  lcd2.setCursor(30, 1);
  lcd2.print(atan(dbl_cmps_z/sqrt(dbl_cmps_x * dbl_cmps_x + dbl_cmps_y * dbl_cmps_y))/PI*360);

  
  lcd2.setCursor(36, 0);
  lcd2.print(50 - (millis() % 50));
  
  delay(50 - (millis() % 50));
  lcd1.clear();
  lcd2.clear();
}



