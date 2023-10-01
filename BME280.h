// gdanek 2022
// Библиотека для работы с bme280
//GitHub: https://github.com/gdanek
#ifndef BME280_H
#define BME280_H
// Всего одна структура. Больше не надо
class BME280
{
  int16_t calibrateData[12];
  bool reset(void){if(!writeReg(0x0E,0xB6)) return false;_delay_ms(11);return true;}
  void bme280Sizing()
  {
    Wire.beginTransmission(address);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)address,(uint8_t)24);
    for(uint8_t i = 0;i < 12;i++){calibrateData[i] = (Wire.read() | (Wire.read() << 8));}
  }
  bool writeReg(uint8_t reg_addr , uint8_t data) 
  {
    Wire.beginTransmission(address);
    Wire.write(reg_addr);
    Wire.write(data);
    if (Wire.endTransmission() != 0) return false;
    return true;
  }
  uint32_t readByte(uint8_t reg_addr,uint8_t howMany) 
  {
    Wire.beginTransmission(address);
    Wire.write(reg_addr);
    if (Wire.endTransmission() != 0)return 0x800000;
    Wire.requestFrom((uint8_t)address,(uint8_t)howMany);
    if(howMany == 1) return(uint8_t)Wire.read();
    if(howMany == 2)return(((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8)));
    else  return (((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | (uint32_t)Wire.read());
  }
  int32_t read_raw_data(void) 
  {
    uint16_t t1 = (uint16_t)calibrateData[0];
    int32_t _raw = readByte(0xFA,3);                  
    _raw >>= 4;                        
    int32_t value_1 = ((((_raw >> 3) - ((int32_t)t1 << 1))) *
    ((int32_t)calibrateData[1])) >> 0x0B;
    int32_t value_2 = (((((_raw >> 4) - ((int32_t)t1))*((_raw >> 4) - ((int32_t)t1))) >> 0x0C) * ((int32_t)calibrateData[3])) >> 0x0E;
    return ((int32_t)value_1 + value_2);           
  }
  public:
  uint8_t address = 0x76;
  BME280(uint8_t address){this->address=address;}
  bool begin(void) 
  {
    Wire.begin();                                
    reset();           
    bme280Sizing();              
    writeReg(0xF4,((0x03<<5)|(0x02<<2)|0x03));   
    writeReg(0xF5,((0x03<<5)|(0x04<<2)));                    
  }
  float readTemperature(void) 
  {
    int32_t temp = read_raw_data();
    float T = (temp * 5 + 0x80) >> 0x08;
    return T / 100.0;                     
  }
  float readPressure(void) 
  {
    uint16_t p1 = (uint16_t)calibrateData[3];
    uint32_t press_raw = readByte(0xF7,3);                
    if (press_raw == 0x800000) return 0;             
    press_raw >>= 4;                       
    int64_t value_1 = ((int64_t)read_raw_data())-0x1F400;
    int64_t value_2 = value_1*value_1*(int64_t)calibrateData[8];
    value_2 = value_2+((value_1 * (int64_t)calibrateData[7])<<0x11);
    value_2 = value_2+(((int64_t)calibrateData[6])<<0x23);
    value_1 = ((value_1*value_1*(int64_t)calibrateData[5])>>0x08)+((value_1*(int64_t)calibrateData[4])<<0x0C);
    value_1 = (((((int64_t)1)<<0x2F)+value_1))*((int64_t)p1)>>0x21;
    if (!value_1) return 0;                     // Avoid division by zero
    int64_t p = 0x100000 - press_raw;
    p = (((p << 0x1F) - value_2) * 0xC35) / value_1;
    value_1 = (((int64_t)calibrateData[11]) * (p >> 0x0D) * (p >> 0x0D)) >> 0x19;
    value_2 = (((int64_t)calibrateData[10]) * p) >> 0x13;
    p = ((p + value_1 + value_2) >> 0x08) + (((int64_t)calibrateData[9]) << 0x04);
    return (float)p/0x100*0.00750061683f;                     
}
};
#endif
