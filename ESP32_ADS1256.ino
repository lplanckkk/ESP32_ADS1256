#include <SPI.h>

#define CS_PIN    15
#define MISO_PIN  12
#define MOSI_PIN  13
#define SCK_PIN   14
#define DRDY_PIN  2
#define VREF      2.5

// 调试模式：serial monitor调试置1，工作置0
#define DEBUG_MODE 0

#if DEBUG_MODE
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// 模式定义
enum ReadMode { IDLE, SINGLE, DIFF, CYCLE };
ReadMode currentMode = IDLE;
int selectedChannel = -1;

// 统计变量
unsigned long cycleCount = 0; // 循环计数器
unsigned long startTime = 0;  // 开始时间

// 当前配置：数据速率和PGA寄存器值
uint8_t currentDataRate = 0xF0;   // 默认数据速率
uint8_t currentPGAReg   = 0x26;    // 默认PGA寄存器值
// 根据PGA寄存器低3位映射增益：0->1, 1->2, 2->4, 3->8, 4->16, 5->32, 6->64
float currentGain = 64.0;

// SPI_CMD
const uint8_t CMD_RESET   = 0xFE;
const uint8_t CMD_SYNC    = 0xFC;
const uint8_t CMD_WAKEUP  = 0x00;
const uint8_t CMD_RDATA   = 0x01;

void printHelp() {
  Serial.println("=== Command List ===");
  Serial.println("SINGLE <channel>    - 单端模式采样指定通道 (0-7)");
  Serial.println("DIFF <channel>      - 差分模式采样指定通道对 (0-3，对应0-1,2-3,4-5,6-7)");
  Serial.println("CYCLE               - 循环采样所有单端通道 (0-7)");
  Serial.println("STOP                - 停止连续采样");
  Serial.println("READ <reg>          - 读取寄存器 (十六进制)");
  Serial.println("WRITE <reg> <val>   - 写寄存器 (十六进制)");
  Serial.println("SET DRATE <val>     - 设置数据速率 (十六进制，0x00~0xFF)");
  Serial.println("SET PGA <val>       - 设置PGA寄存器 (十六进制)，同时更新增益");
  Serial.println("RESET               - 复位设备");
  Serial.println("STAT                - 显示当前采样率统计（DPS）");
  Serial.println("HELP                - 显示帮助菜单");
  Serial.println("====================");
}

void setup() {

  resetDevice();

  Serial.begin(2000000);
  pinMode(CS_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_PIN, HIGH); 
  Serial.println("ADS1256 ready for commands.");
  
  setDataRate(currentDataRate);
  setPGA(currentPGAReg);

  startTime = millis();
  printHelp();
}

void loop() {
  // from python
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      processSerialCommand(command);
    }
  }

  // 根据当前模式读取数据
  switch (currentMode) {
    case SINGLE:
      readSingleChannel(selectedChannel);
      break;
    case DIFF:
      readDifferentialChannel(selectedChannel);
      break;
    case CYCLE:
      readAllChannels();
      break;
    case IDLE:
      delay(100);
      break;
  }

  // from serial monitor
  #if DEBUG_MODE
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n'); // 读取输入直到换行符
      processSerialCommand(input); // 处理串口命令
    }
  #endif

  updateSampleRate();
}

bool statMode = false;

void updateSampleRate(){
  cycleCount++;

  if (millis() - startTime >= 1000) {
    if (statMode){
      Serial.print("reads per second: ");
      Serial.println(cycleCount);
    }

    cycleCount = 0;
    startTime = millis();
  }
}

// 设置数据速率
void setDataRate(byte rate) {
  writeRegister(0x03, rate);  // 写入数据速率寄存器
  Serial.print("Data rate set to: ");
  Serial.println(rate, HEX);
}

// 设置PGA
void setPGA(byte pga){
  writeRegister(0x02, pga);
  currentPGAReg = pga;
  // 低3位确定增益设置
  byte gainSetting = pga & 0x07;
  switch (gainSetting) {
    case 0: currentGain = 1;  break;
    case 1: currentGain = 2;  break;
    case 2: currentGain = 4;  break;
    case 3: currentGain = 8;  break;
    case 4: currentGain = 16; break;
    case 5: currentGain = 32; break;
    case 6: currentGain = 64; break;
    default: currentGain = 1; break;
  }
  Serial.print("PGA set to: ");
  Serial.print(currentGain);
  Serial.print(" (pgaRegisterValue = ");
  Serial.print("0x");
  Serial.print(pga, HEX);
  Serial.println(")");
}


// 设置MUX通道
void setMuxChannel(int positive, int negative = 8) {
  byte muxValue = (positive << 4) | negative;
  writeRegister(0x01, muxValue);
}

// 单端读取指定通道
void readSingleChannel(int channel) {
  setMuxChannel(channel, 8);
  startConversion();
  long adcValue = waitForData();
  if (!statMode){
    Serial.print("SINGLE,");
    Serial.print(channel);
    Serial.print(",");
    Serial.println(adcValue);
    float voltage = convertToVoltage(adcValue);
    #if DEBUG_MODE
      Serial.print("SINGLE,");
      Serial.print(channel);
      Serial.print(":");
      Serial.println(adcValue);
      Serial.print(" (");
      Serial.print(voltage, 6);
      Serial.println(" V)");
    #endif
  }
}

// 差分读取指定通道
void readDifferentialChannel(int channel) {
  int positive = channel * 2;
  int negative = positive + 1;
  setMuxChannel(positive, negative);
  startConversion();
  long adcValue = waitForData();
  if (!statMode){
    Serial.print("DIFF,");
    Serial.print(channel);
    Serial.print(",");
    Serial.println(adcValue);
    float voltage = convertToVoltage(adcValue);
    #if DEBUG_MODE
      Serial.print("DIFF,");
      Serial.print(positive);
      Serial.print("-");
      Serial.print(negative);
      Serial.print(" : ");
      Serial.print(adcValue);
      Serial.print(" (");
      Serial.print(voltage, 6);
      Serial.println(" V)");
    #endif
  }
}

// 循环读取所有通道
void readAllChannels() {
  for (int channel = 0; channel < 8; channel++) {
    setMuxChannel(channel, 8);
    startConversion();
    long adcValue = waitForData();
    if (!statMode){
      Serial.print("CYCLE,");
      Serial.print(channel);
      Serial.print(",");
      Serial.println(adcValue);
      float voltage = convertToVoltage(adcValue);
      #if DEBUG_MODE
        Serial.print("CYCLE,");
        Serial.print(channel);
        Serial.print(" : ");
        Serial.print(adcValue);
        Serial.print(" (");
        Serial.print(voltage, 6);
        Serial.println(" V)");
      #endif
    }
  }
}

// 启动转换
void startConversion() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_SYNC);
  SPI.transfer(CMD_WAKEUP);
  digitalWrite(CS_PIN, HIGH);
}

// 等待数据并读取
long waitForData() {
  while (digitalRead(DRDY_PIN) == HIGH);
  return readADCData();
}

// 读取ADC数据
long readADCData() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_RDATA); // RDATA
  delayMicroseconds(5);
  long adcValue = 0;
  adcValue |= (long)SPI.transfer(0x00) << 16;
  adcValue |= (long)SPI.transfer(0x00) << 8;
  adcValue |= (long)SPI.transfer(0x00);
  if (adcValue > 0x7FFFFF) {
    adcValue -= 16777216;
  }
  digitalWrite(CS_PIN, HIGH);
  return adcValue;
}

// CONVERT_TO_VOLTAGE
inline float convertToVoltage(long adcValue) {
  float fullScale = VREF * 2 / currentGain;
  return (adcValue * fullScale) / 8388607.0;
}

// 写寄存器
void writeRegister(byte regAddress, byte regValue) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x50 | regAddress);
  SPI.transfer(0x01);
  SPI.transfer(regValue);
  digitalWrite(CS_PIN, HIGH);
}

void resetDevice() {
  digitalWrite(CS_PIN, LOW); // 选择ADS1256
  delayMicroseconds(2);

  SPI.transfer(CMD_RESET); 
  digitalWrite(CS_PIN, HIGH); // 释放ADS1256
  delayMicroseconds(2);

  Serial.println("Device reset.");
}


// from serial monitor
void processSerialCommand(const String &input) {
  String command = input;
  command.toUpperCase();
  if (command.startsWith("SINGLE")) {
    statMode = false;
    int channel = command.substring(6).toInt();
    if (channel >= 0 && channel <= 7) {
      selectedChannel = channel;
      currentMode = SINGLE;
      Serial.print("进入SINGLE模式，通道：");
      Serial.println(channel);
    } else {
      Serial.println("ERROR: SINGLE通道范围为 0-7.");
    }
  } else if (command.startsWith("DIFF")) {
    statMode = false;
    int channel = command.substring(4).toInt();
    if (channel >= 0 && channel <= 3) {
      selectedChannel = channel;
      currentMode = DIFF;
      Serial.print("进入DIFF模式，通道对：");
      Serial.print(channel * 2);
      Serial.print("-");
      Serial.println(channel * 2 + 1);
    } else {
      Serial.println("ERROR: DIFF通道范围为 0-3.");
    }
  } else if (command.startsWith("CYCLE")) {
    statMode = false;
    currentMode = CYCLE;
    Serial.println("进入CYCLE模式，循环采样所有单端通道.");
  } else if (command.startsWith("STOP")) {
    currentMode = IDLE;
    statMode = false;
    Serial.println("停止采样.");
  } else if (command.startsWith("READ")) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      String regStr = command.substring(spaceIndex + 1);
      regStr.trim();
      byte regAddress = strtol(regStr.c_str(), NULL, 16);
      readRegisterValue(regAddress);
    } else {
      Serial.println("ERROR: 未提供寄存器地址.");
    }
  } else if (command.startsWith("WRITE")) {
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    if (firstSpace != -1 && secondSpace != -1) {
      String regStr = command.substring(firstSpace + 1, secondSpace);
      String valStr = command.substring(secondSpace + 1);
      regStr.trim();
      valStr.trim();
      byte regAddress = strtol(regStr.c_str(), NULL, 16);
      byte regValue = strtol(valStr.c_str(), NULL, 16);
      writeRegister(regAddress, regValue);
      Serial.print("寄存器 0x");
      Serial.print(regAddress, HEX);
      Serial.print(" 写入 0x");
      Serial.println(regValue, HEX);
    } else {
      Serial.println("ERROR: WRITE命令格式错误.");
    }
  } else if (command.startsWith("SET DRATE")) {
    int spaceIndex = command.indexOf(' ', 9);
    if (spaceIndex != -1) {
      String rateStr = command.substring(spaceIndex + 1);
      rateStr.trim();
      byte rateValue = strtol(rateStr.c_str(), NULL, 16);
      setDataRate(rateValue);
      currentDataRate = rateValue;
    } else {
      Serial.println("ERROR: 未提供DRATE值.");
    }
  } else if (command.startsWith("SET PGA")) {
    int spaceIndex = command.indexOf(' ', 7);
    if (spaceIndex != -1) {
      String pgaStr = command.substring(spaceIndex + 1);
      pgaStr.trim();
      byte pgaValue = strtol(pgaStr.c_str(), NULL, 16) + 0x20;
      setPGA(pgaValue);
    } else {
      Serial.println("ERROR: 未提供PGA值.");
    }
  } else if (command.startsWith("RESET")) {
    resetDevice();
  } else if (command.startsWith("STAT")) {
    statMode = true;
    Serial.print("frequency: ");
  } else if (command.startsWith("HELP")) {
    printHelp();
  } else {
    Serial.println("ERROR: 未知命令，请输入 HELP 获取命令列表.");
  }
}

void readRegisterValue(byte regAddress) {
  byte regValue = readRegister(regAddress); // 读取寄存器值
  Serial.print("Register 0x");
  Serial.print(regAddress, HEX);
  Serial.print(": 0x");
  Serial.println(regValue, HEX);
}

byte readRegister(byte regAddress) {
  digitalWrite(CS_PIN, LOW); // 选择ADS1256
  delayMicroseconds(2);

  SPI.transfer(0x10 | regAddress); // WREG命令 + 寄存器地址
  SPI.transfer(0x00); // 读1字节

  delayMicroseconds(2);
  byte regValue = SPI.transfer(0x00); // 读取寄存器值

  digitalWrite(CS_PIN, HIGH); // 释放ADS1256
  return regValue;
}

void readCycleCount(int channel){
  while (digitalRead(DRDY_PIN) == HIGH);
  setMuxChannel(channel);
  startConversion();
  while (digitalRead(DRDY_PIN) == HIGH);
  readADCData();
}
