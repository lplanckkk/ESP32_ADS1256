#include <SPI.h>

#define CS_PIN    15
#define MISO_PIN  12
#define MOSI_PIN  13
#define SCK_PIN   14
#define DRDY_PIN  2
#define VREF      2.5

#include <algorithm>

// 调试模式：serial monitor调试置1，工作置0
#define DEBUG_MODE 0

#if DEBUG_MODE
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

uint8_t crc8(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    uint8_t in = *buf++;
    for (uint8_t i = 0; i < 8; i++) {
      uint8_t mix = (crc ^ in) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      in >>= 1;
    }
  }
  return crc;
}

static const int MEDIAN_WINDOW = 30;
static long medianBuf[MEDIAN_WINDOW];
static int bufIndex = 0;        // 下一个写入位置
static bool bufFull = false;    // 是否已填满一次
static bool medianMuxSet = false;

// 模式定义
enum ReadMode { IDLE, SINGLE, DIFF, CYCLE, MEDIAN };
ReadMode currentMode = IDLE;
int selectedChannel = -1;
int lastChannel = -1;

// 统计变量
unsigned long cycleCount = 0; // 循环计数器
unsigned long startTime = 0;  // 开始时间

// 当前配置：数据速率和PGA寄存器值
uint8_t currentDataRate = 0xF0;   // 默认数据速率
uint8_t currentPGAReg   = 0x20;    // 默认PGA寄存器值
// 根据PGA寄存器低3位映射增益：0->1, 1->2, 2->4, 3->8, 4->16, 5->32, 6->64
float currentGain = 1.0;

// SPI_CMD
const uint8_t CMD_RESET   = 0xFE;
const uint8_t CMD_SYNC    = 0xFC;
const uint8_t CMD_WAKEUP  = 0x00;
const uint8_t CMD_RDATA   = 0x01;
const uint8_t CMD_RDATAC  = 0x03;
const uint8_t CMD_SDATAC  = 0x0F;
const uint8_t CMD_RREG    = 0x10; // Read from REG
const uint8_t CMD_WREG    = 0x50; // Write to REG
const uint8_t REG_STATUS  = 0x00;
const uint8_t REG_MUX     = 0x01;
const uint8_t REG_ADCON   = 0x02;
const uint8_t REG_DRATE   = 0x03;
const uint8_t REG_IO      = 0x04;

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

byte readRegister(byte regAddress);
void writeRegister(byte regAddress, byte regValue);
void setDataRate(byte rate);
void setPGA(byte pga);
void resetDevice();
long waitForData();
long readADCData();
void sendFrame(int chan, long raw);

void setup() {
  Serial.begin(2000000);
  while (!Serial); // 等待串口连接

  pinMode(CS_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.beginTransaction(SPISettings(1300000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_PIN, HIGH); 
  Serial.println("Initializing ADS1256...");

  resetDevice();
  
  setDataRate(currentDataRate);
  setPGA(currentPGAReg);

  // 启用输入缓冲器 (BUFEN, STATUS bit 1) 和自动校准 (ACAL, STATUS bit 2)
  // 后续PGA或DRATE更改将自动触发校准
  uint8_t status_val = readRegister(REG_STATUS);
  status_val |= (1 << 1); // Set BUFEN (bit 1)
  status_val |= (1 << 2); // Set ACAL (bit 2)
  writeRegister(REG_STATUS, status_val);
  Serial.println("Input Buffer (BUFEN) and Auto-Calibration (ACAL) enabled.");
  
  byte new_status_check = readRegister(REG_STATUS);
  Serial.print("STATUS register after BUFEN/ACAL enable: 0x");
  Serial.println(new_status_check, HEX);

  Serial.println("ADS1256 ready for commands.");

  startTime = millis();
  #if DEBUG_MODE
    printHelp();
  #endif
}

// 设置MUX通道
void setMuxChannel(int positive, int negative = 8) {
  byte muxValue = (positive << 4) | negative;
  writeRegister(0x01, muxValue);
  delayMicroseconds(100);
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


void loop() {
  // from python
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.toUpperCase();
    if (cmd.startsWith("SINGLE")) {
      selectedChannel = cmd.substring(6).toInt();
      currentMode     = SINGLE;
      lastChannel     = -1;
    }
    else if (cmd.startsWith("DIFF")) {
      selectedChannel = cmd.substring(4).toInt();
      currentMode     = DIFF;
      lastChannel     = -1;
    }
    else if (cmd.startsWith("CYCLE")) {
      currentMode     = CYCLE;
      selectedChannel = -1;
      lastChannel     = -1;
    }
    else if (cmd.startsWith("MEDIAN")) {
      selectedChannel = cmd.substring(6).toInt();
      currentMode     = MEDIAN;
      bufFull = false;
      bufIndex        = 0;

      int p = selectedChannel * 2, n = p + 1;
      setMuxChannel(p, n);
      startConversion();
      waitForData();
    }
    else if (cmd.startsWith("STOP")) {
      currentMode = IDLE;
    }
    else if (cmd.startsWith("SET DRATE")) {
      byte r = strtol(cmd.substring(cmd.indexOf(' ',9)+1).c_str(), NULL, 16);
      setDataRate(r);
    }
    else if (cmd.startsWith("SET PGA")) {
      byte p = strtol(cmd.substring(cmd.indexOf(' ',7)+1).c_str(), NULL, 16) | 0x20;
      setPGA(p);
    }
    else if (cmd.startsWith("RESET")) {
      resetDevice();
    }
  }

  // 根据当前模式读取数据
  switch (currentMode) {
    case SINGLE: {
      if (selectedChannel != lastChannel) {
        setMuxChannel(selectedChannel, 8);
        lastChannel = selectedChannel;
      }
      startConversion();
      long v = waitForData();
      sendFrame(selectedChannel, v);
      break;
    }
    case DIFF: {
      if (selectedChannel != lastChannel) {
        int p = selectedChannel*2, n = p+1;
        setMuxChannel(p, n);
        lastChannel = selectedChannel;
      }
      startConversion();
      long v = waitForData();
      sendFrame(selectedChannel, v);
      break;
    }
    case CYCLE: {
      for (int ch=0; ch<8; ch++){
        setMuxChannel(ch,8);
        startConversion();
        long v = waitForData();
        sendFrame(ch, v);
      }
      lastChannel = -1;
      break;
    }
    case MEDIAN: {
      startConversion();
      long s = waitForData();
      medianBuf[bufIndex] = s;
      bufIndex = (bufIndex+1)%MEDIAN_WINDOW;
      if (bufIndex==0) bufFull = true;
      if (bufFull) {
        long tmp[MEDIAN_WINDOW];
        memcpy(tmp, medianBuf, sizeof(tmp));
        std::nth_element(tmp, tmp+MEDIAN_WINDOW/2, tmp+MEDIAN_WINDOW);
        long m = tmp[MEDIAN_WINDOW/2];
        sendFrame(selectedChannel, m);
      }
      break;
    }
    default:
      delay(1);
  }

  // from serial monitor
  #if DEBUG_MODE
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n'); // 读取输入直到换行符
      processSerialCommand(input); // 处理串口命令
      return;
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
void setDataRate(uint8_t rate) {
  writeRegister(REG_DRATE, rate);  // 写入数据速率寄存器
  currentDataRate = rate; // 更新全局变量
  Serial.print("Data rate set to: ");
  Serial.println(rate, HEX);
}

// 设置PGA
void setPGA(uint8_t pga){
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

// 单端读取指定通道
void readSingleChannel(int channel) {
  setMuxChannel(channel, 8);
  startConversion();
  long adcValue = waitForData();

  uint8_t frame[8];
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 0xA5;
  frame[3] = channel & 0x0F;
  frame[4] = (adcValue >> 16) & 0xFF;
  frame[5] = (adcValue >>  8) & 0xFF;
  frame[6] = (adcValue >>  0) & 0xFF;
  frame[7] = crc8(frame,7);
  Serial.write(frame,8);

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

  uint8_t frame[8];
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 0xA5;
  frame[3] = channel & 0x0F;
  frame[4] = (adcValue >> 16) & 0xFF;
  frame[5] = (adcValue >>  8) & 0xFF;
  frame[6] = (adcValue >>  0) & 0xFF;
  frame[7] = crc8(frame,7);
  Serial.write(frame,8);

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

// 读取ADC数据
long readADCData() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_RDATA); // RDATA
  delayMicroseconds(7);
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
void writeRegister(uint8_t regAddress, uint8_t regValue) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_WREG | regAddress);
  SPI.transfer(0x00);
  SPI.transfer(regValue);
  digitalWrite(CS_PIN, HIGH);
}

void resetDevice() {
  digitalWrite(CS_PIN, LOW); // 选择ADS1256
  delayMicroseconds(2);

  SPI.transfer(CMD_RESET); 
  digitalWrite(CS_PIN, HIGH); // 释放ADS1256
  delay(10);

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
      Serial.print("Mode: SINGLE, Channel:");
      Serial.println(channel);
    } else {
      Serial.println("ERROR: SINGLE channel must be 0-7.");
    }
  } else if (command.startsWith("DIFF")) {
    statMode = false;
    int channel = command.substring(4).toInt();
    if (channel >= 0 && channel <= 3) {
      selectedChannel = channel;
      currentMode = DIFF;
      Serial.print("Mode: DIFF, Channel Pair: AIN");
      Serial.print(channel * 2);
      Serial.print("-");
      Serial.println(channel * 2 + 1);
    } else {
      Serial.println("ERROR: DIFF channel pair must be 0-3.");
    }
  } else if (command.startsWith("CYCLE")) {
    statMode = false;
    currentMode = CYCLE;
    Serial.println("Mode: CYCLE - cycling all single-ended channels.");
  } else if (command.startsWith("MEDIAN")) {
    statMode = false;
    int channel = command.substring(6).toInt();
    if (channel >= 0 && channel <= 7) {
      selectedChannel = channel;
      currentMode = MEDIAN;
      selectedChannel = channel;
      medianMuxSet = false;
      bufIndex = 0;
      bufFull = false;
      Serial.print("Mode: MEDIAN, Channel:");
      Serial.println(channel);
    } else {
      Serial.println("ERROR: MEDIAN channel must be 0-7.");
    }
  } else if (command.startsWith("STOP")) {
    currentMode = IDLE;
    statMode = false;
    Serial.println("Sampling stopped.");
  } else if (command.startsWith("READ")) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      String regStr = command.substring(spaceIndex + 1);
      regStr.trim();
      byte regAddress = strtol(regStr.c_str(), NULL, 16);
      readRegisterValue(regAddress);
    } else {
      Serial.println("ERROR: READ command needs a register address (hex).");
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

uint8_t readRegister(byte regAddress) {
  digitalWrite(CS_PIN, LOW); // 选择ADS1256

  SPI.transfer(CMD_RREG | regAddress); // WREG命令 + 寄存器地址
  SPI.transfer(0x00); // 读1字节

  delayMicroseconds(7);
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

void readMedianChannel(int channel){
  if (!medianMuxSet) {
    int positive = channel * 2;
    int negative = positive + 1;
    setMuxChannel(positive, negative);
    medianMuxSet = true;
  }
  startConversion();
  long newSample = waitForData();
  medianBuf[bufIndex] = newSample;

  bufIndex = (bufIndex + 1) % MEDIAN_WINDOW;
  if (bufIndex == 0) bufFull = true;

  if (bufFull) {
    // 复制一份到排序数组
    long temp[MEDIAN_WINDOW];
    memcpy(temp, medianBuf, sizeof(temp));
    // 排序取中位
    std::nth_element(temp, temp + MEDIAN_WINDOW/2, temp + MEDIAN_WINDOW);
    long medianValue = temp[MEDIAN_WINDOW/2];

    uint8_t frame[8];
    frame[0] = 0xA5;
    frame[1] = 0x5A;
    frame[2] = 0xA5;
    frame[3] = channel & 0x0F;
    frame[4] = (medianValue >> 16) & 0xFF;
    frame[5] = (medianValue >>  8) & 0xFF;
    frame[6] = (medianValue >>  0) & 0xFF;
    frame[7] = crc8(frame,7);
    Serial.write(frame,8);

    if (!statMode) {
      Serial.print("MEDIAN,");
      Serial.print(channel);
      Serial.print(",");
      Serial.println(medianValue);
    }
  }
}

void sendFrame(int chan, long raw) {
  if (statMode) return;
  #if DEBUG_MODE
    const char* modeStr;
    switch (currentMode) {
      case SINGLE: modeStr = "SINGLE"; break;
      case DIFF:   modeStr = "DIFF";   break;
      case CYCLE:  modeStr = "CYCLE";  break;
      case MEDIAN: modeStr = "MEDIAN"; break;
      default:     modeStr = "UNKNOWN";break;
    }
    Serial.print(modeStr);
    Serial.print(",");
    Serial.print(chan);
    Serial.print(",");
    Serial.println(raw);
  #else
    uint8_t frame[8];
    frame[0] = 0xA5;
    frame[1] = 0x5A;
    frame[2] = 0xA5;
    frame[3] = chan & 0x0F;
    frame[4] = (raw >> 16) & 0xFF;
    frame[5] = (raw >>  8) & 0xFF;
    frame[6] = (raw >>  0) & 0xFF;
    frame[7] = crc8(frame, 7);
    Serial.write(frame, 8);
  #endif
}
