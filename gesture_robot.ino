/******************************************************************************
 * 二轴手势桌面机器人 - ESP32-WROOM-32 固件
 *
 * 功能：
 *   1. 通过 BLE 接收网页控制台发来的指令（蓝牙名称 GestureRobot）
 *   2. 两个舵机控制头部：左右旋转(pan) + 抬头/低头(tilt)
 *   3. 7 脚 SPI OLED(SSD1306 128x64) 显示表情：
 *      0=待机  1=笑脸(五指张开)  2=哭脸(五指捏合)  3=委屈(握拳)
 *
 * 通信协议（纯文本，每条以换行结尾）：
 *   P090T090E0\n
 *   P=左右舵机角度(0~180)  T=俯仰舵机角度(0~180)  E=表情编号(0~3)
 *
 * 硬件接线：
 *   OLED GND -> GND      VDD -> 3V3     SCK -> GPIO18
 *        SDA -> GPIO23   RES -> GPIO4   DC  -> GPIO2   CS -> GPIO5
 *   舵机1(左右 pan) 信号线 -> GPIO13
 *   舵机2(俯仰 tilt)信号线 -> GPIO14
 *   舵机电源必须外部 5V 供电，外部电源 GND 必须与 ESP32 GND 共地！
 *
 * 依赖库（Arduino IDE 库管理器）：U8g2(oliver)、ESP32Servo(Kevin Harrington)
 * BLE 库随 ESP32 开发板包自带，无需额外安装。
 ******************************************************************************/

#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================== 引脚配置 ==============================
static const uint8_t PIN_SERVO_PAN  = 13;   // 舵机1：左右旋转
static const uint8_t PIN_SERVO_TILT = 14;   // 舵机2：抬头/低头

// ============================== BLE 配置 ==============================
#define SERVICE_UUID  "6e8a5d02-1a2b-4c3d-9e0f-111122223333"
#define CHARACTER_UUID "6e8a5d03-1a2b-4c3d-9e0f-111122223333"
#define BLE_DEVICE_NAME "GestureRobot"

// ============================== 舵机参数 ==============================
static const int SERVO_PAN_MIN  = 30;    // 左右旋转安全范围
static const int SERVO_PAN_MAX  = 150;
static const int SERVO_TILT_MIN = 60;    // 俯仰安全范围（避免机械干涉）
static const int SERVO_TILT_MAX = 120;
static const int SERVO_CENTER   = 90;
static const int SERVO_STEP     = 3;     // 每 20ms 最大转动度数（数字越小越平滑）

// 表情编号
enum Emotion { EM_NEUTRAL = 0, EM_HAPPY = 1, EM_CRY = 2, EM_PITY = 3 };

// ============================== 全局对象 ==============================
// 0.96 寸 SSD1306；1.3 寸 SH1106 请换为文件末尾注释中的构造函数
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
                                            /* cs=*/    5,
                                            /* dc=*/    2,
                                            /* reset=*/ 4);

Servo servoPan;
Servo servoTilt;

// 运行状态
volatile int  targetPan  = SERVO_CENTER;
volatile int  targetTilt = SERVO_CENTER;
volatile int  emotion    = EM_NEUTRAL;
int  curPan  = SERVO_CENTER;
int  curTilt = SERVO_CENTER;
bool connected = false;
uint32_t lastCmdMs = 0;
int  lastEmotion   = -1;
bool lastConnected = false;

// ============================== 指令解析 ==============================
void handleCommand(const std::string &line) {
  int p = -1, t = -1, e = -1;
  if (sscanf(line.c_str(), "P%dT%dE%d", &p, &t, &e) == 3) {
    targetPan  = constrain(p, SERVO_PAN_MIN, SERVO_PAN_MAX);
    targetTilt = constrain(t, SERVO_TILT_MIN, SERVO_TILT_MAX);
    emotion    = constrain(e, EM_NEUTRAL, EM_PITY);
    lastCmdMs  = millis();
  }
}

// ============================== BLE 回调 ==============================
class RobotServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *) override {
    connected = false;
    BLEDevice::startAdvertising();   // 断开后重新广播，等待下次连接
  }
};

class RobotCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    static std::string buf;
    std::string value = chr->getValue().c_str();  // 兼容 ESP32 3.x 内核（getValue 返回 String）
    for (char ch : value) {
      if (ch == '\n') {
        if (!buf.empty()) handleCommand(buf);
        buf.clear();
      } else if (buf.length() < 20) {
        buf += ch;
      }
    }
  }
};

// ============================== OLED 表情绘制 ==============================
// 用像素画圆弧（U8g2 不直接支持任意角度圆弧）
void drawArc(int cx, int cy, int r, int aStart, int aEnd) {
  for (int a = aStart; a <= aEnd; a++) {
    float rad = a * PI / 180.0f;
    for (int rr = r; rr <= r + 1; rr++) {
      u8g2.drawPixel(cx + (int)lroundf(rr * cosf(rad)),
                     cy + (int)lroundf(rr * sinf(rad)));
    }
  }
}

void drawFace() {
  u8g2.clearBuffer();
  u8g2.drawCircle(64, 32, 22);   // 脸

  switch (emotion) {
    case EM_HAPPY:  // 笑脸：圆眼睛 + 上扬微笑(U形)
      u8g2.drawDisc(56, 27, 3);
      u8g2.drawDisc(72, 27, 3);
      drawArc(64, 33, 8, 20, 160);
      break;

    case EM_CRY:    // 哭脸：下垂嘴角(∩形) + 两行眼泪
      u8g2.drawCircle(56, 27, 3);
      u8g2.drawCircle(72, 27, 3);
      drawArc(64, 39, 8, 200, 340);
      u8g2.drawVLine(52, 30, 7);
      u8g2.drawDisc(52, 39, 1);
      u8g2.drawVLine(76, 30, 7);
      u8g2.drawDisc(76, 39, 1);
      break;

    case EM_PITY:   // 委屈：八字眉 + 小眼睛 + 波浪嘴 + 一颗泪
      u8g2.drawLine(49, 24, 58, 19);
      u8g2.drawLine(79, 24, 70, 19);
      u8g2.drawDisc(56, 28, 2);
      u8g2.drawDisc(72, 28, 2);
      u8g2.drawLine(56, 43, 60, 40);
      u8g2.drawLine(60, 40, 64, 43);
      u8g2.drawLine(64, 43, 68, 40);
      u8g2.drawLine(68, 40, 72, 43);
      u8g2.drawVLine(52, 30, 6);
      u8g2.drawDisc(52, 38, 1);
      break;

    default:        // 待机：平静表情
      u8g2.drawDisc(56, 27, 2);
      u8g2.drawDisc(72, 27, 2);
      u8g2.drawLine(57, 42, 71, 42);
      break;
  }

  // 底部连接状态
  u8g2.setFont(u8g2_font_6x10_tr);
  const char *st = connected ? "Connected" : "Waiting BLE";
  int w = u8g2.getStrWidth(st);
  u8g2.drawStr((128 - w) / 2, 63, st);
  u8g2.sendBuffer();
}

// ============================== 舵机平滑运动 ==============================
int stepToward(int cur, int tgt, int step) {
  if (cur < tgt) return min(cur + step, tgt);
  if (cur > tgt) return max(cur - step, tgt);
  return cur;
}

// ============================== setup ==============================
void setup() {
  Serial.begin(115200);

  // OLED
  u8g2.begin();

  // 舵机
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_SERVO_PAN, 500, 2400);
  servoTilt.attach(PIN_SERVO_TILT, 500, 2400);
  servoPan.write(SERVO_CENTER);
  servoTilt.write(SERVO_CENTER);

  // BLE
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new RobotServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *characteristic = service->createCharacteristic(
      CHARACTER_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR |
      BLECharacteristic::PROPERTY_NOTIFY);
  characteristic->addDescriptor(new BLE2902());
  characteristic->setCallbacks(new RobotCharacteristicCallbacks());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("GestureRobot BLE ready, waiting for connection...");
  drawFace();
}

// ============================== loop ==============================
void loop() {
  uint32_t now = millis();

  // 未连接，或连接后 1.5 秒没有收到指令：回正 + 待机表情
  if (!connected || now - lastCmdMs > 1500) {
    targetPan  = SERVO_CENTER;
    targetTilt = SERVO_CENTER;
    emotion    = EM_NEUTRAL;
  }

  // 平滑逼近目标角度
  curPan  = stepToward(curPan,  targetPan,  SERVO_STEP);
  curTilt = stepToward(curTilt, targetTilt, SERVO_STEP);
  servoPan.write(curPan);
  servoTilt.write(curTilt);

  // 仅在表情/连接状态变化时重绘屏幕
  if (emotion != lastEmotion || connected != lastConnected) {
    drawFace();
    lastEmotion   = emotion;
    lastConnected = connected;
  }

  delay(20);
}

/* ===================== 1.3 寸 SH1106 屏幕替换说明 =====================
 * 把第 55 行附近的 SSD1306 构造函数注释掉，替换为：
 * U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 5, 2, 4);
 * 其余代码不变。
 * ===================================================================== */
