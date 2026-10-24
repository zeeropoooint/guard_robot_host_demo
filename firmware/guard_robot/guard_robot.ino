/*
 * Guard_robot控制板程序 V1.0
 *
 * 本程序通过 115200 波特率的文本串口协议控制舵机、A/B 两路直流电机、
 * 发射装填机构和激光发射器。每条串口命令必须以换行符结尾。
 *
 * 串口命令：
 *   @S<角度>              舵机在 200 ms 内移动到目标角度，范围 0～180
 *   @MA<速度>             设置 A 电机，范围 -255～255，0 表示停止
 *   @MB<速度>             设置 B 电机，范围 -255～255，0 表示停止
 *   @MTA<速度>,<毫秒>     A 电机定时运行，到时后自动停止
 *   @MTB<速度>,<毫秒>     B 电机定时运行，到时后自动停止
 *   @FS                   执行一次发射动作
 *   @ZZ                   执行一次装填动作
 *   @JGON                 打开激光发射器
 *   @JGOFF                关闭激光发射器
 *
 * 引脚对应关系：
 *   按键A      —— D9 ：启停云台电机（B 电机）
 *   按键B      —— D10：启停发射电机（A 电机）
 *   手动发射键 —— D13：发射一次
 *   蜂鸣器     —— D12
 *   云台电机（B 电机） —— D6、D7
 *   发射电机（A 电机） —— D4、D5
 *   红外检测传感器 —— D3
 * 功能说明：
 *   按键A：按一下，云台电机（B 电机）停止并锁定（此时串口 @MB / @MTB
 *         命令被忽略）；再按一下解锁，可重新通过串口控制，但不会自动
 *         恢复锁定前的速度。
 *   按键B：按一下，发射电机（A 电机）运转一次，完成一次装填后自动停止。
 *   手动发射键（D13）：按一下，发射一次。
 *   蜂鸣器（D12）：云台电机锁定时间歇鸣响（鸣 200 ms、停 800 ms），解锁时安静。
 * 按键采用消抖加上升沿检测，持续按住不会重复触发；锁定按键在舵机移动、
 * 发射、装填等阻塞动作的等待期间依然有效。
 */

#include <Servo.h>


// ============================== 引脚定义 ==============================

const int SERVO_PIN = 2;
const int POSITION_SENSOR_PIN = 3;

/*
 * 每路电机使用 LEVEL 和 PWM 两个控制端形成电压差。
 * 方向由两个控制端之间电压差的正负决定，不存在单独的方向引脚。
 */
const int MOTOR_LEVEL_PIN_A = 4;
const int MOTOR_PWM_PIN_A = 5;
const int MOTOR_PWM_PIN_B = 6;
const int MOTOR_LEVEL_PIN_B = 7;

const int FIRE_BUTTON_PIN = 9;
const int LOAD_BUTTON_PIN = 10;
const int LASER_PIN = 11;
const int BUZZER_PIN = 12;
const int MANUAL_FIRE_BUTTON_PIN = 13;


// ============================== 工作参数 ==============================

// 按键电平持续稳定达到该时间后，才确认状态发生变化，用于过滤机械抖动。
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// 每次改变舵机角度时，用 200 ms 完成整个移动过程。
const unsigned long SERVO_MOVE_DURATION_MS = 200;

// 舵机通常每 20 ms 接收一次控制脉冲，因此按 20 ms 间隔更新中间角度。
const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;

// 发射或装填开始后先运行固定时间，使机构完全离开当前传感器检测位置。
const unsigned long LEAVE_SENSOR_DELAY_MS = 500;

// 发射/装填时 A 电机的驱动速度。原 255 全速连续运转会严重发热
// （绕组绝缘漆已出现腥甜味），降到 180 降低电流与温升；
// 装填用时自动取发射实测值，降速后 90° 用时会自动变长，无需改时间。
const int ACTION_MOTOR_SPEED = 255;

// 位置传感器检测到定位点后，A 电机断电仍会因惯性滑过定位点。
// 停车前先反向通电一小段时间刹车，使机构停在 90° 定位点上。
// 实测仍过冲就增大 ACTION_BRAKE_MS / ACTION_BRAKE_SPEED，退回过头则减小。
const int ACTION_BRAKE_SPEED = 120;
const unsigned long ACTION_BRAKE_MS = 10;

// 一次发射/装填动作的最长等待时间。超过后判定机构卡住或传感器异常，
// 主动停车并在串口打印错误，避免程序永久停在等待循环里。
const unsigned long ACTION_TIMEOUT_MS = 5000;

// 装填方向的位置传感器始终为 HIGH，无法用它定位 90°，因此装填改为
// 按时间定角：优先用最近一次发射实测的 90° 用时，本次上电还没发射过
// 时用下面的默认值（需按实际机构实测标定，太短/太长就改这里）。
const unsigned long LOAD_STEP_MS = 990;

// B 电机锁定时，蜂鸣器鸣响 200 ms、静音 800 ms，循环提示锁定状态。
const unsigned int BUZZER_FREQUENCY_HZ = 2000;
const unsigned long BUZZER_ON_DURATION_MS = 200;
const unsigned long BUZZER_OFF_DURATION_MS = 800;


// ============================== 运行状态 ==============================

String receivedData;
Servo servo_2;

// 记录上一次下发给舵机的目标角度，作为下一次平滑移动的起点。
int currentServoAngle = 90;

// 每个按键分别保存最近采样值、消抖后的稳定值和最后一次变化时间。
bool fireButtonLastReading = LOW;
bool fireButtonStableState = LOW;
unsigned long fireButtonLastChangeTime = 0;

bool loadButtonLastReading = LOW;
bool loadButtonStableState = LOW;
unsigned long loadButtonLastChangeTime = 0;

bool manualFireButtonLastReading = LOW;
bool manualFireButtonStableState = LOW;
unsigned long manualFireButtonLastChangeTime = 0;

// B 电机上电时默认解锁；锁定后串口不能控制 B 电机。
bool motorBLocked = false;

// B 电机定时命令采用非阻塞计时，确保锁定按键能随时让电机立即停止。
bool motorBTimedRunActive = false;
unsigned long motorBStopTime = 0;

// 最近一次发射实测的 90° 用时（毫秒）；0 表示本次上电还没发射过。
unsigned long lastStepDurationMs = 0;

bool buzzerIsOn = false;
unsigned long buzzerLastChangeTime = 0;


// ============================== 函数声明 ==============================

void processSerialCommand(const String &command);
void checkButtons();
void serviceMotorBControl();
void updateMotorBTimedRun();
void updateBuzzer();
void toggleMotorBLock();
void delayWithMotorBControl(unsigned long durationMs);
void moveServoTo(int targetAngle);
void setMotorA(int speedValue);
void setMotorB(int speedValue);
void runMotorATimed(int speedValue, unsigned long durationMs);
void runMotorBTimed(int speedValue, unsigned long durationMs);
bool parseTimedMotorCommand(const String &parameters, int &speedValue,
                            unsigned long &durationMs);
void fireOnce();
void loadOnce();


// ============================== Arduino 主流程 ==============================

void setup() {
  Serial.begin(115200);

  // 缩短不完整串口行的等待时间，避免按键检测长时间停顿。
  Serial.setTimeout(10);

  servo_2.attach(SERVO_PIN);
  servo_2.write(currentServoAngle);

  pinMode(MOTOR_LEVEL_PIN_A, OUTPUT);
  pinMode(MOTOR_PWM_PIN_A, OUTPUT);
  pinMode(MOTOR_LEVEL_PIN_B, OUTPUT);
  pinMode(MOTOR_PWM_PIN_B, OUTPUT);
  pinMode(POSITION_SENSOR_PIN, INPUT);

  // 按键模块主动输出高低电平，所以这里使用普通 INPUT 模式。
  pinMode(FIRE_BUTTON_PIN, INPUT);
  pinMode(LOAD_BUTTON_PIN, INPUT);
  pinMode(MANUAL_FIRE_BUTTON_PIN, INPUT);

  // D11 上电后立即输出低电平，确保激光发射器默认为关闭状态。
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // 解锁状态下蜂鸣器必须保持安静。
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // 使用上电时的真实按键电平初始化消抖状态，避免产生虚假触发。
  fireButtonLastReading = digitalRead(FIRE_BUTTON_PIN);
  fireButtonStableState = fireButtonLastReading;
  loadButtonLastReading = digitalRead(LOAD_BUTTON_PIN);
  loadButtonStableState = loadButtonLastReading;
  manualFireButtonLastReading = digitalRead(MANUAL_FIRE_BUTTON_PIN);
  manualFireButtonStableState = manualFireButtonLastReading;

  // 明确停止两路电机，防止控制引脚上电瞬间的不确定状态引起误动作。
  setMotorA(0);
  setMotorB(0);

  for (int i = 0; i < 2; i++)
  {
    tone(BUZZER_PIN, 2000U);
    delay(250);
    noTone(BUZZER_PIN);
    delay(250);
  }
  
  Serial.print("Guard_robot控制板程序已启动 当前版本:V1.0\n");

}


void loop() {
  // 每轮循环先检查按键，使按键控制和串口控制可以同时使用。
  checkButtons();

  if (Serial.available() > 0) {
    receivedData = Serial.readStringUntil('\n');
    receivedData.trim();

    if (receivedData.length() > 0) {
      processSerialCommand(receivedData);
    }
  }
}


// ============================== 串口命令处理 ==============================

/*
 * 解析并执行一条完整命令。
 * 定时命令必须先于普通的 @MA/@MB 判断，否则相同的命令前缀会导致误匹配。
 */
void processSerialCommand(const String &command) {
  if (command == "@JGON") {
    digitalWrite(LASER_PIN, HIGH);
  }
  else if (command == "@JGOFF") {
    digitalWrite(LASER_PIN, LOW);
  }
  else if (command == "@FS") {
    fireOnce();
  }
  else if (command == "@ZZ") {
    loadOnce();
  }
  else if (command.startsWith("@MTA")) {
    int speedValue;
    unsigned long durationMs;
    if (parseTimedMotorCommand(command.substring(4), speedValue, durationMs)) {
      runMotorATimed(speedValue, durationMs);
    }
  }
  else if (command.startsWith("@MTB")) {
    int speedValue;
    unsigned long durationMs;
    if (!motorBLocked &&
        parseTimedMotorCommand(command.substring(4), speedValue, durationMs)) {
      runMotorBTimed(speedValue, durationMs);
    }
  }
  else if (command.startsWith("@MA")) {
    int motorValueA = constrain(command.substring(3).toInt(), -255, 255);
    setMotorA(motorValueA);
  }
  else if (command.startsWith("@MB")) {
    if (!motorBLocked) {
      int motorValueB = constrain(command.substring(3).toInt(), -255, 255);
      motorBTimedRunActive = false;
      setMotorB(motorValueB);
    }
  }
  else if (command.startsWith("@S")) {
    int servoAngle = constrain(command.substring(2).toInt(), 0, 180);
    moveServoTo(servoAngle);
  }
}


// ============================== 舵机控制 ==============================

/*
 * 在固定的 200 ms 内将舵机从当前角度平滑移动到目标角度。
 *
 * 程序每隔 20 ms 计算并写入一个线性插值角度，共更新 10 次。函数执行期间
 * 单片机会等待移动完成，因此同一串口中的下一条电机或激光命令不会提前执行。
 * 如果目标角度与当前角度相同，则不产生移动，也不额外等待。
 */
void moveServoTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  if (targetAngle == currentServoAngle) {
    return;
  }

  const int startAngle = currentServoAngle;
  const int angleDifference = targetAngle - startAngle;
  const int updateSteps =
      SERVO_MOVE_DURATION_MS / SERVO_UPDATE_INTERVAL_MS;

  for (int step = 1; step <= updateSteps; step++) {
    // 使用 long 进行乘法，避免角度差与步数相乘时受到 int 范围限制。
    int intermediateAngle = startAngle +
        (long)angleDifference * step / updateSteps;
    servo_2.write(intermediateAngle);
    delayWithMotorBControl(SERVO_UPDATE_INTERVAL_MS);
  }

  // 明确记录并再次写入目标值，避免整数除法造成最终角度存在舍入误差。
  currentServoAngle = targetAngle;
  servo_2.write(currentServoAngle);
}


/*
 * 读取三个按键并执行消抖和上升沿检测。
 * 只有稳定状态从 LOW 变为 HIGH 时才执行动作，因此长按不会连续触发。
 */
void checkButtons() {
  unsigned long now = millis();

  serviceMotorBControl();

  bool loadButtonReading = digitalRead(LOAD_BUTTON_PIN);
  if (loadButtonReading != loadButtonLastReading) {
    loadButtonLastChangeTime = now;
    loadButtonLastReading = loadButtonReading;
  }
  if ((now - loadButtonLastChangeTime) >= BUTTON_DEBOUNCE_MS &&
      loadButtonReading != loadButtonStableState) {
    loadButtonStableState = loadButtonReading;
    if (loadButtonStableState == HIGH) {
      loadOnce();
    }
  }

  bool manualFireButtonReading = digitalRead(MANUAL_FIRE_BUTTON_PIN);
  if (manualFireButtonReading != manualFireButtonLastReading) {
    manualFireButtonLastChangeTime = now;
    manualFireButtonLastReading = manualFireButtonReading;
  }
  if ((now - manualFireButtonLastChangeTime) >= BUTTON_DEBOUNCE_MS &&
      manualFireButtonReading != manualFireButtonStableState) {
    manualFireButtonStableState = manualFireButtonReading;
    if (manualFireButtonStableState == HIGH) {
      fireOnce();
    }
  }
}


/*
 * 持续维护 D9 锁定按键和 B 电机定时停止。该函数也会在舵机、发射、装填及
 * A 电机定时动作的等待期间调用，避免这些动作阻塞 D9 的停机功能。
 */
void serviceMotorBControl() {
  unsigned long now = millis();
  bool fireButtonReading = digitalRead(FIRE_BUTTON_PIN);

  if (fireButtonReading != fireButtonLastReading) {
    fireButtonLastChangeTime = now;
    fireButtonLastReading = fireButtonReading;
  }
  if ((now - fireButtonLastChangeTime) >= BUTTON_DEBOUNCE_MS &&
      fireButtonReading != fireButtonStableState) {
    fireButtonStableState = fireButtonReading;
    if (fireButtonStableState == HIGH) {
      toggleMotorBLock();
    }
  }

  updateMotorBTimedRun();
  updateBuzzer();
}


/*
 * D9 每次有效单击都切换一次 B 电机锁定状态。进入锁定状态时取消尚未结束的
 * 定时命令并立即停止 B 电机；重新解锁后只恢复串口控制权限，不自动恢复旧速度。
 */
void toggleMotorBLock() {
  motorBLocked = !motorBLocked;
  if (motorBLocked) {
    motorBTimedRunActive = false;
    setMotorB(0);

    // 进入锁定状态时立即鸣响一次，再按设定的开/关时间循环。
    buzzerIsOn = true;
    buzzerLastChangeTime = millis();
    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  }
  else {
    // 解锁后立即关闭蜂鸣器。
    buzzerIsOn = false;
    noTone(BUZZER_PIN);
  }
}


// 非阻塞更新蜂鸣器，使按键和串口处理不受鸣响节奏影响。
void updateBuzzer() {
  if (!motorBLocked) {
    if (buzzerIsOn) {
      buzzerIsOn = false;
      noTone(BUZZER_PIN);
    }
    return;
  }

  unsigned long interval = buzzerIsOn
      ? BUZZER_ON_DURATION_MS
      : BUZZER_OFF_DURATION_MS;
  if ((millis() - buzzerLastChangeTime) >= interval) {
    buzzerIsOn = !buzzerIsOn;
    buzzerLastChangeTime = millis();
    if (buzzerIsOn) {
      tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
    }
    else {
      noTone(BUZZER_PIN);
    }
  }
}


// 等待期间持续维护 B 电机锁定按键和定时停止功能。
void delayWithMotorBControl(unsigned long durationMs) {
  unsigned long startTime = millis();
  while ((millis() - startTime) < durationMs) {
    serviceMotorBControl();
    delay(1);
  }
}


// 检查非阻塞的 B 电机定时命令是否到时。
void updateMotorBTimedRun() {
  if (motorBTimedRunActive &&
      (long)(millis() - motorBStopTime) >= 0) {
    motorBTimedRunActive = false;
    setMotorB(0);
  }
}


// ============================== 电机基础控制 ==============================

/*
 * 设置 A 电机两个控制端的电压差。
 *
 * speedValue > 0：LEVEL=HIGH，PWM=255-speed，产生正向电压差。
 * speedValue < 0：LEVEL=LOW， PWM=abs(speed)，产生反向电压差。
 * speedValue = 0：两个控制端均为低电平，电压差为 0，电机停止。
 */
void setMotorA(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(MOTOR_LEVEL_PIN_A, HIGH);
    analogWrite(MOTOR_PWM_PIN_A, 255 - speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(MOTOR_LEVEL_PIN_A, LOW);
    analogWrite(MOTOR_PWM_PIN_A, -speedValue);
  }
  else {
    digitalWrite(MOTOR_LEVEL_PIN_A, LOW);
    analogWrite(MOTOR_PWM_PIN_A, 0);
  }
}


/*
 * B 电机采用与 A 电机相同的差分控制方式。正负号表示电压差方向，
 * 绝对值表示电压差大小，而不是独立方向引脚加 PWM 引脚的控制方式。
 */
void setMotorB(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(MOTOR_LEVEL_PIN_B, HIGH);
    analogWrite(MOTOR_PWM_PIN_B, 255 - speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(MOTOR_LEVEL_PIN_B, LOW);
    analogWrite(MOTOR_PWM_PIN_B, -speedValue);
  }
  else {
    digitalWrite(MOTOR_LEVEL_PIN_B, LOW);
    analogWrite(MOTOR_PWM_PIN_B, 0);
  }
}


/*
 * 解析定时电机命令的“速度,毫秒”参数。
 * 缺少逗号时返回 false；负数持续时间按 0 毫秒处理。
 */
bool parseTimedMotorCommand(const String &parameters, int &speedValue,
                            unsigned long &durationMs) {
  int commaPosition = parameters.indexOf(',');
  if (commaPosition < 0) {
    return false;
  }

  speedValue = constrain(parameters.substring(0, commaPosition).toInt(),
                         -255, 255);
  long requestedDuration = parameters.substring(commaPosition + 1).toInt();
  durationMs = requestedDuration > 0 ? (unsigned long)requestedDuration : 0;
  return true;
}


// A 电机按指定速度运行指定毫秒数，计时结束后由单片机直接停止。
void runMotorATimed(int speedValue, unsigned long durationMs) {
  setMotorA(speedValue);
  delayWithMotorBControl(durationMs);
  setMotorA(0);
}


// 启动 B 电机定时运行；主循环继续检测 D9，以便锁定时能够立即停止。
void runMotorBTimed(int speedValue, unsigned long durationMs) {
  if (durationMs == 0) {
    motorBTimedRunActive = false;
    setMotorB(0);
    return;
  }

  setMotorB(speedValue);
  motorBStopTime = millis() + durationMs;
  motorBTimedRunActive = true;
}


// ============================== 发射与装填 ==============================

/*
 * 执行一次发射动作：A 电机沿发射方向转动，到位置传感器检测点停车。
 * 同时记录从离开上一个检测点到到达下一个检测点的用时（即 90° 用时），
 * 供装填动作使用——装填方向的位置传感器一直为 HIGH，不能用来定位。
 */
void fireOnce() {
  unsigned long startTime = millis();   // 超时计时起点
  setMotorA(-ACTION_MOTOR_SPEED);
  delayWithMotorBControl(LEAVE_SENSOR_DELAY_MS);

  // 500 ms 后传感器仍为 HIGH，说明机构还没离开当前检测点：
  // 先等它变为 LOW，再等下一个检测点，避免把当前点误判成下一个 90° 位置。
  if (digitalRead(POSITION_SENSOR_PIN) == HIGH) {
    while (digitalRead(POSITION_SENSOR_PIN) == HIGH) {
      if ((millis() - startTime) >= ACTION_TIMEOUT_MS) {
        setMotorA(0);
        Serial.print("发射超时：位置传感器一直未离开当前检测点\n");
        return;
      }
      serviceMotorBControl();
    }
  }

  unsigned long stepStart = millis();   // 从离开检测点开始计 90° 用时
  while (digitalRead(POSITION_SENSOR_PIN) == LOW) {
    // 等待机构到达下一个位置传感器检测点（相邻检测点相隔 90°）。
    if ((millis() - startTime) >= ACTION_TIMEOUT_MS) {
      setMotorA(0);
      Serial.print("发射超时：未检测到下一个 90° 定位点\n");
      return;
    }
    serviceMotorBControl();
  }
  lastStepDurationMs = millis() - stepStart;
  Serial.print("发射 90° 用时 ");
  Serial.print(lastStepDurationMs);
  Serial.print(" ms\n");

  // 到达检测点：反向通电刹车，抵消惯性造成的过冲，再断电停住。
  setMotorA(ACTION_BRAKE_SPEED);
  delayWithMotorBControl(ACTION_BRAKE_MS);
  setMotorA(0);
}


/*
 * 执行一次装填动作。
 * 装填方向的位置传感器一直保持 HIGH，无法用它定位 90°，因此按时间定角：
 * 用最近一次发射实测的 90° 用时反向转动；本次上电还没发射过时用
 * LOAD_STEP_MS。要精确控制时，先发射一次让程序测出实际用时。
 */
void loadOnce() {
  unsigned long duration = lastStepDurationMs > 0
      ? lastStepDurationMs : LOAD_STEP_MS;

  setMotorA(ACTION_MOTOR_SPEED);
  delayWithMotorBControl(duration);

  // 反向短脉冲刹车，减少惯性带来的多转。
  setMotorA(-ACTION_BRAKE_SPEED);
  delayWithMotorBControl(ACTION_BRAKE_MS);
  setMotorA(0);
}
