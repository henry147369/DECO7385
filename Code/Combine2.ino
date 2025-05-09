#include <Stepper.h>
#include <math.h>  // 用于 round()
#include <FastLED.h>

// 定义步进电机接线和步数（28BYJ-48，带齿轮减速共 2048 步）
#define STEPS_PER_REVOLUTION 2048  
#define IN1 27  //8
#define IN2 14  //9
#define IN3 12  //10
#define IN4 13  //11
// 灯带设置
#define NUM_STRIPS 3     // 灯带数量
#define LEDS_PER_STRIP 60 // 每条灯带的灯珠数量
#define DATA_PIN_1 2      // 第一条灯带的数据引脚
#define DATA_PIN_2 4      // 第二条灯带的数据引脚
#define DATA_PIN_3 5      // 第三条灯带的数据引脚

// 定义按钮引脚
// BUTTON_PIN：既作为启动/停止按钮（引脚2）
// ADD_BUTTON_PIN：加时按钮（引脚3）
// SUB_BUTTON_PIN：减时按钮（引脚4）
#define BUTTON_PIN 33 //2         
#define ADD_BUTTON_PIN 25 //3     
#define SUB_BUTTON_PIN 26 //4     

// 初始化步进电机库
Stepper stepper(STEPS_PER_REVOLUTION, IN1, IN2, IN3, IN4);

// 定义参与者对应的角度范围（360°分为三个区间，每个 120°）
const int PARTICIPANT_1_START = 0;
const int PARTICIPANT_1_END   = 119;
const int PARTICIPANT_2_START = 120;
const int PARTICIPANT_2_END   = 239;
const int PARTICIPANT_3_START = 240;
const int PARTICIPANT_3_END   = 359;

// 全局状态变量：上一轮选中的参与者、轮次计数、电机当前位置
int previousParticipant = 0;  // 第一轮前无选中
int roundNumber = 1;
long currentSteps = 0;        // 用 long 避免步数累计溢出
int currentAngle = 0;         // 0 ~ 359
int chosenParticipant = 0;    // 当前选中的参与者，新增全局变量

// 按钮消抖变量
int buttonState = HIGH;
int lastButtonState = HIGH;

// 全局运行状态：true 表示系统运行，false 表示已停止
bool running = true;
bool motorRunning = false;   // 新增：表示电机是否正在运行

// 函数原型声明
unsigned long adjustDelay();
void stopAndReset();
void restartSystem();
void selectParticipant();
int getCurrentParticipant();
void moveToAngle(int targetAngle);

// 倒计时相关变量
unsigned long lastUpdateTime = 0;
int activeStrip = -1;  // 当前激活的灯带，-1表示没有激活的灯带
int activeLedCount = LEDS_PER_STRIP;  // 当前亮着的LED数量
unsigned long currentDelay = 10000;  // 默认倒计时时间10秒
unsigned long timePerLed;  // 每个LED对应的时间
unsigned long countdownStartTime = 0; // 倒计时开始时间
unsigned long lastCountdownUpdate = 0; // 用于控制LED更新频率

// LED数组
CRGB leds1[LEDS_PER_STRIP];
CRGB leds2[LEDS_PER_STRIP];
CRGB leds3[LEDS_PER_STRIP];

void setup() {
  // 设置步进电机转速（制造旋转效果时使用较高转速）
  stepper.setSpeed(15);
  Serial.begin(9600);
  Serial.println("Random Participant Selector Ready");
  
  // 初始化随机种子（利用未接的模拟引脚）
  randomSeed(analogRead(A0));
  
  // 设置按钮输入（内部上拉电阻）
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ADD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SUB_BUTTON_PIN, INPUT_PULLUP);
  
  // 软件初始化：假定上电时步进电机物理上位于 0°
  currentSteps = 0;
  currentAngle = 0;
  Serial.println("Assuming motor is at starting position: 0 degrees");
  Serial.print("Initial position: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  Serial.println("Participant 1: 0-119 degrees");
  Serial.println("Participant 2: 120-239 degrees");
  Serial.println("Participant 3: 240-359 degrees");

  // 初始化灯带
  FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leds1, LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(leds2, LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PIN_3, GRB>(leds3, LEDS_PER_STRIP);
  
  // 初始化所有灯带为全灭状态
  resetAllStrips();
  FastLED.show();
  
  Serial.println("系统初始化完成");
}

void loop() {
  // 如果系统处于停止状态，则等待重启指令
  if (!running) {
    // 等待按钮按下以重新启动（确保按键释放后再启动）
    if (digitalRead(BUTTON_PIN) == LOW) {
      restartSystem();
      // 等待释放，消抖处理
      while(digitalRead(BUTTON_PIN) == LOW);
      delay(50);
    }
    return;
  }
  
  // 如果系统正在运行，按下按钮时要有不同的行为：
  // 第一轮：按钮按下用于启动；后续轮次：在等待期间按下按钮将立即停止并返回到 0°
  if (roundNumber == 1) {
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {
      Serial.print("\nRound ");
      Serial.print(roundNumber);
      Serial.println(" started by button press.");
      // 在开始选择参与者前，确保所有LED都是关闭的
      resetAllStrips();
      FastLED.show();
      selectParticipant();
      delay(50);  // 消抖处理
    }
    lastButtonState = buttonState;
  } else {
    // 后续轮次：进入等待发言状态，并允许在等待期间检测停止指令
    // 在 adjustDelay() 内会检测到停止，此处不用重复检测
    unsigned long adjustedDelay = adjustDelay();
    // 如果在 adjustDelay() 中检测到停止，running 会被置为 false，
    // 此时 return 到最前面处理重启
    if (!running) return;
    Serial.print("Adjusted delay = ");
    Serial.print(adjustedDelay);
    Serial.println(" ms. Starting next round automatically.");
    // 在开始下一轮选择参与者前，确保所有LED都是关闭的
    resetAllStrips();
    FastLED.show();
    Serial.print("VOICE_OFF");
    selectParticipant();
  }
  
  // 处理LED倒计时更新 - 增加了更频繁的更新
  updateLedCountdown();
  
  delay(10);  // 小延迟以减少刷新率
}

// 新增：专门用于更新LED倒计时的函数，在每个loop循环中调用
void updateLedCountdown() {
  // 如果有激活的灯带，处理倒计时
  if (activeStrip >= 0 && !motorRunning) {
    unsigned long currentTime = millis();
    
    // 每50毫秒更新一次LED显示，避免频繁更新
    if (currentTime - lastCountdownUpdate >= 50) {
      lastCountdownUpdate = currentTime;
      
      // 计算倒计时已经过的时间
      unsigned long elapsedTime = currentTime - countdownStartTime;
      
      // 计算每个LED对应的时间
      timePerLed = currentDelay / LEDS_PER_STRIP;
      
      // 计算应该亮着多少个LED
      int newActiveLedCount = max(0, LEDS_PER_STRIP - (int)(elapsedTime / timePerLed));
      
      // 只有当LED数量变化时才更新显示
      if (newActiveLedCount != activeLedCount) {
        activeLedCount = newActiveLedCount;
        updateActiveStrip();
        FastLED.show();
        
        // 调试输出
        Serial.print("倒计时进度: ");
        Serial.print(activeLedCount);
        Serial.print("/");
        Serial.print(LEDS_PER_STRIP);
        Serial.print(", 已过时间: ");
        Serial.print(elapsedTime);
        Serial.print("/");
        Serial.println(currentDelay);
      }
      
      // 如果倒计时结束
      if (activeLedCount <= 0) {
        Serial.print("倒计时结束: 灯带 ");
        Serial.println(activeStrip + 1);
        
        // 重置灯带状态
        activeStrip = -1;
        resetAllStrips();
        FastLED.show();
      }
    }
  }
}

// 选取参与者：
// 第一轮：全范围随机生成目标角度；后续轮次：在排除上一轮选中者的两个区间中随机生成目标角度
void selectParticipant() {
  int targetAngle;
  
  // 设置电机运行标志，关闭所有LED
  motorRunning = true;
  resetAllStrips();
  FastLED.show();
  
  if (roundNumber == 1) {
    targetAngle = random(0, 360);
    Serial.print("Target random angle (full range): ");
    Serial.println(targetAngle);
  } else {
    int allowedParticipants[2];
    int idx = 0;
    for (int i = 1; i <= 3; i++) {
      if (i != previousParticipant) {
        allowedParticipants[idx] = i;
        idx++;
      }
    }
    chosenParticipant = allowedParticipants[random(0, 2)];  // 在允许的两个中随机选取
    Serial.print("Allowed participants: ");
    Serial.print(allowedParticipants[0]);
    Serial.print(" and ");
    Serial.println(allowedParticipants[1]);
    Serial.print("Chosen participant zone: ");
    Serial.println(chosenParticipant);
    
    int startAngle, endAngle;
    switch(chosenParticipant) {
      case 1: startAngle = PARTICIPANT_1_START; endAngle = PARTICIPANT_1_END; break;
      case 2: startAngle = PARTICIPANT_2_START; endAngle = PARTICIPANT_2_END; break;
      case 3: startAngle = PARTICIPANT_3_START; endAngle = PARTICIPANT_3_END; break;
    }
    targetAngle = random(startAngle, endAngle + 1);
    Serial.print("Target random angle (restricted): ");
    Serial.println(targetAngle);
  }
  
  // 旋转电机到目标角度
  moveToAngle(targetAngle);
  
  // 根据当前角度判断选中了哪个参与者区间
  int selectedParticipant = getCurrentParticipant();
  chosenParticipant = selectedParticipant;  // 更新全局变量
  
  Serial.println("\n----- RESULT -----");
  Serial.print("Selected Participant: ");
  Serial.println(selectedParticipant);
  Serial.print("Final Angle: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  Serial.println("------------------\n");
  
  // 更新上一轮选中的参与者（供下一轮排除使用）
  previousParticipant = selectedParticipant;
  
  // 电机运行结束，重置标志
  motorRunning = false;
  
  // 触发对应参与者的LED灯带倒计时
  startCountdown(selectedParticipant - 1, currentDelay); // 参与者编号从1开始，灯带索引从0开始
  
  roundNumber++;
}

// 根据 currentAngle 判断当前电机处于哪个参与者区间
int getCurrentParticipant() {
  if (currentAngle >= PARTICIPANT_1_START && currentAngle <= PARTICIPANT_1_END)
    return 1;
  else if (currentAngle >= PARTICIPANT_2_START && currentAngle <= PARTICIPANT_2_END)
    return 2;
  else
    return 3;
}

// 修改后的 moveToAngle()：在开始定位之前，先全速逐步旋转完整一圈，然后以较慢转速精确转动到目标角度
void moveToAngle(int targetAngle) {
  float stepsPerDegree = (float)STEPS_PER_REVOLUTION / 360.0;
  
  // 计算从当前角度到目标角度的差值，归一化到 -180 ~ +180（选择最短路径）
  int angleDifference = targetAngle - currentAngle;
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference < -180) angleDifference += 360;
  
  // 计算精确转动所需步数
  int stepsToMove = round(angleDifference * stepsPerDegree);
  Serial.print("Calculated angle difference: ");
  Serial.print(angleDifference);
  Serial.print(" degrees, steps to move: ");
  Serial.println(stepsToMove);
  
  // 进行全速旋转完整一圈，逐步发出脉冲
  Serial.println("Performing one full revolution at full speed...");
  stepper.setSpeed(15);  // 设置全速
  for (int i = 0; i < STEPS_PER_REVOLUTION; i++) {
    stepper.step(1);
    delay(5); // 每步之间加入短暂延时，确保脉冲能被识别
  }
  
  // 更新 currentSteps（完整一圈在模 360 下不改变 currentAngle）
  currentSteps += STEPS_PER_REVOLUTION;
  currentSteps = (((currentSteps % STEPS_PER_REVOLUTION) + STEPS_PER_REVOLUTION) % STEPS_PER_REVOLUTION);
  currentAngle = (currentSteps * 360L) / STEPS_PER_REVOLUTION;
  
  // 以较慢转速精确转动到目标角度
  Serial.println("Performing precise rotation to target angle...");
  stepper.setSpeed(5);
  stepper.step(stepsToMove);
  
  // 更新 currentSteps 与 currentAngle
  currentSteps += stepsToMove;
  currentSteps = (((currentSteps % STEPS_PER_REVOLUTION) + STEPS_PER_REVOLUTION) % STEPS_PER_REVOLUTION);
  currentAngle = (currentSteps * 360L) / STEPS_PER_REVOLUTION;
  Serial.print("Now at angle: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  
  delay(500);  // 等待旋转完成
  
  // 关闭电机线圈，节能且防止过热
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// adjustDelay()：在发言等待期间，允许通过加时/减时按钮调整延时时间；同时检测 BUTTON_PIN（停止键）的按下
// 基础等待时间 10 秒，初始加/减操作时间为 3000 ms，每次操作后增量递减
unsigned long adjustDelay() {
  const unsigned long baseDelay = 10000;  // 基础等待 10秒
  currentDelay = baseDelay;  // 重置currentDelay为基础延迟时间
  
  float addIncrement = 3000.0;
  float subIncrement = 3000.0;
  
  unsigned long startTime = millis();
  unsigned long targetDelay = currentDelay;
  
  int addLastState = digitalRead(ADD_BUTTON_PIN);
  int subLastState = digitalRead(SUB_BUTTON_PIN);
  
  Serial.println("Adjustable waiting period started.");
  Serial.println("Press ADD to add time, SUB to reduce time, or BUTTON to stop.");
  
  while (millis() - startTime < targetDelay) {
    // 检测停止指令
    if (digitalRead(BUTTON_PIN) == LOW) {
      // 立即停止并返回
      stopAndReset();
      running = false;
      Serial.println("Stop triggered during waiting period. System stopped.");
      return targetDelay;
    }
    
    int addState = digitalRead(ADD_BUTTON_PIN);
    int subState = digitalRead(SUB_BUTTON_PIN);
    
    if (addState == LOW && addLastState == HIGH) {
      currentDelay += (unsigned long)addIncrement;
      targetDelay = currentDelay;
      Serial.print("ADD pressed, increasing delay by ");
      Serial.print(addIncrement);
      Serial.print(" ms. New delay: ");
      Serial.println(currentDelay);
      addIncrement /= 2.0;
      delay(50);
      
      // 如果当前有灯带在倒计时，调整其倒计时时间
      if (activeStrip >= 0) {
        adjustCurrentDelay(currentDelay);
      }
    }
    
    if (subState == LOW && subLastState == HIGH) {
      if (currentDelay > subIncrement)
        currentDelay -= (unsigned long)subIncrement;
      else
        currentDelay = 0;
      targetDelay = currentDelay;
      Serial.print("SUB pressed, decreasing delay by ");
      Serial.print(subIncrement);
      Serial.print(" ms. New delay: ");
      Serial.println(currentDelay);
      subIncrement /= 2.0;
      delay(50);
      
      // 如果当前有灯带在倒计时，调整其倒计时时间
      if (activeStrip >= 0) {
        adjustCurrentDelay(currentDelay);
      }
    }
    
    addLastState = addState;
    subLastState = subState;
    
    // 在等待期间也需要更新LED倒计时
    updateLedCountdown();
  }
  
  unsigned long elapsed = millis() - startTime;
  if (elapsed < currentDelay) {
    delay(currentDelay - elapsed);
  }
  return currentDelay;
}

// stopAndReset()：当检测到停止指令后，立即将电机从当前角度归位至 0°（不做戏剧性旋转）
void stopAndReset() {
  Serial.println("Stop command received. Returning motor to 0° position immediately.");
  float stepsPerDegree = (float)STEPS_PER_REVOLUTION / 360.0;
  int tolerance = 1;  // 允许的角度误差（单位：度）
  
  // 停止所有灯带的倒计时
  activeStrip = -1;
  resetAllStrips();
  FastLED.show();
  
  // 设置电机运行标志
  motorRunning = true;
  
  // 计算当前角度与 0° 之间的差值，并归一化到 -180 ~ +180
  int angleDifference = 0 - currentAngle;
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference < -180) angleDifference += 360;
  
  int iteration = 0;
  // 使用循环分段校正，每次以极低速校正，最多尝试20次
  while (abs(angleDifference) > tolerance && iteration < 20) {
    int stepsToMove = round(angleDifference * stepsPerDegree);
    // 使用更低速（例如2 RPM）进行精确归位
    stepper.setSpeed(2);
    stepper.step(stepsToMove);
    
    delay(200);  // 给电机一些时间稳定
    
    // 更新累计步数和当前角度
    currentSteps += stepsToMove;
    currentSteps = (((currentSteps % STEPS_PER_REVOLUTION) + STEPS_PER_REVOLUTION) % STEPS_PER_REVOLUTION);
    currentAngle = (currentSteps * 360L) / STEPS_PER_REVOLUTION;
    
    Serial.print("Iteration ");
    Serial.print(iteration);
    Serial.print(" - Current angle: ");
    Serial.println(currentAngle);
    
    angleDifference = 0 - currentAngle;
    while (angleDifference > 180) angleDifference -= 360;
    while (angleDifference < -180) angleDifference += 360;
    
    iteration++;
  }
  
  if (abs(angleDifference) <= tolerance) {
    Serial.println("Motor successfully reset to approximately 0°.");
  } else {
    Serial.println("Motor reset attempted, but error remains. Consider checking mechanical factors.");
  }
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // 重置电机运行标志
  motorRunning = false;
}

// restartSystem()：当系统停止后按下 BUTTON_PIN，重置系统状态并复位电机至 0°，然后允许重新开始
void restartSystem() {
  Serial.println("Restart command received. Restarting system...");
  stopAndReset();
  roundNumber = 1;
  previousParticipant = 0;
  chosenParticipant = 0;  // 重置chosenParticipant
  currentSteps = 0;
  currentAngle = 0;
  running = true;
  Serial.println("System restarted. Waiting for button press to start Round 1.");
}

// 启动指定灯带的倒计时
void startCountdown(int strip, unsigned long delay) {
  // 如果已经有灯带在倒计时，先取消
  if (activeStrip >= 0) {
    resetAllStrips();
  }
  
  Serial.print("启动灯带 ");
  Serial.print(strip + 1);
  Serial.print(" 的倒计时，时间: ");
  Serial.print(delay);
  Serial.println(" 毫秒");
  
  activeStrip = strip;
  currentDelay = delay;
  activeLedCount = LEDS_PER_STRIP;  // 设置全部LED点亮
  countdownStartTime = millis();    // 记录倒计时开始时间
  lastCountdownUpdate = millis();   // 更新最后一次LED更新时间
  
  // 初始化灯带显示 - 将选定的灯带全部点亮
  updateActiveStrip();
  FastLED.show();
}

// 调整当前倒计时时间
void adjustCurrentDelay(unsigned long newDelay) {
  if (activeStrip < 0) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - countdownStartTime;
  
  // 计算已经过的时间占比
  float elapsedProportion = (float)elapsedTime / currentDelay;
  
  // 计算剩余时间的比例
  float remainingProportion = 1.0 - elapsedProportion;
  
  // 计算新的剩余时间
  unsigned long newRemainingTime = newDelay * remainingProportion;
  
  // 更新倒计时参数
  currentDelay = newDelay;
  countdownStartTime = currentTime - (newDelay - newRemainingTime);
  
  Serial.print("调整倒计时时间为: ");
  Serial.print(newDelay);
  Serial.print(" 毫秒，剩余时间: ");
  Serial.println(newRemainingTime);
  
  // 更新灯带显示
  activeLedCount = LEDS_PER_STRIP * remainingProportion;
  updateActiveStrip();
  FastLED.show();
}

// 更新激活的灯带显示
void updateActiveStrip() {
  if (activeStrip < 0) return;
  
  // 根据activeStrip选择正确的灯带
  CRGB* currentStrip;
  
  switch (activeStrip) {
    case 0:
      currentStrip = leds1;
      break;
    case 1:
      currentStrip = leds2;
      break;
    case 2:
      currentStrip = leds3;
      break;
    default:
      return;
  }
  
  // 更新LED状态 - 前activeLedCount个LED亮起，其余熄灭
  for (int i = 0; i < LEDS_PER_STRIP; i++) {
    if (i < activeLedCount) {
      currentStrip[i] = CHSV(i * 10, 255, 255); // 色相随索引变化，形成彩虹效果
    } else {
      currentStrip[i] = CRGB::Black;  // 熄灭的LED设为黑色
    }
  }
}

// 重置所有灯带为初始状态（全灭）
void resetAllStrips() {
  // 将所有灯带设为全灭状态
  fill_solid(leds1, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds2, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds3, LEDS_PER_STRIP, CRGB::Black);
  
  // 重置倒计时状态
  activeLedCount = LEDS_PER_STRIP;
}