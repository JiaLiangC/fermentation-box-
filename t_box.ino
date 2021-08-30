#include <Arduino.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
//#include <uFire_SHT20.h>
#include <Adafruit_PCD8544.h>
#include <MsTimer2.h>
#include <PID_v1.h>


#define PIN_INPUT 0
#define PIN_OUTPUT 3

double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//这里的kp 5000x(温差)+kix(温差) ，WindowSize 给的6秒，输出比WindowSize大就打开加热器，比如温差为2 ，那么始终打开，
// 比如温差为1，那么5000 在WindowSize 的前5S都小与6000，所以6秒的窗口中打开5S，PID输出其实就是6S的时间窗口中打开多久的开关。
double aggKp = 5000, aggKi = 0.2, aggKd = 0;

//温差为1，那么1*4000 输出大概四千几，就是加热6秒的窗口中加热4s,
//问题在于0.5的时候，输出大概2000多，加热片和风扇刚启动还没来得及做功就关了。所以调整积分项很大。保证差0.5度的时候，至少加热3-4秒
double consKp = 4000, consKi = 400, consKd = 0;
//差2度至少加热5s

long WindowSize = 6000;
long negWindowSize = -6000;
unsigned long windowStartTime;

static int heaterStatus=0;
static int coolerStatus=0;


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

//TODO 制冷的PID需要调整，因为制冷很不灵敏


/*
   快速PWM
   对于快速PWM来说，时钟都是从0计数到255。
   当计数器=0时，输出高电平1，当计数器等于比较寄存器时，输出低电平0。所以输出比较器越大，占空比越高。
   这就是快速PWM模式。后面的例子会解释如何用OCRnA和OCRnB设置两路输出的占空比。
   很明显这种情况下，这两路输出的周期是相同的，只是占空比不同。
   atmega328p 默认的pwm 16M ,快速PWM模式预分频为8
   举个例子
   TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
   TCCR2B = _BV(CS22);
   OCR2A = 180;
   OCR2B = 50;
   WGM的设置为011，表示选择了快速PWM模式
   COM2A和COM2B设置为10，表示A和B输出都是非反转的PWM
   WGM2 WGM1 1和2 表示timer2 和 timer2
   OCR2A和OCR2B分别是180和50，表示两路输出的占空比；

   快速PWM下，修改时钟的计数上限
   快速PWM和相位修正PWM都可以重新设置输出的频率，先看看快速PWM是如何设置的。、
   在修改频率的模式下，时钟从0开始计数到OCRA而不是255，注意这个OCRA我们之前是用来做比较用的。这样一来，频率的设置就非常灵活了。对Timer1来说，OCRA可以设置到16位（应该是0~65535)
   WGM设置为111表示“OCRA控制计数上限的快速PWM”
*/


/* We have already used this function throughout this tutorial, but I didn’t want to finish it without talking about it.
* This is the display.setCursor function, which will position you at the pixel you indicate. For this we have to take into account that the display has a resolution of 84×48 pixels,
* so the center, for example, would be the 42×24 coordinate.
* x = (<display-x-size> - <object-x-size>)/2
* y = (<display-y-size> - <object-y-size>)/2
* With this formula we can center the letter with the following formula:
* x = (84 - 10)/2 = 74 / 2 = 37
* y = (48 - 14)/2 = 34 / 2 = 17
*/


/*功能，
 *  天贝模式，
 * 记忆功能，掉电重启后从上次的地方继续开始, 复位功能。
*/

/* LCD5110_Pin SCK  - Pin  2
       MOSI - Pin 3
       DC   - Pin 4
       CS   - Pin 5
       RST  - Pin 6
*/

Adafruit_PCD8544 display = Adafruit_PCD8544(2, 3, 4, 5, 6); //LCD 5110 pin （CLK,DIN,D/C,CE,RST）


const unsigned long UPDATE_INTERVAL = 200;  //风扇测速更新间隔

const unsigned long SECOND = 1000 * 1;
const unsigned long MINUTES = 60 * SECOND;
const unsigned long HOUR = MINUTES * 60;

//制热风扇
const int HEATER_POWER_PIN = 7;      // Fan power ssr
const int HEATER_PWM_PIN = 9;        // Timer1 pwm pin 制热风扇转速

//制冷风扇
const int COOLER_PWM_PIN = 10;      // Timer1 pwm pin 制冷风扇转速
const int COOLER_POWER_PIN = 12;    // Fan power ssr switch

#define heaterPin A0    //PTC  heater switch pin
//TODO  DA 功率控制
#define coolerPin A1    //PTC  heater switch pin

#define ONE_WIRE_BUS  A3              //ds18b20 pin
OneWire oneWire(ONE_WIRE_BUS);       //声明
DallasTemperature sensors(&oneWire); //声明

Adafruit_AHTX0 aht;

// uFire_SHT20 sht20;    //A4 A5 sht20 iic pin 环境温度

/*参数设置区
  确保天贝处于29-32摄氏度
  针孔的尺寸十分关键 2厘米一个孔，多了没啥用，缺氧但是有氧的环境刚好
  湿度在 50% 到 75% 之间最好
  气流在120cfm ?? 只能自己测试了
  主要控制温度，湿度，湿度大了就吹风，温度高了也吹风
  第一阶段
*/

sensors_event_t humidity, temp;

typedef struct {
  unsigned long active_time;
  unsigned long time_sec;
  unsigned long targetTemp;
} process;


void displayInit()
{
  display.begin();
  display.setContrast(50); // you can change the contrast around to adapt the displayfor the best viewing!
}

void pidInit() {
  windowStartTime = millis();
  myPID.SetOutputLimits(negWindowSize, WindowSize);//设置输出范围
  myPID.SetSampleTime(500); //采样时间设置
  myPID.SetMode(AUTOMATIC); // PID 开
}


// Timer1 使用ICR1作为比较器，所以有两路PWM输出, 控制风扇电源
void fanOpenWithPWMPulseRatio(int pin, int ration) {
  TCCR1A = 0;           // undo the configuration done by...
  TCCR1B = 0;           // ...the Arduino core library
  TCNT1  = 0;           // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
           | _BV(COM1B1)  // same on ch; B
           | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)   // ditto
           | _BV(CS10);   // prescaler = 1
  ICR1   = 320;         // TOP = 320

  pinMode(COOLER_POWER_PIN, OUTPUT);
  pinMode(HEATER_POWER_PIN, OUTPUT);

  switch (pin) {
    case HEATER_PWM_PIN:
      OCR1A = ration;
      pinMode(HEATER_PWM_PIN, OUTPUT);
      digitalWrite(HEATER_POWER_PIN, HIGH);
      break;
    case COOLER_PWM_PIN:
      OCR1B = ration;
      pinMode(COOLER_PWM_PIN, OUTPUT);
      digitalWrite(COOLER_POWER_PIN, HIGH);
      break;
    default:
      break;
  }
}


//关闭PWM，清零对应的寄存器位, 直接关电源
void fanOff(int pin)
{
  // digitalWrite 内部实现会调用 turnOffPWM 清空定时器设置，关闭PWM
  if (pin == HEATER_PWM_PIN) {
    digitalWrite(HEATER_POWER_PIN, LOW);
  }
  if (pin == COOLER_PWM_PIN)
  {
    digitalWrite(COOLER_POWER_PIN, LOW);
  }
}

void periodicalAirFlow()
{
  MsTimer2::set(500, ariFlow); // 500ms period
  MsTimer2::start();
}

void periodicalAirFlowOff()
{
  MsTimer2::set(500, ariFlow); // 500ms period
  MsTimer2::stop();
}

void ariFlow()
{
  static unsigned long ariFlowWindowStartTime = 0;

  // 学习数字PID控制中的窗口PWM控制法
  int ariFlowWindowSize = 6*MINUTES;
  int pulse_ration = 2*MINUTES;

  if (millis() - ariFlowWindowStartTime > ariFlowWindowSize)
  { //time to shift the Relay Window
    ariFlowWindowStartTime += ariFlowWindowSize;
  }
  double elapsed =  millis() - windowStartTime;

  if ((elapsed < pulse_ration) && heaterStatus==0)
  {
    fanOpenWithPWMPulseRatio(HEATER_PWM_PIN, 120);//48%
  }else{
      //运行2分钟后，停4分钟, 已经停止了的状态就不停
     fanOff(HEATER_PWM_PIN);
  }
}

// TODO 外部环境温度湿度，时间单独用个Arduino 去做, 明年升级的时候换个大一点的板子
//目前只支持天贝模式, 30度发酵13小时，25度发酵18小时

process TempahProcess[2] =
{
    {
        .time_sec = MINUTES * 2,
        .targetTemp = 36.0,
        .active_time = 0
    },
    {
        .time_sec = MINUTES * 10,
        .targetTemp = 24.0,
        .active_time = 0
    }
};


void setup()
{
  Serial.begin(9600);

  aht.begin(); //Aht20Init();

  displayInit(); //LCD init

  //18b20 init
  sensors.begin(); //初始化总线
  sensors.setWaitForConversion(false); //设置为非阻塞模式

  //加热制冷初始化
  pinMode(heaterPin, OUTPUT);
  pinMode(coolerPin, OUTPUT);

  pidInit(); //pid 设置初始化

  periodicalAirFlow();

}

// 设定温度,当前温度,当前湿度
// 阶段倒计时
// 风扇状态：HFan CFan

process tempahP;

void loop()
{
    static int i=0;
    tempahP = TempahProcess[i];
    Setpoint = tempahP.targetTemp;
    pidTemControl();
    tempahP.active_time += UPDATE_INTERVAL;
    Displaytemp(tempahP.targetTemp, 0, tempahP.active_time, tempahP.time_sec);
    if tempahP.active_time > TempahProcess0.time_sec  i+=1;
    if(i>2) i=2;
    delay(UPDATE_INTERVAL);
}

//调整到目标温度后gap >1 才调整
void pidTemControl() {
  static double offset = 0;

  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  double  current_temp = temp.temperature;

  Input = current_temp;

  double error = Setpoint - Input;

  volatile double  gap = abs(error); //distance away from setpoint

  if (gap < 0.2) {
    offset = 1.5;
  }

  if (gap <= offset) {
    Serial.println("(gap<=offset  gap:" + (String)gap + " offset:" + (String)offset);
    coolerOff();
    //digitalWrite(coolerPin, LOW);
    heaterOff();
    digitalWrite(heaterPin, LOW);
    fanOff(COOLER_PWM_PIN);
    fanOff(HEATER_PWM_PIN);
    return;
  }

  //走到这里说明 gap>1.5 了，就重置T 为0 ，这样小的误差也会走过去
  offset = 0;
  Serial.println("current_temp:" + (String)current_temp+ " gap:" + (String)gap + " offset:" + (String)offset);

  if (gap < 1.5)
  { 
    //距离目标很近，使用分段的保守的PID参数
    myPID.SetTunings(consKp, consKi, consKd);
  } else{
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();

  if (millis() - windowStartTime > WindowSize){ windowStartTime += WindowSize;}

  double elapsed =  millis() - windowStartTime;


  //加热
  if ((Setpoint > current_temp) && (Output > elapsed)) {
    Serial.println("heater and Output:" + (String)Output+" elapsed:" + (String)elapsed);
    coolerOff();
    fanOff(COOLER_PWM_PIN);

    heaterOpen();
    fanOpenWithPWMPulseRatio(HEATER_PWM_PIN, 120);//48%

  }//制冷
  else if ((Setpoint < current_temp) &&  (abs(Output) > elapsed)) 
  {
    Serial.println("cooler and Output:" + (String)Output+" elapsed:" + (String)elapsed);
    heaterOff();
    fanOff(HEATER_PWM_PIN);

    coolerOpen();
    fanOpenWithPWMPulseRatio(COOLER_PWM_PIN, 130);//48%
    fanOpenWithPWMPulseRatio(HEATER_PWM_PIN, 150);//48%
  } else if (abs(Output) < elapsed) {
    Serial.println("(Output < millis() - windowStartTime :output:" + (String)Output + " current_temp:" + (String)current_temp+ " gap:" + (String)gap + " offset:" + (String)offset);

    coolerOff();
    heaterOff();

    fanOff(COOLER_PWM_PIN);
    fanOff(HEATER_PWM_PIN);
  }
}


void Displaytemp(int  targetTemp, int process, unsigned long activeTimeMills, unsigned long totaltimeMills)
{
  char tempStr[5];
  char humidityStr[5];

  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  dtostrf(temp.temperature, 2, 1, tempStr);
  dtostrf(humidity.relative_humidity, 2, 1, humidityStr);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);

  display.print("tC " + (String)targetTemp);
  display.println( " eC " + (String)tempStr);

  display.println("humidity:" + (String)humidityStr + " %RH");

  display.println("proc:" + (String)process);

  display.print("t:" + (String)(totaltimeMills / 1000) + "m");
  display.print("  act:" + (String)(activeTimeMills / 1000) + "m");

  display.display();

}

void heaterOpen(){
  digitalWrite(heaterPin, HIGH);
  heaterStatus=1;
}

void heaterOff(){
  digitalWrite(heaterPin, LOW);
  heaterStatus=0;
}

void coolerOpen(){
  digitalWrite(coolerPin, HIGH);
  coolerStatus=1;
}

void coolerOff()
{
  digitalWrite(coolerPin, LOW);
  coolerStatus=0;
}

unsigned long CalculateRPM(int pin) {
  unsigned long htime = pulseIn(pin, HIGH);
  unsigned long  rpm = (1000000 * 60) / (htime * 4);
  return rpm;
}


float get18b20tempC() {
  // Serial.println("发起温度转换");
  sensors.requestTemperatures(); //向总线上所有设备发送温度转换请求，默认情况下该方法会阻塞
  float tempC = sensors.getTempCByIndex(0); //获取索引号0的传感器摄氏温度数据
  //if (tempC != DEVICE_DISCONNECTED_C) //如果获取到的温度正常
  return tempC;
}
