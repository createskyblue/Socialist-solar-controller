/*
   备注:本控制器主要任务 0.保证自身运行 1.定时浇灌 2.歌颂社会主义
                          几乎没有电力   少量电力  大量闲余电力
   按照系统电压为标准        <4.22       >=4.25V    >=4.5
   程序优先级从低到高
*/
/*==================================================
   库文件
  ==================================================== */
#include <LiquidCrystal.h>  //LCD
#include <avr/sleep.h>        //省电
#include <avr/wdt.h>          //看门狗
/*==================================================
   变量或固定初始值
  ==================================================== */
volatile byte ST = 100; //睡眠计数器 加入修饰指令volatile是为了不被编译器忽略 100为禁用休眠
double temp;
double SVol = 5.0; //系统电压
double NVol = 5.0; //标准电压
#define ntc A2 //NTC热敏传感器 这个是固定在板子上
/*
   LCD1602 显示设置
*/
#define LCD_BG 13 //屏幕背光灯
#define RS 8
#define E 9
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define SRI 1500 //屏幕刷新间隔 单位ms
unsigned long SC, SCT;
byte SCP; //显示周期 刷新间隔 以及显示页数
/*
   NTC参数
*/
#define R1 48.6 //NTC串联的电阻大小
#define B 3950 //NTC的B值 重要参数
#define NTCTR 10 //NTC材料在25度时的电阻 单位千欧
/*
   太阳能电池板参数
*/
#define Apin A3 //没有电流的时候为系统电压的一半，然后每1A电压上升100mv  警告：电流不可以超过25A 否则烧坏电流传感器
#define Vpin A1 //采用5个10K电阻分压 电压值要*5                          警告：电流不可以超过25V 否则烧坏单片机
#define IC 10 //装机容量 单位W                                           警告：设计上装机容量峰值不可以超过625W 否则将会烧毁控制器 建议运行在低于500W的系统中
double Amp, Vol, W; //电池板输出电流 电压 以及功率
unsigned long TW, TWT; //总共发电量 大于1 以及上次计算发电量的时间
double TWF; //小数发电量
/*
   浇花系统
*/
#define relay 12 //继电器引脚
#define WWD 900 //等待浇花延迟 单位秒
#define WT 20 //浇花时长 单位秒
byte TB = 0, TBC =   2; //0为晚上 1为早上 2为初始化
unsigned long TO, WO; //浇花执行时间 以及浇花时间 0为禁用
/*
   东方红乐音
*/
#define BZ 11
#define L1 131
#define L2 147
#define L3 165
#define L4 175
#define L5 196
#define L6 220
#define L7 247
#define M1 262
#define M2 294
#define M3 330
#define M4 349
#define M5 392
#define M6 440
#define M7 497
#define H1 523
#define H2 587
#define H3 659
#define H4 698
#define H5 784
#define H6 880
#define H7 988
byte PSP; //播放进度
byte PSS = 255; //播放曲目 255为禁用播放
unsigned long PST; //播放某个音符的时间
const int melody[] PROGMEM = {  //音符表
  M5, M5, M6, M2, M1, M1, L6, M2, M5, M5, M6, H1, M6, M5, M1, M1,
  L6, M2, M5, M2, M1, L7, L6, L5, M5, M2, M3, M2, M1, M1, L6, M2,
  M3, M2, M1, M2, M1, L7, L6, L5, L5, M5, M5, M6, M2, M1, M1, L6,
  M2, M5, M5, M6, H1, M6, M5, M1, M1, L6, M2, M5, M2, M1, L7, L6,
  L5, M5, M2, M3, M2, M1, M1, L6, M2, M3, M2, M1, M2, M1, L7, L6,
  L5, L5, M5, M5, M6, M2, M1, M1, L6, M2, M5, M5, M6, H1, M6, M5,
  M1, M1, L6, M2, M5, M2, M1, L7, L6, L5, M5, M2, M3, M2, M1, M1,
  L6, M2, M3, M2, M1, M2, M1, L7, L6, L5, L5, 0, M5, M2, M1, L7,
  L6, L5, M5, M2, M3, M2, M1, M1, L6, M2, M3, M2, M1, H2, H1, M7,
  M6, M5, M5
};
const uint8_t noteDurations[] PROGMEM = { //音符时长表
  1125, 562, 562, 2250, 1125, 562, 562, 2250, 1125, 1125, 562, 562, 562, 562, 1125, 562,
  562, 2250, 1125, 1125, 1125, 562, 562, 1125, 1125, 1125, 562, 562, 1125, 562, 562, 562,
  562, 562, 562, 562, 562, 562, 562, 1687, 1687, 1152, 562, 562, 2250, 1125, 562, 562,
  2250, 1125, 1125, 562, 562, 562, 562, 1125, 562, 562, 2250, 1125, 1125, 1125, 562, 562,
  1125, 1125, 1125, 562, 562, 1125, 562, 562, 562, 562, 562, 562, 562, 562, 562, 562,
  1728, 1728, 1125, 562, 562, 2250, 1125, 562, 562, 2250, 1125, 1125, 562, 562, 562, 562,
  1125, 562, 562, 2250, 1125, 1125, 1125, 562, 562, 1125, 1125, 1125, 562, 562, 1125, 562,
  562, 562, 562, 562, 562, 562, 562, 562, 562, 1728, 1728,
  1125, 1125, 1125, 1125, 562, 562, 1125, 1125, 1125, 562, 562, 1125, 562, 562, 562, 562,
  562, 562, 562, 562, 562, 562, 1728, 1728
};
bool PSD = false;
unsigned long PSDT; //某个间隔开始的时候
/*==================================================
   启动初始项目
  ==================================================== */
LiquidCrystal lcd(RS, E, D4, D5, D6, D7); //LCD1602引脚

/*==================================================
   只循环一次
  ==================================================== */
void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2); //初始化LCD
  pinMode(LCD_BG, OUTPUT); //屏幕背光
  pinMode(relay, OUTPUT); //初始化控制继电器控制引脚
  digitalWrite(relay, LOW); //防止进入程序后几秒内继电器处于开启状态 耗费电力
  pinMode (BZ, OUTPUT);  //初始化东方红乐音播放引脚
  setup_watchdog(5);  //设置看门狗
  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  //ACSR |= _BV(ACD); //OFF ACD
  //ADCSRA = 0; //OFF ADC
  //Sleep_avr();//Sleep_Mode
}
/*==================================================
   不断循环
  ==================================================== */
void loop()
{
  if (ST >= 7) {  //注意 看门狗中断被配置为8秒叫一次  休眠结束的时间为8*ST
    SVol = readVcc() / 1000.0; //计算系统电压
    /*==================================================
      LCD显示
      ==================================================== */
      if (millis() >= SCT + SRI) {
        lcd.clear();
        SCT = millis(); //重置计时器
        switch (SC) {
          case 0:
            lcd.setCursor(0, 0);
            lcd.print(F("TMP: "));
            lcd.print(float(temp), 2);
            lcd.print(F(" C"));
            lcd.setCursor(0, 1);
            lcd.print(F("RT: "));
            lcd.print(millis() / 1000);
            lcd.print(F(" s"));
            break;
          case 1:
            lcd.setCursor(0, 0);
            lcd.print(F("V: "));
            lcd.print(float(Vol), 2);
            lcd.print(F(" "));
            lcd.print(F("A: "));
            lcd.print(float(Amp), 2);
            lcd.setCursor(0, 1);
            lcd.print(F("W: "));
            lcd.print(float(W), 2);
            lcd.print(F("  "));
            lcd.print(F("U: "));
            lcd.print(int((W / IC) * 100));
            lcd.print(F("%"));
            break;
          case 2:
            lcd.setCursor(0, 0);
            lcd.print(F("Total: "));
            lcd.print(TW);
            lcd.setCursor(0, 1);
            lcd.print(TWF, 15);
            break;
          case 3:
            lcd.setCursor(0, 0);
            lcd.print(F("SystemVol: "));
            lcd.print(SVol);
            break;
          case 4:
            lcd.setCursor(0, 0);
            lcd.print(F("TO: "));
            lcd.print(TO);
            lcd.print(F(" TB: "));
            lcd.print(TB);
            lcd.setCursor(0, 1);
            lcd.print(F("WO: "));
            lcd.print(WO);
            lcd.print(F(" TBC: "));
            lcd.print(TBC);
            /*   lcd.setCursor(0, 0);
               lcd.print(F("SystemVol: "));
               lcd.print(SVol);*/
            break;
        }
        SC++;
        if (SC >= 5) SC = 0;
      }
    /*==================================================
      NTC温度计算
      ==================================================== */
    float hq = NVol * (analogRead(ntc)) / 1024 ; // 读取传感器模拟值
    float x = R1 * (hq / (5.0 - hq)); //计算当前的电阻值 测量用的辅助电阻阻值在*前 单位kΩ
    float hs = log(x / NTCTR); //计算NTC对应阻值的对数值  10为25度时NTC的电阻
    temp = 1 / ( hs / B + 1 / 298.15) - 273.15; //计算当前的温度值
    /*==================================================
      太阳能电池板输出计算
      ==================================================== */
    /*
       计算基本参数
    */

    Vol = SVol * 5 * (analogRead(Vpin) / 1023.0);
    if (Vol > 0.15)  Amp = ((NVol) * (analogRead(Apin) / 1023.0) - ((NVol) / 2)) / 0.1;
    if (Amp < 0) Amp = 0;  //喂：传感器接反了吧 电表倒转啊
    W = Amp * Vol;
    /*
       防止错误的抖动 和 系统背光 以及是否需要休眠
    */
    // SVol = 3.0; //模拟欠压
    if (Vol <= 0.15 || SVol < 4.22) {
      Amp = 0; //不符合实际 的电流
      if (SVol >= 4.22) {
        digitalWrite(LCD_BG, HIGH); //假若系统有一定的电量
        ST = 100; //重置看门狗计时器
      } else {
        //系统低电压 进入睡眠
        ST = 0; //重置睡眠模式计时器 *必须 不可省略语句
        Sleep_avr();  //休眠
      }
    } else {
      if (temp <= -35 || temp >= 80) {
        //过热 进入休眠状态
        ST = 0; //重置睡眠模式计时器 *必须 不可省略语句
        Sleep_avr();  //休眠
      } else {
        ST = 100; //重置看门狗计时器
        digitalWrite(LCD_BG, LOW);
      }
    }
    /*
       计算发电量
    */
    TWF += ((millis() - TWT) / 3600000.0) * W;
    TWT = millis(); //重新计时
    while (TWF >= 1) {
      TWF--;
      TW++;
    }
    /*==================================================
      浇花系统计算
      ==================================================== */
    /*
       判定光照状态以及当前时间状态为AM还是PM
    */
    if (Vol < 5) {
      TBC = 0;
      if (TBC != TB) {
        PSS = 255; //漆黑的夜晚 资本主义的世界
        TB = 0;
        TO = millis() / 1000 + 1;
      }
    } else if (Vol > 15) {
      TBC = 1;
      if (TBC != TB) {
        if (SVol >= 4.25) {
          //确保系统有足够的能源启动继电器
          if (SVol >= 4.5) {
            //确保系统真的有能源歌颂社会主义
            PSP = 0; //设置播放地址为0  地址0为东方红开始的位置
            PSS = 0; //启用播放器
          }
          TB = 1;
          TO = millis() / 1000 + 1;
        }
      }
    }
    /*
       浇花控制程序
    */
    if (TO != 0 && millis() / 1000.0 >= TO + WWD) {
      if (WO == 0)  WO = millis() / 1000;
      if (WO != 0 && millis() / 1000.0 <= WO + WT) {
        digitalWrite(relay, LOW);
      } else {
        WO = 0;
        TO = 0;
        digitalWrite(relay, HIGH);
      }
    }
    if (WO == 0)  digitalWrite(relay, HIGH);


    /*
       音乐播放器
    */
    playsound();
    wdt_reset();
  } else {
    //还没到睡醒的时间
    Sleep_avr();  //休眠
  }
}
/*==================================================
  读取系统电压
  ==================================================== */
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  analogReference(DEFAULT);  //设置为基准电压为标准
  return result;
}
/*==================================================
  没电的时候进入睡眠模式
  ==================================================== */
void Sleep_avr() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //设置睡眠模式
  sleep_enable();                        //开启睡眠模式
  sleep_mode();                          //系统进入休眠状态
}
/*==================================================
  配置看门狗
  ==================================================== */
void setup_watchdog(int ii) {
  byte bb; //局部变量
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  MCUSR &= ~(1 << WDRF);
  // 开始时间序列
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // 设置新的看门狗超时值
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
/*==================================================
  看门狗中断
  ==================================================== */
ISR(WDT_vect) {
  ++ST; //计数器++
  wdt_reset();
}
/*=========================================================
                   声音播放器
  =========================================================*/
void playsound() {
  //检测播放地址是否合法
  if (PSS != 255) {  //如果曲目是255 *注意PSS=255时关闭音乐
    if (PSS == 0) {  //曲目1 东方红
      if (PSP >= 148) {  //检测到播放位置溢出 重置位置到曲目开头
        PSP = 0;  //设置当前曲目在音符表的开始地址
        PSS = 255; //停止播放
        PST = millis() / 10.0; //记录开始播放该位置音符的时间
      }
    }
    if (pgm_read_word_near(&melody[PSP]) != 0 && PSD == false) {
      tone(BZ, pgm_read_word_near(&melody[PSP]));
    } else {
      noTone(BZ); //禁用
    }
    //检测是否播放完一个音节
    if (millis() / 10.0 >= PST + (pgm_read_byte_near(&noteDurations[PSP]))) {
      //该PSP位置音符播放完毕，加地址
      if (PSD == false) {
        //播放完音符
        PSD = true;
        PSDT = millis();
      } else if (millis() >= PSDT + 15) { //间隔单位ms
        //如果超过间隔的时间间隔完成
        PSD = false;
        PSP++; //音乐播放位置加一
        PST = millis() / 10.0; //重置播放音符时间
      }
    }
  } else noTone(BZ); //禁用
}
