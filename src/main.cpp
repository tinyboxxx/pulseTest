#include "driver/pcnt.h" // ESP32 library for pulse count
#include "soc/pcnt_struct.h"
#include "Arduino.h"

#define PCNT_FREQ_UNIT PCNT_UNIT_0 //选择 ESP32 脉冲计数器单元 0（从0到7。ESP32有8个独立计数单元，-S3有4个）
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
// https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/hw-reference/chip-series-comparison.html

int INPUT_PIN = 15; // 选择计数器的输入引脚

int16_t PulseCounter = 0;     //脉冲计数器，最大值值为 65536
int OverflowCounter = 0;      //脉冲计数器的溢出次数
int PCNT_H_LIM_VAL = 10000;   //设置计数上限。当达到这个值时，将溢出次数+1。这个值的最大值为32767。
uint16_t PCNT_FILTER_VAL = 0; //滤波（阻尼，惯性）值以避免计数中的毛刺，最大值 1023

pcnt_isr_handle_t user_isr_handle = NULL; // 中断处理程序 -未使用
// hw_timer_t *timer = NULL;                 // timer实例

void IRAM_ATTR CounterOverflow(void *arg)
{                                         //脉冲计数器溢出中断
  OverflowCounter = OverflowCounter + 1;  //溢出次数+1
  PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT); //清除溢出flag
  pcnt_counter_clear(PCNT_FREQ_UNIT);     //脉冲计数器单元归零和复位
}

void initPulseCounter()
{                                                //初始化脉冲计数器
  pcnt_config_t pcntFreqConfig = {};             //脉冲计数器实例
  pcntFreqConfig.pulse_gpio_num = INPUT_PIN;     //脉冲计数器的引脚分配
  pcntFreqConfig.pos_mode = PCNT_COUNT_INC;      //将上升沿计数（从低逻辑电平变为高逻辑电平）作为脉冲
  pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL; //设置计数上限
  pcntFreqConfig.unit = PCNT_FREQ_UNIT;          //选择 ESP32 脉冲计数器单元 0
  pcntFreqConfig.channel = PCNT_CHANNEL_0;       //选择脉冲计数器单元0的通道0
  pcnt_unit_config(&pcntFreqConfig);             //配置脉冲计数器的寄存器

  pcnt_counter_pause(PCNT_FREQ_UNIT); //暂停脉冲计数器单元
  pcnt_counter_clear(PCNT_FREQ_UNIT); //脉冲计数器单元归零和复位

  pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);             //达到计数上限时启用中断事件
  pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle); //配置寄存器溢出中断处理程序
  pcnt_intr_enable(PCNT_FREQ_UNIT);                              //启用溢出中断

  pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL); //设置滤波数字
  pcnt_filter_enable(PCNT_FREQ_UNIT);                     //启用滤波

  pcnt_counter_resume(PCNT_FREQ_UNIT); //恢复脉冲计数器单元的计数
}

void Read_Reset_PCNT()
{                                                        //读取脉冲计数器的函数（用于定时器）
  pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter); //获取脉冲计数器值 -最大值为 16 位

  //为了示例，下面重置计数器，非必需，按需要调用
  OverflowCounter = 0;                //将溢出次数设置为零
  pcnt_counter_clear(PCNT_FREQ_UNIT); //脉冲计数器单元归零和复位
}

void Read_PCNT()
{                                                        //读取脉冲计数器的函数（用于定时器）
  pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter); //获取脉冲计数器值 -最大值为 16 位
}

void Print_PCNT(void *pvParameters) // 用于打印计数器的函数，仅测试用，可删掉
{
  while (1)
  {
    Read_PCNT();
    Serial.print(PulseCounter);
    Serial.print("\t");
    Serial.print(OverflowCounter);
    Serial.print("\n");
    vTaskDelay(233 / portTICK_PERIOD_MS); //延迟233毫秒，防止出现整数
  }
}

void setup()
{
  initPulseCounter(); //初始化脉冲计数器
  Serial.begin(115200);
  xTaskCreatePinnedToCore(   //创建打印计数器任务
      Print_PCNT,            // function to be executed
      "Print_PCNT",          // name of the task
      2048,                  // stack size
      NULL,                  // parameter passed to the function
      1,                     // priority
      NULL,                  // task handle
      0);                    // core where task should run
  pinMode(INPUT_PIN, INPUT); //设置脉冲计数器的引脚为输入
  pinMode(16, OUTPUT);       //设置测试用的输出引脚
}

void loop()
{
  digitalWrite(16, HIGH); // turn on
  delay(1);
  digitalWrite(16, LOW); // turn off
  delay(2);
}