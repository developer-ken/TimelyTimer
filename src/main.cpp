#include <Arduino.h>
#include <PCF2131_I2C.h>
#include <SPI.h>

#include "pinout.h"

const uint16_t ADC_FULLSCALE = 4095;       // 12位ADC
const uint16_t ADC_POWER_FULL = 3400;      // 电量全满阈值，电容端4.800V  大于则允许无线通信、高帧动画
const uint16_t ADC_POWER_ENOUGH = 3187;    // 电量充足阈值，电容端4.499V  大于则关闭射频，允许秒级刷新
const uint16_t ADC_POWER_LOW = 2479;       // 电量偏低阈值，电容端3.500V  大于则关闭射频，允许分钟级刷新
const uint16_t ADC_POWER_DANGEROUS = 1771; // 电量危险阈值，电容端2.500V  大于则关闭射频，允许小时级刷新
                                           // 否则关闭射频、关闭屏幕，尽量维持最大功率点跟踪装置运行

PCF2131_I2C rtc;
uint16_t adc_value;

void setup()
{
    // RTC发来EN信号，主控唤醒
    // 执行节能初始化
    {
        // 检查电源情况
        pinMode(PIN_ADC, ANALOG);
        adc_value = analogRead(PIN_ADC);

        Wire.begin(PIN_SDA, PIN_SCL, 5000);

        if (adc_value > ADC_POWER_DANGEROUS) // 剩余电量允许主控运行
        {
            // 立即以开漏模式锁定NINTA，保证EN信号存在
            // 自锁，保持主控运行
            pinMode(PIN_NINTA, OUTPUT_OPEN_DRAIN);
            digitalWrite(PIN_NINTA, LOW);
        }

        // 初始化RTC，释放NINTA
        // 此时若上一步没有锁定NINTA，主控将立即停止运行，直至RTC再次尝试唤醒
        rtc.begin();

        // 运行到这里说明RTC已经成功唤醒主控，且电量足以支撑一些操作。
        Serial.begin(115200);
        SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);

        // 保证每分钟发送一次EN唤醒信号
        rtc.periodic_interrupt_enable(PCF2131_base::EVERY_MINUTE, 0);
        rtc.periodic_interrupt_enable(PCF2131_base::EVERY_MINUTE, 1);
    }

}

void loop()
{
}