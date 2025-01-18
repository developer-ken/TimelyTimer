#include <SPI.h>
#include <Arduino.h>
#include <esp_sleep.h>
#include <PCF2131_I2C.h>
#include <JDI_MIP_Display.h>

#include "types.h"
#include "pinout.h"

const uint16_t ADC_FULLSCALE = 4095;       // 12位ADC
const uint16_t ADC_POWER_FULL = 3400;      // 电量全满阈值，电容端4.800V  大于则允许无线通信、高帧动画
const uint16_t ADC_POWER_ENOUGH = 3187;    // 电量充足阈值，电容端4.499V  大于则关闭射频，允许秒级刷新
const uint16_t ADC_POWER_LOW = 2479;       // 电量偏低阈值，电容端3.500V  大于则关闭射频，允许分钟级刷新
const uint16_t ADC_POWER_DANGEROUS = 1771; // 电量危险阈值，电容端2.500V  大于则关闭射频，允许小时级刷新
                                           // 否则关闭射频、关闭屏幕，尽量维持最大功率点跟踪装置运行

PCF2131_I2C rtc;
JDI_MIP_Display display;

uint16_t adc_value;
V_RUN_LEVEL run_level;
bool WAKEUP_MINUTE = false,
     WAKEUP_ALARM = false,
     WAKEUP_BAT_LOW = false;

void ScreenUpdate();
void ScreenUpdateYolo();
V_RUN_LEVEL CheckRunLevel();

void setup()
{
    // RTC发来EN信号，主控唤醒
    // 执行节能初始化
    {
        pinMode(PIN_NINTB, INPUT);

        // 检查电源情况
        pinMode(PIN_ADC, ANALOG);
        analogSetAttenuation(ADC_2_5db);
        Wire.begin(PIN_SDA, PIN_SCL, 5000);

        // 立即以开漏模式锁定NINTA，保证EN信号存在
        // 自锁，保持主控运行
        pinMode(PIN_NINTA, OUTPUT_OPEN_DRAIN);
        digitalWrite(PIN_NINTA, LOW);

        // 初始化RTC，释放NINTA
        // 此时若上一步没有锁定NINTA，主控将立即停止运行，直至RTC再次尝试唤醒
        uint8_t int_stat[3];
        rtc.alarm_clear();
        rtc.int_clear(int_stat);
        if (int_stat[0] & 0x80)
            WAKEUP_MINUTE = true;
        if (int_stat[0] & 0x10)
            WAKEUP_ALARM = true;
        if (int_stat[1] & 0x04)
            WAKEUP_BAT_LOW = true;

        // 运行到这里说明RTC已经成功唤醒主控，且电量足以支撑一些操作。
        Serial.begin(115200);
        display.begin();
        display.setRotation(1);
        // display.setTextWrap(false);
        SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);

        // 每小时发送一个EN信号
        rtc.alarm(RTC_NXP::alarm_setting::MINUTE, 0, 0);

        // 第四位置1，使用GPIO4唤醒
        esp_deep_sleep_enable_gpio_wakeup(0b111111, ESP_GPIO_WAKEUP_GPIO_LOW);
        while (true)
        {
            switch (run_level = CheckRunLevel())
            {
            case V_RUN_LEVEL_STOP:
            {
                Serial.println("V_RUN_LEVEL_STOP");
                Serial.flush();
                // 只在整点唤醒检测电量，避免频繁唤醒
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 0);
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 1);
                // 释放NINTA，主控停止工作，关闭屏幕显示，等待下次唤醒
                gpio_hold_dis((gpio_num_t)PIN_NINTA);
                gpio_hold_dis((gpio_num_t)PIN_DISP);
                display.displayOff();
                digitalWrite(PIN_NINTA, HIGH);
            }
            break;
            case V_RUN_LEVEL_SUPERHUNGRY:
            {
                Serial.println("V_RUN_LEVEL_SUPERHUNGRY");
                Serial.flush();
                // 每分钟唤醒检测一次电量，不刷新屏幕
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 0);
                rtc.periodic_interrupt_enable(PCF2131_base::EVERY_MINUTE, 1);
                // 进入深度休眠，保持屏幕显示状态
                gpio_hold_en((gpio_num_t)PIN_NINTA);
                gpio_hold_en((gpio_num_t)PIN_DISP);
                gpio_deep_sleep_hold_en();
                esp_deep_sleep_start();
            }
            break;
            case V_RUN_LEVEL_LITTLEHUNGRY:
            {
                Serial.println("V_RUN_LEVEL_LITTLEHUNGRY");
                Serial.flush();
                // 每分钟唤醒系统刷新屏幕
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 0);
                rtc.periodic_interrupt_enable(PCF2131_base::EVERY_MINUTE, 1);
                ScreenUpdate();
                // 进入深度休眠，保持屏幕显示状态
                gpio_hold_en((gpio_num_t)PIN_NINTA);
                gpio_hold_en((gpio_num_t)PIN_DISP);
                gpio_deep_sleep_hold_en();
                esp_deep_sleep_start();
            }
            break;
            case V_RUN_LEVEL_NORMAL:
            {
                Serial.println("V_RUN_LEVEL_NORMAL");
                Serial.flush();
                // 每秒唤醒系统刷新屏幕
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 0);
                rtc.periodic_interrupt_enable(PCF2131_base::EVERY_SECOND, 1);
                ScreenUpdate();
                // 进入深度休眠，保持屏幕显示状态
                Serial.println(gpio_hold_en((gpio_num_t)PIN_NINTA));
                gpio_hold_en((gpio_num_t)PIN_DISP);
                gpio_deep_sleep_hold_en();
                esp_deep_sleep_start();
            }
            break;
            case V_RUN_LEVEL_YOLO:
            {
                Serial.println("V_RUN_LEVEL_YOLO");
                Serial.flush();
                // 持续刷新屏幕
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 0);
                rtc.periodic_interrupt_enable(PCF2131_base::DISABLE, 1);
                gpio_hold_dis((gpio_num_t)PIN_DISP);
                display.displayOn();
                display.clearScreen();
                unsigned long last_time = millis();
                while (true)
                {
                    ScreenUpdateYolo();
                    if (millis() - last_time > 2000)
                    {
                        if (CheckRunLevel() != V_RUN_LEVEL_YOLO)
                        {
                            break;
                        }
                        last_time = millis();
                    }
                    yield();
                }
            }
            break;
            }
            yield();
        }
    }
}

void loop()
{
    rtc.int_clear();
    rtc.alarm_clear();
    delay(3600);
}

void ScreenUpdate()
{
    gpio_hold_dis((gpio_num_t)PIN_DISP);
    display.displayOn();
    ScreenUpdateYolo();
}

void ScreenUpdateYolo()
{
    display.setTextSize(2);
    display.fillRect(0, 0, 2 * 5 * 11, 2 * 7, COLOR_RED);
    display.fillRect(2 * 5 * 11, 0, 2 * 6 * 1, 2 * 7, COLOR_BLUE);
    display.setCursor(0, 0);
    display.print(rtc.rtc_time());
    display.setCursor(2 * 5 * 11, 0);
    display.print(run_level);
    // display.drawPixel(1, 1, COLOR_GREEN);
    display.refresh();
    // display.refresh();
}

V_RUN_LEVEL CheckRunLevel()
{
    adc_value = analogRead(PIN_ADC);
    Serial.println(adc_value);
    if (adc_value > ADC_POWER_FULL)
        return V_RUN_LEVEL_YOLO;
    else if (adc_value > ADC_POWER_ENOUGH)
        return V_RUN_LEVEL_NORMAL;
    else if (adc_value > ADC_POWER_LOW)
        return V_RUN_LEVEL_LITTLEHUNGRY;
    else if (adc_value > ADC_POWER_DANGEROUS)
        return V_RUN_LEVEL_SUPERHUNGRY;
    else
        return V_RUN_LEVEL_STOP;
}