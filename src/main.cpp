#include <SPI.h>
#include <Wifi.h>
#include <Arduino.h>
#include <esp_sleep.h>
#include <ArduinoOTA.h>
#include <PCF2131_I2C.h>
#include <JDI_MIP_Display.h>

#include "types.h"
#include "pinout.h"

// const uint16_t ADC_FULLSCALE = 4095;       // 12位ADC
// const uint16_t ADC_POWER_FULL = 3400;      // 电量全满阈值，电容端4.800V  大于则允许无线通信、高帧动画
// const uint16_t ADC_POWER_ENOUGH = 3187;    // 电量充足阈值，电容端4.499V  大于则关闭射频，允许秒级刷新
// const uint16_t ADC_POWER_LOW = 2479;       // 电量偏低阈值，电容端3.500V  大于则关闭射频，允许分钟级刷新
// const uint16_t ADC_POWER_DANGEROUS = 1771; // 电量危险阈值，电容端2.500V  大于则关闭射频，允许小时级刷新
//                                            // 否则关闭射频、关闭屏幕，尽量维持最大功率点跟踪装置运行

const uint16_t ADC_FULLSCALE = 4095;       // 12位ADC
const uint16_t ADC_POWER_FULL = 900;      // 电量全满阈值  大于则允许无线通信、高帧动画
const uint16_t ADC_POWER_ENOUGH = 750;    // 电量充足阈值  大于则关闭射频，允许秒级刷新
const uint16_t ADC_POWER_LOW = 670;       // 电量偏低阈值  大于则关闭射频，允许分钟级刷新
const uint16_t ADC_POWER_DANGEROUS = 0; // 电量危险阈值  大于则关闭射频，允许小时级刷新
                                           // 否则关闭射频、关闭屏幕，尽量维持最大功率点跟踪装置运行

PCF2131_I2C rtc;
JDI_MIP_Display display;

uint16_t adc_value;
RTC_DATA_ATTR V_RUN_LEVEL run_level;
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
        analogSetAttenuation(ADC_11db);
        auto sstmp_runlevel = CheckRunLevel();

        // 如果从完全断电中唤醒，而且电量仍然低于NORMAL，立即停止运行
        // 将会在下一分钟被高精度RTC重新唤醒。
        if (!(run_level == 0 && sstmp_runlevel < V_RUN_LEVEL_NORMAL))
        {
            Wire.begin(PIN_SDA, PIN_SCL, 5000);

            // 立即以开漏模式锁定NINTA，保证EN信号存在
            // 自锁，保持主控运行
            pinMode(PIN_NINTA, OUTPUT_OPEN_DRAIN);
            digitalWrite(PIN_NINTA, LOW);
        }

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

        if (adc_value >= 3500)
        {
            WiFi.begin("DJZ_Debug", "1250542735");
            WiFi.setHostname("timelytimer");
            ArduinoOTA.setHostname("timelytimer");
            ArduinoOTA.begin();
        }

        Serial.begin(115200);
        display.begin();
        display.setRotation(1);
        display.setTextWrap(false);
        SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);

        // 每小时发送一个EN信号
        // rtc.alarm(RTC_NXP::alarm_setting::MINUTE, 0, 0);

        // 每分钟发送一个EN信号
        rtc.periodic_interrupt_enable(PCF2131_base::EVERY_MINUTE, 0);

        while (true)
        {
            Serial.println("EVAL_PWR_LEVEL...");
            auto new_level = CheckRunLevel();
            if (new_level <= V_RUN_LEVEL_YOLO)
            {
                run_level = new_level;
            }
            else if (run_level < V_RUN_LEVEL_NORMAL)
            {
                run_level = V_RUN_LEVEL_NORMAL;
            }
            else
            {
                run_level = new_level;
            }
            switch (run_level)
            {
            case V_RUN_LEVEL_STOP:
            {
                Serial.println("V_RUN_LEVEL_STOP");
                Serial.flush();
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
                esp_sleep_enable_timer_wakeup(60 * 1000000);
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
                esp_sleep_enable_timer_wakeup(1 * 1000000);
                ScreenUpdate();
                Serial.println("holding");
                Serial.flush();
                // 进入深度休眠，保持屏幕显示状态

                Serial.println(gpio_hold_en((gpio_num_t)PIN_NINTA));
                Serial.println(gpio_hold_en((gpio_num_t)PIN_DISP));
                gpio_deep_sleep_hold_en();
                Serial.flush();
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
                    if (millis() - last_time > 500)
                    {
                        if (CheckRunLevel() != V_RUN_LEVEL_YOLO)
                        {
                            break;
                        }
                        last_time = millis();
                    }
                    if (adc_value >= 3450)
                    {
                        ArduinoOTA.handle();
                    }
                    else
                    {
                        if (WiFi.status() != WL_STOPPED)
                        {
                            WiFi.mode(WIFI_OFF);
                            Serial.println("WiFi OFF");
                        }
                    }
                    yield();
                }
            }
            break;
            }
            Serial.println("ENDSWITCH");
        }
    }
}

void loop()
{
    rtc.int_clear();
    rtc.alarm_clear();
    ArduinoOTA.handle();
    delay(500);
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