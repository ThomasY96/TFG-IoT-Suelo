#include <Arduino.h>

// ** !!! 必须定义 XPOWERS_CHIP_AXP2102，确保 XPowersLib.h 能正确工作!!! **
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"    // PMU 电源管理

#include "utilities.h"     // 开发板引脚定义
#include <Wire.h>          // I2C 通信（PMU）
#include <FS.h>
#include <SD_MMC.h>        // SD 卡库
#include <esp_bt.h>        // 关闭蓝牙以节省功耗
#include <esp_wifi.h>      // WiFi 省电管理
#include <WiFi.h>          // WiFi 连接库
#include "time.h"          // NTP 时间同步
#include <PubSubClient.h>  //MQTT 库

// **WiFi 连接信息**
WiFiClient espClient;                 // WiFi 客户端
#define WIFI_SSID "Livebox6-8B92"         // 你的 WiFi 名称
#define WIFI_PASSWORD "xhbFN5FDZ7J7" // 你的 WiFi 密码
#define TIMEOUT_WIFI 20000            // WiFi 连接超时时间 (20 秒)

// **MQTT 服务器信息**
PubSubClient client(espClient);       // MQTT 客户端
String dataString;                    // 传感器数据字符串
#define MQTT_SERVER "192.168.1.100"   // MQTT 服务器 IP
#define MQTT_PORT 1883                // MQTT 端口号
#define MQTT_USER "user1"         // MQTT 用户名
#define MQTT_PASSWORD "user1" // MQTT 密码
#define MQTT_TOPIC "sensor/data"      // MQTT 主题
#define TIMEOUT_MQTT 10000            // MQTT 连接超时（10 秒）
bool boolcallback;          // MQTT 回调标志

// **NTP 服务器信息**
struct tm timeinfo;                // 时间信息结构体
#define NTP_SERVER "pool.ntp.org"  // NTP 服务器
#define GMT_OFFSET_SEC (1 * 3600)  // GMT+1（需要调整时区）
#define DAYLIGHT_OFFSET_SEC 0      // 夏令时调整（无夏令时则为 0）
bool timeSynced = false;  // 时间同步标志

// **深度睡眠相关**
#define SLEEP_TIME_SECONDS 900  // 15分钟（900秒）
uint64_t uS_TO_S_FACTOR = 1000000;  // 微秒转换为秒
esp_sleep_wakeup_cause_t wakeup_reason; // **ESP32 睡眠唤醒计时器

// **传感器数据采集**
#define TIME_OUT_SENSOR 2000  // 传感器读取超时时间（2 秒）
#define RS485_RXD_PIN 16      // RS485 传感器接收引脚
#define RS485_TXD_PIN 17      // RS485 传感器发送引脚
#define RS485_DIR1 4          // RS485 方向控制引脚1
#define RS485_DIR2 5          // RS485 方向控制引脚2
float sensores[7];
unsigned long timeout = TIME_OUT_SENSOR;

// **SD 卡存储**
#define LOG_FILE "/log.txt" // 存储日志文件
String fich_log = LOG_FILE;
String file_name;
bool sd_initialized = false;

// **PMU 电源管理对象**
XPowersPMU PMU;
String getBattVoltage;      // 电池电压信息

void setup_wifi();
void setup_ntp();
bool logData(bool connection);
bool wifi_mqtt_connect();
void print_wakeup_reason();
bool setup_sd_card();
bool appendFile(const char *path, const char *message);
uint32_t calcula_offset();
String get_formatted_time();

void setup()
{   
    // **1️. Initialize the serial port**
    Serial.begin(115200);

    delay(5000);  // Wait for 5 seconds to observe the serial port output
    Serial.println("\n🚀 Booting...");

    // **2️. 关闭蓝牙省电**
    esp_bt_controller_disable();
    
    // **3️. 打印 ESP32 唤醒原因**
    print_wakeup_reason();

    // **4️. 初始化 PMU 电源管理**
    Serial.println("🔋 Initializing PMU...");
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("❌ Failed to initialize PMU!");
        while (1) delay(5000);
    }
    Serial.printf("✅ PMU ID: 0x%x\n", PMU.getChipID());

    // **5️. 设置 VBUS 电压 & 充电管理**
    PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_3V88);   // 设置 VBUS 最低工作电压
    PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA); // 设置 VBUS 最大电流
    PMU.setSysPowerDownVoltage(2600);                             // 系统掉电电压（低于 2.6V 关机）

    // **6️. 关闭低电压自动关机**
    PMU.disableDC5LowVoltageTurnOff();
    PMU.disableDC4LowVoltageTurnOff();

    // **7️. 禁用过温自动关机**
    PMU.disableOverTemperaturePowerOff();

    // **8️. pwr按键设置**
    PMU.enableFastPowerOn(); // 快速开机
    PMU.disableLongPressShutdown(); // 禁用长按关机
    PMU.setLongPressRestart(); // 长按重启

    // **9️. 关闭 TS Pin 检测（否则无法充电）**
    PMU.disableTSPinMeasure();

    // **10. 启用电池检测**
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();

    // **11. 设置充电参数**
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);        // 预充电电流 50mA
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);   // 充电电流 200mA
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA); // 终止电流 25mA
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);     // 充电目标电压 4.2V

    // **1️2. 显示电池状态**
    Serial.println();
    Serial.print("🔋 Is Charging: "); Serial.println(PMU.isCharging() ? "Yes" : "No");
    Serial.print("🔌 VBUS Connected: "); Serial.println(PMU.isVbusIn() ? "Yes" : "No");
    getBattVoltage = String(PMU.getBattVoltage());
    Serial.print("⚡ Battery Voltage: "); Serial.print(getBattVoltage); Serial.println("mV");

    // **13. 关闭不必要的电源通道**
    PMU.disableDC2();
    PMU.disableDC3();
    PMU.disableDC4();
    PMU.disableDC5();

    PMU.disableALDO1();
    PMU.disableALDO2();
    PMU.disableALDO3();
    PMU.disableALDO4();
    PMU.disableBLDO1();
    PMU.disableBLDO2();

    PMU.disableDLDO1();
    PMU.disableDLDO2();
    PMU.disableCPUSLDO();

    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);

    Serial.println("⚡ PMU Setup Complete!");   
}

void loop()
{
    uint32_t offset;
    bool connection = false;
    String error;

    if((PMU.getBattVoltage()>3700) || PMU.isVbusIn()){

        // **1 只有在需要时才连接 WiFi**
        if (WiFi.status() != WL_CONNECTED) {
            setup_wifi();
        }

        // **2 开启 SD 卡电源，不要修改参数 ！！**
        PMU.setALDO3Voltage(3300); 
        PMU.enableALDO3();

        // **3 初始化 SD 卡**
        SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);   // 设置 SD 卡引脚

        if (!sd_initialized) {
            if (!setup_sd_card()) {
                Serial.println("⚠️ SD Card init failed, skipping data logging.");
                return;
            }
        }
        // **4 采集传感器数据**
        delay(20000); // **等待 20 秒稳定传感器数据**
        if (!logData(connection)) {
            error = "❌ LogData error!!!\n";
            Serial.print(error);
            appendFile(fich_log.c_str(), error.c_str());
        } else {
            // **5 连接 WiFi 并上传数据**
            connection = wifi_mqtt_connect(); 

            unsigned long previous = millis();
            boolcallback = true;
            while (((millis() - previous) < TIMEOUT_MQTT) && boolcallback) {
                client.loop();
            }

            Serial.println("⏳ Timeout Client.");
            WiFi.disconnect();
            esp_wifi_stop();
        }

        // **6 计算唤醒时间**
        offset = calcula_offset() + 5;
        Serial.print("⏰ Next wakeup in: ");
        Serial.println(offset);

        // **7 关闭 SD 卡电源**
        SD_MMC.end();
        PMU.disableALDO3();
    }
    else {
        Serial.println("Battery low, offset 300.");
        offset = 300;
    }
    // **8 进入深度睡眠**
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    esp_sleep_enable_timer_wakeup(offset * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

void setup_wifi() {
    String cadena;
    bool a;

    Serial.print("\n🔌 Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long previousMillis = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - previousMillis > TIMEOUT_WIFI) {
            Serial.println("\n❌ WiFi connection failed! Retrying later...");
            delay(10000); // Esperar 10 segundos antes de volver a intentar
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            previousMillis = millis();
        }
    }

    cadena = "Connected to WiFi.\n";
    Serial.print(cadena);
    a = appendFile(fich_log.c_str(),cadena.c_str());

    cadena = "Gateway IP address: \n";
    Serial.print(cadena);
    a = appendFile(fich_log.c_str(),cadena.c_str());

    cadena = WiFi.gatewayIP().toString() + "\n";
    Serial.println(cadena);
    a = appendFile(fich_log.c_str(),cadena.c_str());

    // **WiFi 连接成功后，立即同步 NTP**
    setup_ntp();
}


void setup_ntp() {
    if (timeSynced) return;  // **如果已经同步过时间，就不再执行**
    
    Serial.println("⏳ Synchronizing time with NTP...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    
    if (!getLocalTime(&timeinfo)) {
        Serial.println("⚠️ Failed to obtain time!");
        return;
    }
    
    Serial.println("✅ Time synchronized successfully!");
    Serial.printf("📆 Current Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    timeSynced = true;  // **标记为已同步**
}


bool logData(bool connection) 
{
    uint8_t k, l, indice, car[19];
    uint8_t index = 0;
    unsigned long previous;
    unsigned long timeout = TIME_OUT_SENSOR;
    float sensores[7];
    String minutos;
    String cadena;

    // ** 采集传感器数据**
    Serial2.begin(4800, SERIAL_8N1, RS485_RXD_PIN, RS485_TXD_PIN);
    pinMode(RS485_DIR2, OUTPUT);
    pinMode(RS485_DIR1, OUTPUT);
    digitalWrite(RS485_DIR2, HIGH);
    digitalWrite(RS485_DIR1, HIGH);

    Serial.println("🔍 Sampling...");

    delay(100);
    const byte ec[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
    for (k = 0; k < 8; k++) {
        Serial2.write(ec[k]);
        Serial2.flush();
    }
    delay(1);
    digitalWrite(RS485_DIR2, LOW);
    digitalWrite(RS485_DIR1, LOW);

    // **读取数据**
    k = 0;
    indice = 0;
    previous = millis();
    do {
        if (Serial2.available() != 0) {
            car[k] = Serial2.read();
            Serial.print(car[k]);
            Serial.print(" ");
            if (car[k] == 1) indice = 1;
            if (indice) k++;
        }
        if (millis() < previous) previous = millis();
    } while ((k < 19) && ((millis() - previous) < timeout));

    Serial.println("");

    // **解析数据**
    if (k != 19) {
        cadena = "❌ Timeout data error!!!\n";
        Serial.print(cadena);
        appendFile(fich_log.c_str(), cadena.c_str());
        return false;
    }

    sensores[1] = ((car[3] << 8) | car[4]) * 0.1;
    sensores[0] = ((car[5] << 8) | car[6]) * 0.1;
    sensores[2] = (car[7] << 8) | car[8];
    sensores[3] = ((car[9] << 8) | car[10]) * 0.1;
    sensores[4] = (car[11] << 8) | car[12];
    sensores[5] = (car[13] << 8) | car[14];
    sensores[6] = (car[15] << 8) | car[16];

    Serial.println("✅ Sensor Data Collected!");

    // ** 获取当前时间（使用 NTP）**
    String current_time = get_formatted_time();

    // ** 存储到 SD 卡**
    String dataString = current_time + "," + 
                        String(sensores[0]) + "," + 
                        String(sensores[1]) + "," + 
                        String(sensores[2]) + "," + 
                        String(sensores[3]) + "," + 
                        String(sensores[4]) + "," + 
                        String(sensores[5]) + "," + 
                        String(sensores[6]);

    String file_name = "/log_data.csv";  // 固定的日志文件
    appendFile(file_name.c_str(), dataString.c_str());

    return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("📩 Message received on topic: ");
    Serial.println(topic);

    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.print("📨 Message: ");
    Serial.println(message);

    // **解析 MQTT 指令**
    if (message == "restart") {
        Serial.println("🔄 Restarting ESP32...");
        delay(2000);
        ESP.restart();
    } else if (message == "status") {
        Serial.println("📡 Device is online.");
    }
}


bool wifi_mqtt_connect()
{
    unsigned long previous;
    WiFi.mode(WIFI_STA);

    // 确保 WiFi 连接
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi disconnected. Trying to reconnect...");
        setup_wifi();
    }

    Serial.println("🔄 Connecting to MQTT...");
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(callback);  // 设置 MQTT 消息回调函数

    previous = millis();
    while (!client.connected()) {
        String client_id = "ESP32Client-" + String(WiFi.macAddress());
        if (client.connect(client_id.c_str(), MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("✅ Connected to MQTT!");

            // **订阅 MQTT 主题**
            client.subscribe(MQTT_TOPIC);
            Serial.print("📡 Subscribed to: ");
            Serial.println(MQTT_TOPIC);

            // **构造要上传的数据**
            String mqtt_data = "{";
            mqtt_data += "\"temperature\": " + String(sensores[0]) + ",";
            mqtt_data += "\"humidity\": " + String(sensores[1]) + ",";
            mqtt_data += "\"conductivity\": " + String(sensores[2]) + ",";
            mqtt_data += "\"ph\": " + String(sensores[3]) + ",";
            mqtt_data += "\"nitrogen\": " + String(sensores[4]) + ",";
            mqtt_data += "\"phosphorus\": " + String(sensores[5]) + ",";
            mqtt_data += "\"potassium\": " + String(sensores[6]);
            mqtt_data += "}";

            // **发布数据到 MQTT 服务器**
            client.publish(MQTT_TOPIC, mqtt_data.c_str());
            Serial.println("📤 Data published to MQTT:");
            Serial.println(mqtt_data);

            return true;
        } else {
            if ((millis() - previous) > TIMEOUT_MQTT) {
                Serial.println("❌ Timeout MQTT connection.");
                return false;
            }
        }
    }

    return true;
}


uint32_t calcula_offset()
{
    uint32_t wake_time;
    uint32_t Periodo_muestreo = SLEEP_TIME_SECONDS; // 采样周期 15 分钟 (900秒)
    char timeBuffer[40];

    // **获取当前时间**
    if (!getLocalTime(&timeinfo)) {
        Serial.println("⚠️ Failed to obtain time from NTP!");
        return Periodo_muestreo;  // 如果时间获取失败，默认间隔
    }

    // **计算下一次采样时间**
    uint32_t current_seconds = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    wake_time = Periodo_muestreo - (current_seconds % Periodo_muestreo);

    // **打印唤醒时间**
    snprintf(timeBuffer, sizeof(timeBuffer), "⏰ Next wakeup in: %d seconds\n", wake_time);
    Serial.print(timeBuffer);
    appendFile(fich_log.c_str(), timeBuffer);

    return wake_time;
}

String get_formatted_time() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("⚠️ Failed to get time");
        return "0000-00-00 00:00:00";
    }
    char timeStringBuff[30];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}


void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();  

    switch (wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("⏰ Wakeup caused by timer (Scheduled wake-up for data logging)");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("🔔 Wakeup caused by external GPIO (e.g., button press)");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.println("🔔 Wakeup caused by external signal (Multiple GPIOs)");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            Serial.println("❓ Wakeup reason is undefined (First power-on or reset)");
            break;
        default:
            Serial.print("🤷 Unexpected wakeup reason. Code: ");
            Serial.println(wakeup_reason);  // 打印未定义的唤醒原因
            break;
    }
}


bool setup_sd_card() {
    if (!SD_MMC.begin()) {
        Serial.println("❌ SD card initialization failed!");
        delay(2000);
    }
    Serial.println("✅ SD card initialized successfully!");
    sd_initialized = true;
    return true;
}

bool appendFile(const char *path, const char *message)
{
    // **尝试打开 SD 卡**
    if (!sd_initialized) {  //  **确保 SD 已初始化**
        if (!setup_sd_card()) return false;
    }

    // **打开文件，追加写入**
    File file = SD_MMC.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("❌ Failed to open file for appending!");
        return false;
    }

    // **写入数据**
    if (!file.println(message)) {
        Serial.println("⚠️ Append failed!");
        file.close();
        return false;
    }

    // **关闭文件**
    file.flush();
    file.close();

    Serial.println("✅ Log saved to SD!");
    return true;
}

