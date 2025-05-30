#include <Arduino.h>

//**!!! Debe definir XPOWERS_CHIP_AXP2102 para que XPowersLib.h funcione correctamente!!!**
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"    // Gestión de energía PMU

#include <HTTPClient.h>
#include "utilities.h"     // Definición de PIN del tablero de desarrollo
#include <Wire.h>          // Comunicación I2C (PMU)
#include <FS.h>
#include <SD_MMC.h>        // biblioteca de tarjetas SD
#include <esp_bt.h>        // Apague Bluetooth para guardar la alimentación
#include <esp_wifi.h>      // gestión de ahorro de energía wifi
#include "esp_log.h"
#include <WiFi.h>          // biblioteca de conexión wifi
#include "time.h"          // Sincronización de tiempo NTP
#include <PubSubClient.h>  // biblioteca mqtt

//**Información de conexión wifi **
WiFiClient espClient;//Cliente WiFi
#define WIFI_SSID "Livebox6-8B92"
#define WIFI_PASSWORD "xhbFN5FDZ7J7"

// #define WIFI_SSID "MIWIFI_3PhE"
// #define WIFI_PASSWORD "ZFYFDRuG"

#define TIMEOUT_WIFI 20000       //Tiempo de espera de conexión WiFi (20 segundos)

//**Información del servidor MQTT **
PubSubClient client(espClient);
String dataString;               //cadena de datos del sensor
#define MQTT_SERVER "192.168.1.100"
#define MQTT_PORT 1883
#define MQTT_USER "user1"
#define MQTT_PASSWORD "user1"
#define MQTT_TOPIC "sensor/data"  //Tema mqtt
#define TIMEOUT_MQTT 10000        //Tiempo de espera de conexión MQTT (10 segundos)
bool boolcallback;                //Bandera de devolución de llamada MQTT

//**Información del servidor NTP **
struct tm timeinfo;                //Estructura de información de tiempo
#define NTP_SERVER "pool.ntp.org"   
#define GMT_OFFSET_SEC (2 * 3600)   //GMT+1 (la zona horaria debe ajustarse，Verano horario GMT+2)
#define DAYLIGHT_OFFSET_SEC 0       //Ajuste de tiempo de ahorro de luz del día (0 si no hay tiempo de ahorro de verano)
bool timeSynced = false;            //Bandera de sincronización de tiempo

//**Relacionado con el sueño profundo **
#define SLEEP_TIME_SECONDS 900              //15 minutos (900 segundos)

#define uS_TO_S_FACTOR 1000000ULL      //convertir microsegundos en segundos
esp_sleep_wakeup_cause_t wakeup_reason;     //**ESP32 Temporizador de atención de sueño

//**Adquisición de datos del sensor **
#define TIME_OUT_SENSOR 2000    //Tiempo de espera de lectura del sensor (2 segundos)
#define RS485_RXD_PIN 11       //Pin de recepción del sensor RS485
#define RS485_TXD_PIN 14        //sensor rs485 envía pin
#define RS485_DIR1 13            //RS485 Pin de control de dirección 1
#define RS485_DIR2 13            //RS485 Pin de control de dirección 2
float sensores[7];

//**Almacenamiento de tarjeta SD **
#define LOG_FILE "/IoT_log.txt"      //almacenar archivos de registro
String fich_log = LOG_FILE;
String file_name;

//**Objeto de administración de energía PMU **
XPowersPMU PMU;
String getBattVoltage;          //Información de voltaje de la batería

bool setup_wifi();
bool setup_ntp();
bool logData(bool connection);
bool wifi_mqtt_connect();
void print_wakeup_reason();
bool setup_sd_card();
bool appendFile(const char *path, const char *message);
uint32_t calcula_offset();
String get_formatted_time();
String read_last_time();
bool save_last_time(const String &datetime);
bool wifi_thingspeak_upload();

void setup()
{   
    //**1️. Inicializar el puerto serie **
    Serial.begin(115200);

    delay(5000);    //espera 5 segundos para observar la salida del puerto serie
    Serial.println("\n🚀 Booting...");

    //**2 Se apaga el Bluetooth para ahorrar energía **
    esp_bt_controller_disable();
    
    //**3 Imprimir la razón de despertar de ESP32 **
    print_wakeup_reason();

    //**4 Inicializar la gestión de energía PMU **
    Serial.println("🔋 Initializing PMU...");
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("❌ Failed to initialize PMU!");
        while (1) delay(5000);
    }
    Serial.printf("✅ PMU ID: 0x%x\n", PMU.getChipID());

    //**5 Configurar el PMU **
    PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_3V88);//Establecer VBUS Voltaje de funcionamiento mínimo
    PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);//establecer la corriente máxima VBUS
    PMU.setSysPowerDownVoltage(2600);//Voltaje de apagado del sistema (por debajo de 2.6V)

    //**6️. Apagar el apagado automático por bajo voltaje **
    PMU.disableDC5LowVoltageTurnOff();
    PMU.disableDC4LowVoltageTurnOff();

    //**7️. Apagar la protección contra sobrecalentamiento **
    PMU.disableOverTemperaturePowerOff();

    //**8️. Configuración de la botón de encendido **
    PMU.enableFastPowerOn();//enciende rápidamente
    PMU.disableLongPressShutdown();//Deshabilitar prensa larga y apagar la máquina
    PMU.setLongPressRestart();//Presione y reinicie

        // Set the button power-on press time
    PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    Serial.println("PowerKeyPressOnTime:128 Ms");

    //** !!! 9. Apagar TSPinMeasure ,de lo contrario, no se cargará la batería **
    PMU.disableTSPinMeasure();

    //**10. Configurar la detección de la batería **
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();

    //**11. Configurar parámetros de carga **
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);//Corriente de precarge 50 mA
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);//cargar 200MA actual
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);//Corriente de terminación 25MA
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);//Voltaje objetivo de carga 4.2V

    //**1️2. Estado de información **
    Serial.println();
    Serial.print("🔋 Is Charging: "); Serial.println(PMU.isCharging() ? "Yes" : "No");
    Serial.print("🔌 VBUS Connected: "); Serial.println(PMU.isVbusIn() ? "Yes" : "No");
    getBattVoltage = String(PMU.getBattVoltage());
    Serial.print("⚡ Battery Voltage: "); Serial.print(getBattVoltage); Serial.println("mV");

    //**13. Apagar canales de alimentación innecesarios **
    PMU.disableGauge();
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU.disableWatchdog();
    PMU.disableTemperatureMeasure();

    PMU.disableDC2();
    PMU.disableDC3();
    PMU.disableDC4();
    PMU.disableDC5();       //La alimentación del sensor

    PMU.disableALDO1();
    PMU.disableALDO2();
    PMU.disableALDO3();     //La alimentación de la tarjeta SD
    PMU.disableALDO4();
    PMU.disableBLDO1();
    PMU.disableBLDO2();

    PMU.disableDLDO1();
    PMU.disableDLDO2();
    PMU.disableCPUSLDO();

    while(PMU.isCharging()){
        Serial.println("Charging...");
        getBattVoltage = (String)PMU.getBattVoltage();
        Serial.print("GetBattVoltage:"); Serial.print(getBattVoltage); Serial.println("mV");
        PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
        delay(1000);
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        delay(1000);
    }
    Serial.println("⚡ PMU Setup Complete!");   
}

void loop()
{
    uint32_t offset;
    bool connection = false;
    String error;

    if((PMU.getBattVoltage()>3700) || PMU.isVbusIn()){
      
        //Encienda la alimentación del sensor, no modifique los parámetros!!
        PMU.setDC5Voltage(3300);  
        PMU.enableDC5();
        delay(300);

        //** Encienda la fuente de alimentación de la tarjeta SD, ¡no modifique los parámetros! ! **
        PMU.setALDO3Voltage(3300); 
        PMU.enableALDO3();
        delay(300);

        //** Inicializa la tarjeta SD **
        SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA); //establecer pines de tarjeta SD
        
        
        Serial.println("🔌 Initializing SD Card...");
               
        while (!SD_MMC.begin("/sdcard", true)) {
            Serial.println("⚠️ SD Card init failed");
            delay(1000);
        }
        
        uint8_t cardType = SD_MMC.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("No SD_MMC card attached");
            while (1) {
                delay(1000);
            }
        }

        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

        //** Escriba el encabezado del archivo de registro **
        String lastTime = read_last_time();
        String startupLog = "\n ****** Última vez de colección： " + lastTime + " ******\n";
        appendFile(fich_log.c_str(), startupLog.c_str());


        //** Conecte wifi para sincronizar el tiempo **
        if (WiFi.status() != WL_CONNECTED) {
            setup_wifi();
        }

        //**4 recopilar datos del sensor **
        if (!logData(connection)) {
            error = "LogData error!!!\n";
            Serial.print(error);
            appendFile(fich_log.c_str(), error.c_str());
        } else {
            //**5 Conéctese a wifi y cargue datos **
            //connection = wifi_thingspeak_upload();//wifi_mqtt_connect(); 
            connection = wifi_mqtt_connect();

            unsigned long previous = millis();
            boolcallback = true;
            while (((millis() - previous) < TIMEOUT_MQTT) && boolcallback) {
                client.loop();
            }
            Serial.println("⏳ Timeout Client.");

            //** Guarde la hora actual en la tarjeta SD **
            String currentTime = get_formatted_time();  
            save_last_time(currentTime);  

            //WiFi.disconnect();
            WiFi.disconnect(true); // Desconectar WiFi y liberar recursos
            esp_wifi_stop();
        }

        //**6 Calcule el tiempo de despertar **
        offset = calcula_offset() + 5;


        //**7 Apague la alimentación de la tarjeta SD y sensor **
        SD_MMC.end();
        PMU.disableALDO3();
        PMU.disableDC5();
    }
    else {
        Serial.println("Battery low, minimum voltage 3700mV, offset 300.");
        offset = 3600;  // 1 hora de sueño profundo
    }
    
    //**8 Entra el sueño profundo **
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    esp_sleep_enable_timer_wakeup(offset * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    //delay(30000); // simular el sueño profundo
}

bool setup_wifi() {
    unsigned long previous;
    String cadena;
    bool a;


    WiFi.mode(WIFI_STA);

    // ** Muestra la dirección MAC de la placa ESP32 **
    Serial.print("ESP32 Board MAC Address: ");
    Serial.println(WiFi.macAddress());

    // ** Si desea cambiar la dirección MAC, descomente las siguientes líneas y ajuste la dirección MAC**
    // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
    // Serial.print("[NEW] ESP32 Board MAC Address: ");
    // Serial.println(WiFi.macAddress());

    // ** Conectarse a la red WiFi **
    Serial.print("🔌 Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    previous = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - previous > TIMEOUT_WIFI) {
            cadena = "\n❌ WiFi connection failed! Timeout.\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
            return false;
        }
    }

    cadena = "✅ Connected to WiFi.\n";
    Serial.print(cadena);
    appendFile(fich_log.c_str(), cadena.c_str());

    cadena = "Gateway IP address: " + WiFi.gatewayIP().toString() + "\n";
    Serial.print(cadena);
    appendFile(fich_log.c_str(), cadena.c_str());

    if(!timeSynced) { //Si el tiempo no se ha sincronizado, sincronizarlo
        setup_ntp();
    }

    return true;
}



bool setup_ntp() {

    String cadena;
    bool a;

    cadena = "⏳ Synchronizing time with NTP...\n";
    Serial.print(cadena);
    appendFile(fich_log.c_str(), cadena.c_str());

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER); // Configurar el servidor NTP y la zona horaria
    
    if (!getLocalTime(&timeinfo)) {
        cadena = "⚠️ Failed to obtain time!\n";
        Serial.print(cadena);
        appendFile(fich_log.c_str(), cadena.c_str());
        timeSynced = false;
        return timeSynced;
    }
    
    cadena = "✅ Time synchronized successfully!\n";
    Serial.print(cadena);
    appendFile(fich_log.c_str(), cadena.c_str());    

    // Serial.printf("📆 Current Time: %04d-%02d-%02d %02d:%02d:%02d\n",
    //             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
    //             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    String now_time = get_formatted_time();
    Serial.print("📆 Current Time: ");
    Serial.println(now_time);

    // ** Escribir la hora actual en el archivo de registro **
    String logTime = "\n****** " + now_time + " ******\n";
    appendFile(fich_log.c_str(), logTime.c_str());

    timeSynced = true;
    return timeSynced;
}


bool logData(bool connection) 
{
    uint8_t k, l, indice, car[19];
    uint8_t index = 0;
    unsigned long previous;
    unsigned long timeout = TIME_OUT_SENSOR;
    String minutos;
    String cadena;

    //** recopilar datos del sensor **
    Serial2.begin(4800, SERIAL_8N1, RS485_RXD_PIN, RS485_TXD_PIN);
    delay(5000); //espera 5 segundos para estabilizar la comunicación RS485

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
    delay(2);
    digitalWrite(RS485_DIR2, LOW);
    digitalWrite(RS485_DIR1, LOW);

    //**Leer datos **
    k = 0;
    indice = 0;
    previous = millis();

    do {
        if (Serial2.available() != 0) {
            car[k] = Serial2.read();
            //Serial.printf("0x%02X ", car[k]);   //imprimir datos hexadecimales
            if (car[k] == 1) indice = 1;
            if (indice) k++;
        }
        if (millis() < previous) previous = millis();
    } while ((k < 19) && ((millis() - previous) < timeout));
    

    Serial.print("Trama: ");
    for(l=0; l<k; l++){
        Serial.printf("0x%02X ", car[l]);
    }

    Serial.println();
    digitalWrite(RS485_DIR2, HIGH);
    digitalWrite(RS485_DIR1, HIGH);

    //**Analizar datos **
    if (k != 19) {
        cadena = "❌Timeout data error!!!\nNumero de elementos: " + String(k) + "\n";
        Serial.print(cadena);
        appendFile(fich_log.c_str(), cadena.c_str());
        return false;
    }

    sensores[1] = ((car[3] << 8) | car[4]) * 0.1;       //Humedad 
    sensores[0] = ((car[5] << 8) | car[6]) * 0.1;       //Temperatura 
    sensores[2] = (car[7] << 8) | car[8];         //Conductividad 
    sensores[3] = ((car[9] << 8) | car[10]) * 0.1;     //pH 
    sensores[4] = (car[11] << 8) | car[12];     //Nitrógeno 
    sensores[5] = (car[13] << 8) | car[14];    //Fósforo 
    sensores[6] = (car[15] << 8) | car[16];     //Potasio 

    Serial.println("✅ Sensor Data Collected!");

    //**Obtenga la hora actual (usando NTP) **
    String current_time = get_formatted_time();

    //**Almacenamiento a la tarjeta SD **
    dataString = current_time + "," + 
                    String(sensores[0]) + "," + 
                    String(sensores[1]) + "," + 
                    String(sensores[2]) + "," + 
                    String(sensores[3]) + "," + 
                    String(sensores[4]) + "," + 
                    String(sensores[5]) + "," + 
                    String(sensores[6])+ "," +
                    getBattVoltage;

    char dateString[20];
    strftime(dateString, sizeof(dateString), "%Y-%m-%d", &timeinfo);
    file_name = "/" + String(dateString) + ".csv";

    appendFile(fich_log.c_str(), ("New data written to: " + file_name).c_str());
    appendFile(fich_log.c_str(), dataString.c_str());

    bool success = appendFile(file_name.c_str(), dataString.c_str());
    if (!success) {
         appendFile(fich_log.c_str(), "Failed to write data to SD card!\n");
    }
    return success;
}


void callback(char *topic, byte *payload, unsigned int length)
{
        String cadena;
        bool a;

        cadena = "Message arrived in topic: \n";
        Serial.print(cadena);
        a = appendFile(fich_log.c_str(),cadena.c_str());

        cadena = (String)(topic)+"\n";
        Serial.print(cadena);
        a = appendFile(fich_log.c_str(),cadena.c_str());

        cadena = "Message:\n";
        Serial.print(cadena);
        a = appendFile(fich_log.c_str(),cadena.c_str());

        for (int i = 0; i < length; i++) {
            Serial.print((char) payload[i]);
            a = appendFile(fich_log.c_str(),cadena.c_str());
        }
        cadena = "\n-----------------------\n";
        Serial.print(cadena);
        a = appendFile(fich_log.c_str(),cadena.c_str());
        boolcallback = false;
}


bool wifi_mqtt_connect()
{
    unsigned long previous;
    String cadena;
    bool a;

    if (WiFi.status() != WL_CONNECTED) {
        cadena= "❌ WiFi disconnected. Trying to reconnect...\n";
        Serial.print(cadena);
        setup_wifi();
    }

    // **Establecer el servidor MQTT y la función de devolución de llamada
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(callback);

    // **Conectarse al servidor MQTT
    previous = millis();
    while (!client.connected()) {
        cadena = "🔄 Attempting MQTT connection...\n";
        Serial.print(cadena);
        appendFile(fich_log.c_str(), cadena.c_str());

        String client_id = "ESP32Client-" + String(WiFi.macAddress());
        if (client.connect(client_id.c_str(), MQTT_USER, MQTT_PASSWORD)) {
            cadena = "✅ Connected to MQTT broker!\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
            client.subscribe("sensor/control");
            break;  
        } else {
            cadena = "❌ Failed, rc=" + String(client.state()) + ". Retrying in 2 seconds.\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
            delay(2000);
        }

        if (millis() - previous > TIMEOUT_MQTT) {
            cadena = "⏰ Timeout MQTT connection.\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
            return false;
        }
    }

    // **Muestra los datos del sensor en la consola **
    String print_data;
    print_data += "\"temperature(t)\": " + String(sensores[0]) + " °C"+ ",";
    print_data += "\"humidity(h)\": " + String(sensores[1]) + "%" + ",";
    print_data += "\"conductivity(c)\": " + String(sensores[2]) + " uS/cm"+ ",";
    print_data += "\"ph(ph)\": " + String(sensores[3]) + ",";
    print_data += "\"nitrogen(n)\": " + String(sensores[4]) +  " mg/kg"+ ",";
    print_data += "\"phosphorus(p)\": " + String(sensores[5]) + " mg/kg"+ ",";
    print_data += "\"potassium(k)\": " + String(sensores[6]) + " mg/kg"+ ",";
    print_data += "\"battery(b)\": " + getBattVoltage+ " mV";

    cadena= "📤 Publishing data to MQTT:\n" + print_data + "\n";
    Serial.print(cadena);
    appendFile(fich_log.c_str(), cadena.c_str());
    
    // **Publique los datos del sensor a MQTT

    String mqtt_data = "{";
    mqtt_data += "\"t\":" + String(sensores[0]) + ",";
    mqtt_data += "\"h\":" + String(sensores[1]) + ",";
    mqtt_data += "\"c\":" + String(sensores[2]) + ",";
    mqtt_data += "\"ph\":" + String(sensores[3]) + ",";
    mqtt_data += "\"n\":" + String(sensores[4]) + ",";
    mqtt_data += "\"p\":" + String(sensores[5]) + ",";
    mqtt_data += "\"k\":" + String(sensores[6]) + ",";
    mqtt_data += "\"b\":" + getBattVoltage; 
    mqtt_data += "}";
    
    if (client.connected()) {
        if (client.publish(MQTT_TOPIC, mqtt_data.c_str())) {
            cadena = "✅ MQTT Publish success\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
        } else {
            cadena = "❌ MQTT Publish failed\n";
            Serial.print(cadena);
            appendFile(fich_log.c_str(), cadena.c_str());
        }
    }
    return true;
}

// bool wifi_thingspeak_upload() {
//     if (WiFi.status() != WL_CONNECTED) {
//         Serial.println("❌ WiFi not connected, trying to reconnect...");
//         if (!setup_wifi()) return false;
//     }
    
//     String server = "http://api.thingspeak.com/update";
//     String apiKey = "35XFHBIOM1QA8XNL";    // API key de ThingSpeak
//     String url = server + "?api_key=" + apiKey +
//                  "&field1=" + String(sensores[0]) +  // 温度
//                  "&field2=" + String(sensores[1]) +  // 湿度
//                  "&field3=" + String(sensores[2]) +  // 电导率
//                  "&field4=" + String(sensores[3]) +  // pH
//                  "&field5=" + String(sensores[4]) +  // 氮
//                  "&field6=" + String(sensores[5]) +  // 磷
//                  "&field7=" + String(sensores[6]) +  // 钾
//                  "&field8=" + getBattVoltage; // 电池电压

//     Serial.println("📤 Sending data to ThingSpeak...");
//     Serial.println(url);

//     HTTPClient http;
//     http.begin(url);
//     int httpResponseCode = http.GET();

//     if (httpResponseCode > 0) {
//         Serial.printf("✅ ThingSpeak response: %d\n", httpResponseCode);
//         http.end();
//         return true;
//     } else {
//         Serial.printf("❌ Failed to send data. Code: %d\n", httpResponseCode);
//         http.end();
//         return false;
//     }
// }


uint32_t calcula_offset()
{
    uint32_t wake_time;
    uint32_t Periodo_muestreo = SLEEP_TIME_SECONDS; //Período de muestreo 15 minutos (900 segundos)
    char timeBuffer[40];

    //**Obtenga la hora actual **
    if (!getLocalTime(&timeinfo)) {

        Serial.println("⚠️ Failed to obtain time from NTP!");
        return Periodo_muestreo;  //Si la adquisición de tiempo falla, el intervalo predeterminado
    }

    //**Calcule el siguiente tiempo de muestreo **
    uint32_t current_seconds = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    wake_time = Periodo_muestreo - (current_seconds % Periodo_muestreo);

    //**Tiempo de atención de impresión **
    snprintf(timeBuffer, sizeof(timeBuffer), "⏰ Next wakeup in: %d seconds\n", wake_time);
    String cadena= "Next wakeup in: " + String(wake_time) + " seconds\n";
    appendFile(fich_log.c_str(), cadena.c_str());

    return wake_time;
}

String get_formatted_time() {
    struct tm timeinfo;
    String cadena;

    if (!getLocalTime(&timeinfo)) {
        cadena = "⚠️ Failed to get time\n";
        Serial.print(cadena);
        appendFile(fich_log.c_str(), cadena.c_str());
        return "0000-00-00 00:00:00";
    }
    char timeStringBuff[30];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}


void print_wakeup_reason()
{
    wakeup_reason = esp_sleep_get_wakeup_cause();  

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
            Serial.println(wakeup_reason);  //Imprima la razón de despertar indefinida
            break;
    }
}

bool setup_sd_card() {
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("❌ SD card initialization failed!");
        return false;
    }
    Serial.println("✅ SD card initialized successfully!");
    return true;
}

bool appendFile(const char *path, const char *message)
{
    //**Abra el archivo y agregue la escritura **
    File file = SD_MMC.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("❌ Failed to open file for appending!");
        return false;
    }
    //**Escribir datos **
    if (!file.println(message)) {
        Serial.println("⚠️ Append failed!");
        file.close();
        return false;
    }
    //**Cierre el archivo **
    file.flush();
    file.close();

    //Serial.println("✅ Log saved to SD!");
    return true;
}

bool save_last_time(const String &datetime) {
    File file = SD_MMC.open("/last_time.txt", FILE_WRITE);
    if (!file) {
        Serial.println("❌ Failed to open last_time.txt for writing");
        return false;
    }
    file.println(datetime);
    file.close();
    Serial.println("✅ Last time saved to SD.");
    return true;
}

String read_last_time() {
    File file = SD_MMC.open("/last_time.txt");
    if (!file || file.size() == 0) {
        Serial.println("❌ No valid last_time.txt found");
        return "Unknown";
    }
    String line = file.readStringUntil('\n');
    file.close();
    Serial.println("📂 Last recorded time: " + line);
    return line;
}

