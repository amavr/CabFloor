// #include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "DHT.h"
#include <GyverOLED.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "am.names.h"

#define ALARM_PIN GPIO_NUM_35
#define SWITCH_PIN GPIO_NUM_37
#define SENSORS_PIN GPIO_NUM_33

// DECLARE DHT11 --------------------
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
#define DHTPIN GPIO_NUM_3 // Digital pin connected to the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// DECLARE DS18B20 --------------------
#define ONE_WIRE_BUS GPIO_NUM_5  // вывод, к которому подключён DS18B20
#define TEMPERATURE_PRECISION 12 // точность измерений (9 ... 12)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature flo_sensors(&oneWire);
DeviceAddress ThermometerAddr;

// DECLARE OLED SSD1306_128x32 --------------------
GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;

#define CHANNEL 4

#define PRODUCT true

SemaphoreHandle_t sem;
static portMUX_TYPE spin_lock = portMUX_INITIALIZER_UNLOCKED;

// мастер живой и отдает команды
volatile bool master_alive = false;

typedef struct
{
    int8_t min;
    int8_t max;

} interval_t;

interval_t air_t = {20, 25};
interval_t flo_t = {20, 25};
uint16_t thres_air_h;

typedef struct
{
    uint8_t cmd_code;
    uint8_t prm_code;
    int16_t prm1;
    int16_t prm2;
} espnow_message_t;

typedef struct
{
    espnow_message_t data;
    uint8_t mac[6]; // 6 bytes
} queue_item_t;

xQueueHandle recv_queue;

void read_sensors_values(void *parameter);
void showStates(const char *info);
void showStates(float TA, float HA, float TF, bool heaterOn);
void OnDataRecv(uint8_t *mac, uint8_t *msg_data, uint8_t len);

void setup()
{
    pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SENSORS_PIN, OUTPUT);
    pinMode(SWITCH_PIN, OUTPUT);
    pinMode(ALARM_PIN, OUTPUT);

    Serial.begin(9600);

    oled.init(GPIO_NUM_12, GPIO_NUM_11); // инициализация
    oled.clear();                        // очистка
    oled.home();                         // курсор в 0,0

    oled.setScale(1); // масштаб текста (1..4)

    oled.print("INIT.");

    dht.begin();

    flo_sensors.begin(); // инициализация DS18B20
    // bool has_address = flo_sensors.getAddress(ThermometerAddr, 0);     // адрес DS18B20 (поиск по индексу)
    // flo_sensors.setResolution(ThermometerAddr, TEMPERATURE_PRECISION); // установка точности измерения 9...12 разрядов
    // digitalWrite(GPIO_NUM_37, !has_address);

    flo_sensors.setResolution(TEMPERATURE_PRECISION);

    for (uint8_t i = 0; i < 5; i++)
    {
        if (dht.readTemperature() > 127)
        {
            oled.print('.');
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else
        {
            break;
        }
    }

    recv_queue = xQueueCreate(5, sizeof(queue_item_t *));

    xTaskCreatePinnedToCore(
        read_sensors_values,
        "read_sensors_values", // Task name
        5000,                  // Stack size (bytes)
        NULL,                  // Parameter
        1,                     // Task priority
        NULL,                  // Task handle
        CONFIG_ARDUINO_RUNNING_CORE);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        showStates("ESP-NOW: init error");
        digitalWrite(ALARM_PIN, true);
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    // wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    // esp_wifi_get_channel(&primaryChan, &secondChan);
}

void loop()
{
    // sensors_event_t event;

    // if (false)
    // {
    //     dht.temperature().getEvent(&event);
    //     if (!isnan(event.temperature))
    //     {
    //         tin = event.temperature;
    //     }

    //     dht.humidity().getEvent(&event);
    //     if (!isnan(event.relative_humidity))
    //     {
    //         hin = event.relative_humidity;
    //     }

    //     flo_sensors.requestTemperatures(); // считывание значение температуры
    //     tfl = flo_sensors.getTempCByIndex(0);
    //     // tfl = flo_sensors.getTempC(ThermometerAddr); // температура в градусах Цельсия

    //     showStates(tin, hin, tfl);
    // }

    vTaskDelay(1000);
}

void sendSensorValue(uint8_t sensor_id, int16_t sensor_value)
{
}

// ----------------------- TASKS
void read_sensors_values(void *parameter)
{
    float cur_air_t = 0;
    float cur_air_h = 0;
    float cur_flo_t = 0;

    float pre_air_t = 0;
    float pre_air_h = 0;
    float pre_flo_t = 0;

    bool heater_on = false;
    while (true)
    {
        digitalWrite(SENSORS_PIN, true);
        flo_sensors.requestTemperatures(); // считывание значение температуры

        cur_air_t = trunc(dht.readTemperature() * 10);
        cur_air_h = trunc(dht.readHumidity() * 10);
        cur_flo_t = trunc(flo_sensors.getTempCByIndex(0) * 10);
        digitalWrite(SENSORS_PIN, false);

        if (pre_air_t != cur_air_t)
        {
            sendSensorValue(BYT_SNS_TMP_TAB, (int16_t)cur_air_t);
            pre_air_t = cur_air_t;
        }
        if (pre_air_h != cur_air_h)
        {
            sendSensorValue(BYT_SNS_HUM_TAB, (int16_t)cur_air_h);
            pre_air_h = cur_air_h;
        }
        if (pre_flo_t != cur_flo_t)
        {
            sendSensorValue(BYT_SNS_HUM_TAB, (int16_t)cur_flo_t);
            pre_flo_t = cur_flo_t;
        }
        cur_air_t /= 10.0;
        cur_air_h /= 10.0;
        cur_flo_t /= 10.0;

        if(cur_flo_t < flo_t.min || cur_air_h < air_t.min){
            heater_on = true;
        }
        if(cur_flo_t >= flo_t.max || cur_air_h >= air_t.max){
            heater_on = false;
        }
        digitalWrite(SWITCH_PIN, heater_on);

        showStates(cur_air_t, cur_air_h, cur_flo_t, heater_on);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// ----------------------- UTILS
void OnDataRecv(uint8_t *mac, uint8_t *msg_data, uint8_t len)
{
    if (!esp_now_is_peer_exist(mac))
    {
        esp_now_peer_info_t peerInfo;
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = CHANNEL;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    espnow_message_t pack;
    memcpy(&pack, msg_data, sizeof(pack));

    queue_item_t *msg;
    msg->data = pack;
    memcpy(msg->mac, mac, 6);

    portBASE_TYPE xStatus = xQueueSend(recv_queue, (void *)&pack, 0);

    // записать в очередь для неспешного разбора и выйти

    // digitalWrite(ALARM_PIN, true);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    // digitalWrite(ALARM_PIN, false);
}

void showStates(const char *info)
{
    oled.clear();
    oled.home(); // курсор в 0,0
    oled.print(info);
}

void showStates(float TA, float HA, float TF, bool heaterOn)
{
    const uint8_t line_h = 12;
    const char *dir = heaterOn ? " > " : " < ";
    oled.home(); // курсор в 0,0
    oled.print("^ ");
    oled.print(air_t.min);
    oled.print(dir);
    oled.print(TA, 1);
    oled.print(dir);
    oled.print(air_t.max);

    // oled.println();
    oled.setCursorXY(0, line_h * 1);
    oled.print("v ");
    oled.print(flo_t.min);
    oled.print(dir);
    oled.print(TF, 1);
    oled.print(dir);
    oled.print(flo_t.max);

    oled.setCursorXY(0, line_h * 2);
    oled.print("% ");
    oled.print(HA, 1);

    oled.printf(" -=(%d)=-", CHANNEL);
    // oled.print(" -=(");
    // oled.print(CHANNEL);
    // oled.print(")=-");

    // oled.println();

    // oled.print("ADDR: ");
    // for(int i = 0; i < 8; i++){
    //     oled.print(ThermometerAddr[i], HEX);
    // }
}
