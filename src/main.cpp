// #include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "DHT.h"
#include <GyverOLED.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "am.names.h"

#define TURN_PIN GPIO_NUM_33
#define ENC_BTN_PIN GPIO_NUM_35
#define ENC_DAT_PIN GPIO_NUM_37
#define ENC_CLK_PIN GPIO_NUM_39

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
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;

#define CHANNEL 4

#define PRODUCT true

SemaphoreHandle_t sem;
static portMUX_TYPE spin_lock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t clickHandlerTask;
TaskHandle_t rotateHandlerTask;

// мастер живой и отдает команды
volatile bool master_alive = false;

typedef struct
{
    uint16_t val; // текущее значение настройки
    uint16_t min; // нижняя граница настроек
    uint16_t max; // верхняя граница настроек
    uint8_t x;    // позиция на экране по Х (pixels)
    uint8_t y;    // позиция на экране по Y (pixels)
} param_t;

typedef struct
{
    param_t min; // нижний порог срабатывания
    param_t max; // верхний порог срабатывания
} sensor_params_t;

param_t params[] = {
    {19, 0, 40, 30, 0}, {25, 0, 45, 100, 0}, {20, 0, 40, 30, 12}, {24, 0, 40, 100, 12}, {4, 0, 16, 30, 24}};

const uint8_t AIR_INDEX_MIN = 0;
const uint8_t AIR_INDEX_MAX = 1;
const uint8_t FLO_INDEX_MIN = 2;
const uint8_t FLO_INDEX_MAX = 3;
const uint8_t CHANNEL_INDEX = 4;

typedef struct
{
    float cur_val;
    float pre_val;
} sensor_data_t;

sensor_data_t datas[] = {{0, 0}, {0, 0}};

const uint8_t AIR_INDEX = 0;
const uint8_t FLO_INDEX = 1;

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
void showStatic();
void showInfo(const char *info);
void showState();
void OnDataRecv(uint8_t *mac, uint8_t *msg_data, uint8_t len);

void IRAM_ATTR onEncoderClick()
{
    BaseType_t woken = pdFALSE;
    portENTER_CRITICAL_ISR(&spin_lock);

    vTaskNotifyGiveFromISR(clickHandlerTask, &woken);
    if (woken)
    {
        portYIELD_FROM_ISR(woken);
    }

    portEXIT_CRITICAL_ISR(&spin_lock);
}

void IRAM_ATTR onEncoderRotate()
{
    BaseType_t woken = pdFALSE;
    portENTER_CRITICAL_ISR(&spin_lock);

    vTaskNotifyGiveFromISR(rotateHandlerTask, &woken);
    if (woken)
    {
        portYIELD_FROM_ISR(woken);
    }

    portEXIT_CRITICAL_ISR(&spin_lock);
}

void handleEncoderClick(void *param)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        if (digitalRead(ENC_BTN_PIN) == LOW)
        {
            Serial.println("clicked!");
        }
    }
}

volatile int encCounter;
volatile boolean flag, resetFlag;
volatile byte curState, prevState;

void handleEncoderRotate(void *param)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // digitalRead хорошо бы заменить чем-нибудь более быстрым
        curState = digitalRead(ENC_CLK_PIN) | digitalRead(ENC_DAT_PIN) << 1;
        if (resetFlag && curState == 0b11)
        {
            if (prevState == 0b10)
                encCounter--;
            if (prevState == 0b01)
                encCounter++;
            resetFlag = 0;
            flag = true;
        }
        if (curState == 0b00)
        {
            resetFlag = 1;
        }
        prevState = curState;

        if (flag)
        {
            Serial.println(encCounter);
            flag = false;
        }

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        // if (digitalRead(ENC_DAT_PIN) == LOW)
        // {
        //     Serial.print("rotate");
        //     if (digitalRead(ENC_CLK_PIN) == LOW)
        //     {
        //         Serial.println(" as clock");
        //     }
        //     else
        //     {
        //         Serial.println(" anti clock");
        //     }
        // }
    }
}

void setup()
{
    pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
    pinMode(ENC_BTN_PIN, INPUT_PULLUP);
    pinMode(ENC_DAT_PIN, INPUT_PULLUP);
    pinMode(ENC_CLK_PIN, INPUT_PULLUP);

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(9600);

    oled.init(GPIO_NUM_12, GPIO_NUM_11); // инициализация
    oled.clear();                        // очистка
    oled.home();                         // курсор в 0,0

    oled.setScale(1); // масштаб текста (1..4)

    oled.print("INIT.");
    oled.update();

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

    xTaskCreate(handleEncoderClick,
                "ClickHandler",
                8192,
                NULL,
                1,
                &clickHandlerTask);

    xTaskCreate(handleEncoderRotate,
                "RotateHandler",
                8192,
                NULL,
                1,
                &rotateHandlerTask);

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
        showInfo("ESP-NOW: init error");
        // digitalWrite(ALARM_PIN, true);
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    // wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    // esp_wifi_get_channel(&primaryChan, &secondChan);

    showStatic();

    attachInterrupt(digitalPinToInterrupt(ENC_BTN_PIN), onEncoderClick, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_DAT_PIN), onEncoderRotate, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), onEncoderRotate, CHANGE);
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

    int pre_air_t = 0;
    int pre_air_h = 0;
    int pre_flo_t = 0;

    bool heater_on = false;
    while (true)
    {
        digitalWrite(LED_BUILTIN, true);
        flo_sensors.requestTemperatures(); // считывание значение температуры

        cur_air_t = trunc(dht.readTemperature() * 10);
        cur_air_h = trunc(dht.readHumidity() * 10);
        cur_flo_t = trunc(flo_sensors.getTempCByIndex(0) * 10);
        digitalWrite(LED_BUILTIN, false);

        cur_air_t /= 10.0;
        cur_air_h /= 10.0;
        cur_flo_t /= 10.0;

        datas[AIR_INDEX].pre_val = datas[AIR_INDEX].cur_val;
        datas[AIR_INDEX].cur_val = cur_air_t;
        datas[FLO_INDEX].pre_val = datas[FLO_INDEX].cur_val;
        datas[FLO_INDEX].cur_val = cur_flo_t;

        if ((cur_flo_t < params[FLO_INDEX_MIN].val) || (cur_air_t < params[AIR_INDEX_MIN].val))
        {
            heater_on = true;
        }
        if ((cur_flo_t > params[FLO_INDEX_MAX].val) || (cur_air_t > params[AIR_INDEX_MAX].val))
        {
            heater_on = false;
        }
        digitalWrite(TURN_PIN, heater_on);

        showState();
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

#pragma region Вывод на информации OLED
void showInfo(const char *info)
{
    oled.clear();
    oled.home(); // курсор в 0,0
    oled.print(info);
    oled.update();
}

const uint8_t LINE_HEIGHT = 12;

void showLabels()
{
    oled.clear();
    oled.setCursorXY(0, LINE_HEIGHT * 0);
    oled.print("ВОЗ ");
    oled.setCursorXY(0, LINE_HEIGHT * 1);
    oled.print("ПОЛ ");
    oled.setCursorXY(0, LINE_HEIGHT * 2);
    oled.print("КАН ");
}

void showSensorParams(uint8_t index)
{
    param_t sp = params[index];
    oled.setCursorXY(sp.x, sp.y);
    oled.print(sp.val);
}

void showParams()
{
    showSensorParams(AIR_INDEX_MIN);
    showSensorParams(AIR_INDEX_MAX);
    showSensorParams(FLO_INDEX_MIN);
    showSensorParams(FLO_INDEX_MAX);
    showSensorParams(CHANNEL_INDEX);
}

void showSensorValue(uint8_t index)
{
    oled.setCursorXY(60, LINE_HEIGHT * index);
    oled.print(datas[index].cur_val, 1);
}

void showSensorValues()
{
    showSensorValue(AIR_INDEX);
    showSensorValue(FLO_INDEX);
}

void showStatic()
{
    oled.clear();
    showLabels();
    showParams();
    oled.update();
}

bool hasChanges(u_int8_t index)
{
    return datas[index].cur_val != datas[index].pre_val;
}

void showState()
{
    if (hasChanges(AIR_INDEX) || hasChanges(FLO_INDEX))
    {
        showSensorValues();
        oled.update();
        Serial.println("changed");
    }
    // const char *dir = heaterOn ? " > " : " < ";
}
#pragma endregion
