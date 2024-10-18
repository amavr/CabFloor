#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "DHT.h"
#include "DHT_U.h"
#include <GyverOLED.h>
#include <microDS18B20.h>

#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
#define DHTPIN GPIO_NUM_3 // Digital pin connected to the DHT sensor
DHT_Unified dht(DHTPIN, DHTTYPE);

#define ONE_WIRE_BUS GPIO_NUM_5 // вывод, к которому подключён DS18B20
#define TEMPERATURE_PRECISION 12 // точность измерений (9 ... 12)

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature flo_sensor(&oneWire);
DeviceAddress Thermometer;

// OneWire ds(ONE_WIRE_BUS);

// MicroDS18B20<ONE_WIRE_BUS> flo_sensor;

#define MOTION_PIN GPIO_NUM_12
#define MOTION_CALM_DOWN 2000

GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;

SemaphoreHandle_t sem;
static portMUX_TYPE spin_lock = portMUX_INITIALIZER_UNLOCKED;

void showInfo(float TA, float HA, float TF)
{
    oled.home(); // курсор в 0,0
    oled.print("Воздух: ");
    //    oled.setCursorXY(44, 0); // курсор в 0,0
    oled.print(TA, 1);
    oled.print(" / ");
    //    oled.setCursorXY(64, 0); // курсор в 0,0
    oled.print(HA, 1);
    oled.println("%");

    //    oled.setCursorXY(64, 0); // курсор в 0,0
    oled.print("Пол: ");
    oled.print(TF, 1);
}

void IRAM_ATTR detectsMovement()
{
    BaseType_t woken = pdFALSE;
    portENTER_CRITICAL_ISR(&spin_lock);
    xSemaphoreGiveFromISR(sem, &woken);
    portEXIT_CRITICAL_ISR(&spin_lock);
    if (woken)
    {
        portYIELD_FROM_ISR(woken);
    }
}

void taskMovement(void *pv)
{
    int last_state = 0;
    while (true)
    {
        int motion_sensor = digitalRead(MOTION_PIN);
        if (last_state != motion_sensor)
        {
            last_state = motion_sensor;
            if (motion_sensor == HIGH)
            {
                digitalWrite(LED_BUILTIN, HIGH);

                oled.home(); // курсор в 0,0
                oled.print("Кто здесь?");
            }
            else
            {
                digitalWrite(LED_BUILTIN, LOW);

                oled.home(); // курсор в 0,0
                oled.print("Тихо!");
            }
        }
        vTaskDelay(100);
    }
}

void setup()
{
    // pinMode(ONE_WIRE_BUS, INPUT);


    Serial.begin(115200);

    dht.begin();

    sensor_t dht_sensor;
    dht.temperature().getSensor(&dht_sensor);
    dht.humidity().getSensor(&dht_sensor);

    flo_sensor.begin();                                           // инициализация DS18B20
    flo_sensor.getAddress(Thermometer, 0);                        // адрес DS18B20 (поиск по индексу)
    flo_sensor.setResolution(Thermometer, TEMPERATURE_PRECISION); // установка точности измерения 9...12 разрядов

    // pinMode(MOTION_PIN, INPUT_PULLUP);

    // attachInterrupt(digitalPinToInterrupt(MOTION_PIN), detectsMovement, CHANGE);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    oled.init(GPIO_NUM_12, GPIO_NUM_11); // инициализация
    oled.clear();                        // очистка
    oled.home();                         // курсор в 0,0

    oled.setScale(1); // масштаб текста (1..4)

    oled.print("INIT.");

    // xTaskCreate(taskMovement, "Motion detection", 4096, NULL, 1, CONFIG_ARDUINO_RUNNING_CORE);
}

float tin = 0;
float hin = 0;
float tfl = 0;

byte addr[8];

void loop()
{
    sensors_event_t event;

    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature))
    {
        tin = event.temperature;
    }

    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity))
    {
        hin = event.relative_humidity;
    }

    flo_sensor.requestTemperatures();        // считывание значение температуры
    tfl = flo_sensor.getTempC(Thermometer); // температура в градусах Цельсия

    // if (!ds.search(addr))
    // {
    //     Serial.println("No more addresses.");
    //     Serial.println();
    //     ds.reset_search();
    // }
    // else
    // {
    //     Serial.print("ROM =");
    //     for (int i = 0; i < 8; i++)
    //     {
    //         Serial.write(' ');
    //         Serial.print(addr[i], HEX);
    //     }
    // }

    // flo_sensor.requestTemp();
    // vTaskDelay(1000);
    // if (flo_sensor.readTemp()) {
    //     tfl = flo_sensor.getTemp();
    //     Serial.println(tfl);
    // }
    // else {
    //     Serial.println("Not ready");
    // }

    showInfo(tin, hin, tfl);

    vTaskDelay(1000);
}