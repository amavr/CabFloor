// ----------------------------------------
// 1. Вентиляция
// ----------------------------------------
// ДАТЧИКИ ОПРАШИВАЕМЫЕ
#define BYT_SNS_CO2_FAN  0x11 // (у вентилятора)
#define BYT_SNS_TMP_FAN  0x12 // (у вентилятора)
#define BYT_SNS_HUM_FAN  0x13 // (у вентилятора)

// ДАТЧИКИ СОБЫТИЙНЫЕ
#define BYT_SNS_BTN_FAN  0x21 // (вентилятор)

// УСТРОЙСТВА
#define BYT_CTR_FAN_REL  0x31 // (реле вентилятора)

// ----------------------------------------
// 2. Отопление
// ----------------------------------------
// ДАТЧИКИ ОПРАШИВАЕМЫЕ
#define BYT_SNS_TMP_TAB 0x14 // (воздух у стола)
#define BYT_SNS_HUM_TAB 0x15 // (воздух у стола)
#define BYT_SNS_TMP_FLR 0x16 // (пол у стола)
#define BYT_SNS_TMP_EXT 0x17 // (у воздухо-забора)
#define BYT_SNS_HUM_EXT 0x18 // (у воздухо-забора)

// ДАТЧИКИ СОБЫТИЙНЫЕ
#define BYT_SNS_BTN_HTR 0x22 (тепл.пол)

// УСТРОЙСТВА
#define BYT_CTR_HTR_REL 0x32 // (реле тепл.пола)

// ----------------------------------------
// 3. Освещение
// ----------------------------------------
// ДАТЧИКИ ОПРАШИВАЕМЫЕ
#define BYT_SNS_LHT_FAN 0x19 // (у вентилятора)
#define BYT_SNS_MOV_FAN 0x1A // (у воздухо-забора)
#define BYT_SNS_LHT_EXT 0x1B // (внешн. у вентилятора)
#define BYT_SNS_MOV_EXT 0x1C // (внешн. у воздухо-забора)

// ДАТЧИКИ СОБЫТИЙНЫЕ
#define BYT_SNS_BTN_TOP 0x23 // (верхнее освещение)
#define BYT_SNS_BTN_EXT 0x24 // (внешнее освещение)
#define BYT_SNS_BTN_BR1 0x25 // (бра 1)
#define BYT_SNS_BTN_BR2 0x26 // (бра 1)
#define BYT_SNS_BTN_DSK 0x27 // (настольная лампа)
#define BYT_SNS_BTN_NGT 0x28 // (ночник)

// УСТРОЙСТВА
#define BYT_CTR_LMP_TOP 0x33 // (верхнее освещение)
#define BYT_CTR_LMP_EXT 0x34 // (внешнее освещение)
#define BYT_CTR_LMP_BR1 0x35 // (бра 1)
#define BYT_CTR_LMP_BR2 0x36 // (бра 1)
#define BYT_CTR_LMP_DSK 0x37 // (настольная лампа)
#define BYT_CTR_LMP_NGT 0x38 // (ночник)


// ----------------------------------------
// 4. Энергия
// ----------------------------------------
// ДАТЧИКИ ОПРАШИВАЕМЫЕ
#define BYT_SNS_ENG_EXT 0x1D // (щит)

// ----------------------------------------
// 5. Доступ
// ----------------------------------------
// ДАТЧИКИ СОБЫТИЙНЫЕ
#define BYT_SNS_OPN_DOR 0x29 // (дверь)