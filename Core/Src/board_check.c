#include "board_check.h"

#if ENABLE_BOARD_CHECK
#include "spi.h"
#include "usart.h"
#include "MyIIC_2.h"
#include <stdio.h>
#include <string.h>

#define CHECK_SPI_TIMEOUT 50U
#define CHECK_UART_TIMEOUT 200U

typedef struct {
    const char *name;
    uint8_t passed;
    char detail[96];
} BoardCheckResult;

static BoardCheckResult results[] = {
    {"MIMU ACC/GYRO", 0, ""}, {"MIMU MAG", 0, ""},
    {"ADC ADS124S06", 0, ""}, {"PPG2 MAX30101", 0, ""},
    {"BLE HJ-131IMH", 0, ""}
};

static void Check_Log(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), CHECK_UART_TIMEOUT);
}

static void Check_Report(const BoardCheckResult *result)
{
    char line[160];
    snprintf(line, sizeof(line), "[%s] %s: %s\r\n",
             result->passed == 3 ? "MANUAL" : result->passed == 2 ? "SKIP" : result->passed ? "PASS" : "FAIL", result->name, result->detail);
    Check_Log(line);
}

static void Check_IMU(BoardCheckResult *result, uint16_t cs, uint8_t expected)
{
    uint8_t identities[3] = {0};
    HAL_StatusTypeDef status[3];
    result->passed = 1;
    for (unsigned i = 0; i < 3; ++i) {
        uint8_t tx[2] = {0x8F, 0}, rx[2] = {0};
        HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin | CS_M_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CS_A_G_GPIO_Port, cs, GPIO_PIN_RESET);
        status[i] = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), CHECK_SPI_TIMEOUT);
        HAL_GPIO_WritePin(CS_A_G_GPIO_Port, cs, GPIO_PIN_SET);
        identities[i] = rx[1];
        if (status[i] != HAL_OK || identities[i] != expected) result->passed = 0;
        HAL_Delay(10);
    }
    snprintf(result->detail, sizeof(result->detail), "SPI=%u/%u/%u WHO_AM_I=0x%02X/0x%02X/0x%02X expected=0x%02X",
             (unsigned)status[0], (unsigned)status[1], (unsigned)status[2],
             identities[0], identities[1], identities[2], expected);
}

static void Check_ADC(BoardCheckResult *result)
{
    uint8_t tx[3] = {0x20, 0, 0}, rx[3] = {0};
    HAL_GPIO_WritePin(START_CONV_GPIO_Port, START_CONV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), CHECK_SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    /* DEV_ID is bits 2:0; reserved upper bits are not an identity check. */
    result->passed = status == HAL_OK && (rx[2] & 0x07U) == 0x01U;
    snprintf(result->detail, sizeof(result->detail), "SPI=%u ID=0x%02X expected DEV_ID=1",
             (unsigned)status, rx[2]);
}

#if PPG_DEFAULT_CHANNEL != 0
static uint8_t Check_PPGRead(uint8_t reg, uint8_t *value)
{
    if (HAL_GPIO_ReadPin(I2C_SCL2_GPIO_Port, I2C_SCL2_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(I2C_SDA2_GPIO_Port, I2C_SDA2_Pin) != GPIO_PIN_SET)
        return 0;
    IIC2_Start();
    IIC2_Send_Byte(0xAE);
    if (IIC2_Wait_Ack()) goto failed;
    IIC2_Send_Byte(reg);
    if (IIC2_Wait_Ack()) goto failed;
    IIC2_Start();
    IIC2_Send_Byte(0xAF);
    if (IIC2_Wait_Ack()) goto failed;
    *value = IIC2_Read_Byte(0);
    IIC2_Stop();
    return 1;
failed:
    IIC2_Stop();
    return 0;
}

static void Check_PPG(BoardCheckResult *result)
{
    uint8_t part = 0, revision = 0;
    uint8_t ack = Check_PPGRead(0xFF, &part) && Check_PPGRead(0xFE, &revision);
    result->passed = ack && part == 0x15;
    snprintf(result->detail, sizeof(result->detail), "IIC2=%s PART_ID=0x%02X expected=0x15 REV_ID=0x%02X",
             ack ? "ACK" : "NO_ACK/BUS_LOW", part, revision);
}
#endif

/* USB-TTL TX and BLE TX are hardwired together at PA3 on this PCB.
 * No UART response can reliably determine module health on this shared RX. */
static void Check_BLE(BoardCheckResult *result)
{
    HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);
    result->passed = 3;
    snprintf(result->detail, sizeof(result->detail),
#if ENABLE_BLE_CONFIG
             "SETUP_UNVERIFIED; shared RX; confirm connection/data via HJ-380; disable init next flash");
#else
             "CONFIG_PRESERVED; shared RX; confirm connection/data via HJ-380");
#endif
}

void BoardCheck_PrintSummary(void)
{
    unsigned passed = 0, skipped = 0, manual = 0;
    Check_Log("\r\n=== BOARD COMMUNICATION CHECK SUMMARY ===\r\n");
    for (unsigned i = 0; i < sizeof(results) / sizeof(results[0]); ++i) {
        Check_Report(&results[i]);
        passed += results[i].passed == 1;
        skipped += results[i].passed == 2;
        manual += results[i].passed == 3;
    }
    char line[96];
    snprintf(line, sizeof(line), "TOTAL: PASS=%u FAIL=%u SKIP=%u MANUAL=%u; PPG1=NOT_TESTED\r\n",
             passed, (unsigned)(sizeof(results) / sizeof(results[0])) - passed - skipped - manual, skipped, manual);
    Check_Log(line);
    Check_Log("BLE reset released. HJ-380 confirmation pending; setup writes are not proof of success. Reset board to rerun.\r\n");
}

void BoardCheck_Run(void)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin | CS_M_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(START_CONV_GPIO_Port, START_CONV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    Check_Log("\r\nBOARD COMMUNICATION CHECK / PPG2 optional / 115200 8N1\r\n");
#if ENABLE_BLE_CONFIG
    Check_Log("BLE setup commands sent via acquisition init; readback unavailable on shared RX. Disable init next flash.\r\n");
#else
    Check_Log("BLE config preserved. For a new/unconfigured board, enable first-board BLE init and flash once.\r\n");
#endif
    Check_Log("USB-TTL: RX -> PA2, GND -> GND. TX is hardwired with BLE TX at PA3; no PC writes. Auto start in 3 seconds.\r\n");
    HAL_Delay(3000);
    for (unsigned i = 0; i < sizeof(results) / sizeof(results[0]); ++i) {
        results[i].passed = 0;
        results[i].detail[0] = '\0';
        char line[64];
        snprintf(line, sizeof(line), "\r\nSTEP %u/5: %s\r\n", i + 1, results[i].name);
        Check_Log(line);
        switch (i) {
        case 0: Check_IMU(&results[i], CS_A_G_Pin, 0x68); break;
        case 1:
            results[i].passed = 2;
            snprintf(results[i].detail, sizeof(results[i].detail), "MAG excluded; A/G identity is the MIMU criterion");
            break;
        case 2: Check_ADC(&results[i]); break;
        case 3:
#if PPG_DEFAULT_CHANNEL == 0
            results[i].passed = 2;
            snprintf(results[i].detail, sizeof(results[i].detail), "external PPG board not connected / check disabled");
#else
            Check_PPG(&results[i]);
#endif
            break;
        case 4: Check_BLE(&results[i]); break;
        }
        Check_Report(&results[i]);
        HAL_Delay(500);
    }
    BoardCheck_PrintSummary();
}
#endif
