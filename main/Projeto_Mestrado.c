#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "driver/i2c.h"

#define LED_ESP32 (GPIO_NUM_2)
#define TXD_UART1 (GPIO_NUM_43)
#define RXD_UART1 (GPIO_NUM_44)
#define RTS_UART (UART_PIN_NO_CHANGE)
#define CTS_UART (UART_PIN_NO_CHANGE)

#define BUF_SIZE 500

// defines canais ADC
#define ADC_CHANNEL ADC_CHANNEL_2
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12

// defines LED's
#define GPIO_LED_19 GPIO_NUM_19
#define GPIO_LED_20 GPIO_NUM_20
#define GPIO_LED_21 GPIO_NUM_21
#define GPIO_LED_47 GPIO_NUM_47

// defines SENSOR INA219
#define I2C_MASTER_SCL_IO GPIO_NUM_4 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_5 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0             /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define INA219_SENSOR_ADDR 0x40      /*!< I2C address for INA219 */
#define INA219_REG_CONFIG 0x00       /*!< Configuration register */
#define INA219_REG_SHUNTVOLTAGE 0x01 /*!< Shunt voltage register */
#define INA219_REG_BUSVOLTAGE 0x02   /*!< Bus voltage register */
#define INA219_REG_POWER 0x03        /*!< Power register */
#define INA219_REG_CURRENT 0x04      /*!< Current register */
#define INA219_REG_CALIBRATION 0x05  /*!< Calibration register */

int Leds_Debug_RTOS[4] = {GPIO_LED_19, GPIO_LED_20, GPIO_LED_21, GPIO_LED_47};

// Função para escrever um byte em um registrador do INA219
static esp_err_t ina219_register_write(uint8_t reg_addr, uint16_t data)
{
    uint8_t write_buf[3] = {reg_addr, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return i2c_master_write_to_device(I2C_MASTER_NUM, INA219_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Função para ler um registrador de 16 bits do INA219
static esp_err_t ina219_register_read(uint8_t reg_addr, uint16_t *data)
{
    uint8_t read_buf[2];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, INA219_SENSOR_ADDR, &reg_addr, 1, read_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        *data = (read_buf[0] << 8) | read_buf[1];
    }
    return ret;
}

// Configuração inicial do INA219
static esp_err_t ina219_init(void)
{
    uint16_t calibration_value = 0x2000; // Valor de calibração, ajustar conforme necessário
    return ina219_register_write(INA219_REG_CALIBRATION, calibration_value);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void xLED(void *arg)
{
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            gpio_set_level(Leds_Debug_RTOS[i], 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(Leds_Debug_RTOS[i], 0);
        }
    }
}

void xLeituraINA333(void *arg)
{
    char conversao[100];
    int adc_raw = 0;
    int voltage = 0;

    // Inicialização do ADC1
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    // Inicialização da calibração ADC1
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan1_handle);

    // Configuração do canal ADC1
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);

    while (1)
    {
        // Leitura bruta do ADC
        adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);

        // Conversão para tensão calibrada
        adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw, &voltage);

        // Conversão para string e envio via UART
        sprintf(conversao, "Tensão convertida: %.4fV\n", (float)adc_raw / 4095 * 3.3);
        uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));

        sprintf(conversao, "Tensão Calibrada: %dmV\n", voltage);
        uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Liberação de recursos (caso saia do loop)
    adc_cali_delete_scheme_curve_fitting(adc1_cali_chan1_handle);
    adc_oneshot_del_unit(adc1_handle);
}

void xLeituraINA219(void *arg)
{
    char conversao[100];
    uint16_t bus_voltage, shunt_voltage, current;

    while (1)
    {
        //Leitura da tensão
        ina219_register_read(INA219_REG_BUSVOLTAGE, &bus_voltage);
        sprintf(conversao, "Bus Voltage: %.2f V\n", (bus_voltage >> 3) * 0.004);
        uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));

        // Leitura da tensão do shunt
        (ina219_register_read(INA219_REG_SHUNTVOLTAGE, &shunt_voltage));
        sprintf(conversao, "Shunt Voltage: %.2f mV\n", shunt_voltage * 0.01);
        uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));

        // Leitura da corrente
        ina219_register_read(INA219_REG_CURRENT, &current);
        sprintf(conversao, "Current: %.2f mA\n", current * 0.01);
        uart_write_bytes(UART_NUM_1, conversao, strlen(conversao));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(ina219_init());

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << GPIO_LED_19) | (1ULL << GPIO_LED_20) | (1ULL << GPIO_LED_21) | (1ULL << GPIO_LED_47));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, TXD_UART1, RXD_UART1, RTS_UART, CTS_UART);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreatePinnedToCore(xLED, "xLED", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(xLeituraINA333, "xLeituraINA333", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(xLeituraINA219, "xLeituraINA219", 4096, NULL, 2, NULL, 0);
}
