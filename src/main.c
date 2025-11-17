#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "FreeRTOSConfig.h"

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bmi160.h"

// SPI pins
#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK 4
#define PIN_NUM_CS 7

// SPI parameters
#define SPI_HOST SPI2_HOST
#define DMA_CHAN SPI_DMA_CH_AUTO

// Global variables
static const char * TAG = "BMI160";
static const char * PERF = "PERF";
spi_device_handle_t spi;
struct bmi160_dev bmi160;
struct bmi160_sensor_data accel_data, gyro_data, accel_bias, gyro_bias;

static void user_delay_ms(uint32_t period) {
	vTaskDelay(pdMS_TO_TICKS(period));
}

// Read from BMI160 register via SPI
int8_t bmi160_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t len) {
	esp_err_t ret;
	spi_transaction_t t = {
		.length = (len + 1) * 8,
		.rxlength = (len + 1) * 8,
	};
	uint8_t txbuf[len + 1];
	uint8_t rxbuf[len + 1];
	txbuf[0] = reg_addr | 0x80;
	t.tx_buffer = txbuf;
	t.rx_buffer = rxbuf;
	t.user = (void * ) 1;

	// ESP_LOGI(TAG, "Reading reg 0x%02x, len=%d", reg_addr, len);
	ret = spi_device_polling_transmit(spi, & t);
	if (ret == ESP_OK) {
		memcpy(data, rxbuf + 1, len);
		if (reg_addr == 0x0C) {
			// ESP_LOGI(TAG, "Sensor data read, first byte: 0x%02x", data[0]);
		} else {
			// ESP_LOGI(TAG, "Read data: 0x%02x", data[0]);
		}
		return BMI160_OK;
	}
	ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
	return BMI160_E_COM_FAIL;
}

// Write to BMI160 register via SPI 
int8_t bmi160_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t len) {
	esp_err_t ret;
	spi_transaction_t t = {
		.length = (len + 1) * 8,
	};
	uint8_t txbuf[len + 1];
	txbuf[0] = reg_addr & 0x7F;
	memcpy( & txbuf[1], data, len);
	t.tx_buffer = txbuf;
	t.user = (void * ) 1;

	ESP_LOGI(TAG, "Writing reg 0x%02x, data=0x%02x, len=%d", reg_addr, data[0], len);
	ret = spi_device_polling_transmit(spi, & t);
	if (ret == ESP_OK) {
		return BMI160_OK;
	}
	ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
	return BMI160_E_COM_FAIL;
}

// Assert CS before transfer
static void pre_transfer_callback(spi_transaction_t * t) {
	if ((int) t -> user) gpio_set_level(PIN_NUM_CS, 0);
}

// Deassert CS after transfer
static void post_transfer_callback(spi_transaction_t * t) {
	if ((int) t -> user) gpio_set_level(PIN_NUM_CS, 1);
}

// UART initialization
int8_t init_UART(void) {
	esp_err_t ret;

	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	ESP_LOGI(TAG, "Initializing UART...");
	ret = uart_param_config(UART_NUM_0, & uart_config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "UART config error: %s", esp_err_to_name(ret));
		return 0;
	}
	ret = uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "UART pin setup error: %s", esp_err_to_name(ret));
		return 1;
	}
	ret = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "UART driver install error: %s", esp_err_to_name(ret));
		return 2;
	}

	return ESP_OK;
}

// Manual accelerometer calibration
void calibrate_accel() {
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
	const int samples = 100;

	// ESP_LOGI(TAG, "Starting accelerometer calibration...");
	for (int i = 0; i < samples; i++) {
		bmi160_get_sensor_data(BMI160_ACCEL_SEL, & accel_data, NULL, & bmi160);
		ax_sum += accel_data.x;
		ay_sum += accel_data.y;
		az_sum += accel_data.z;
		user_delay_ms(10);
	}

	accel_bias.x = ax_sum / samples; // Expect ~0
	accel_bias.y = ay_sum / samples; // Expect ~0
	accel_bias.z = az_sum / samples - (32768.0f / 4.0f); // ~8192 for 1g

	ESP_LOGI(TAG, "Accel bias: X=%d, Y=%d, Z=%d", accel_bias.x, accel_bias.y, accel_bias.z);
}

// Manual gyroscope calibration
void calibrate_gyro() {
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
	const int samples = 100;

	for (int i = 0; i < samples; i++) {
		bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, & gyro_data, & bmi160);
		gx_sum += gyro_data.x;
		gy_sum += gyro_data.y;
		gz_sum += gyro_data.z;
		user_delay_ms(10);
	}

	gyro_bias.x = gx_sum / samples;
	gyro_bias.y = gy_sum / samples;
	gyro_bias.z = gz_sum / samples;
	
	ESP_LOGI(TAG, "Gyro bias: X=%d, Y=%d, Z=%d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
}

// Perform Fast Offset Calibration (FOC)
int8_t bmi160_perform_foc() {
	esp_err_t ret;
	uint8_t foc_conf = 0x3C; // X=0g, Y=0g, Z=1g
	uint8_t cmd = 0x03; // FOC start command

	// Configure FOC_CONF
	ESP_LOGI(TAG, "Configuring FOC...");
	ret = bmi160_spi_write(0, 0x69, & foc_conf, 1);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "Failed to write FOC_CONF: %d", ret);
		return ret;
	}

	// Delay before starting FOC
	user_delay_ms(50);

	// Start FOC
	ESP_LOGI(TAG, "Starting FOC...");
	ret = bmi160_spi_write(0, 0x7E, & cmd, 1);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "Failed to start FOC: %d", ret);
		return ret;
	}

	// Wait for FOC completion (up to 250 ms per datasheet)
	user_delay_ms(300);

	// Check ERR_REG
	uint8_t err_reg;
	ret = bmi160_spi_read(0, 0x02, & err_reg, 1);
	if (ret == BMI160_OK) {
		ESP_LOGI(TAG, "ERR_REG after FOC: 0x%02x", err_reg);
		if (err_reg == 0x00) {
			ESP_LOGI(TAG, "FOC completed successfully");
		} else {
			ESP_LOGE(TAG, "FOC failed, ERR_REG: 0x%02x", err_reg);
			return BMI160_E_COM_FAIL;
		}
	} else {
		ESP_LOGE(TAG, "Failed to read ERR_REG: %d", ret);
		return ret;
	}

	return BMI160_OK;
}

// Sensor data task
void sensor_task(void * arg) {
	while (1) {
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), & accel_data, & gyro_data, & bmi160);

		printf(">AX:%d,AY:%d,AZ:%d\r\n", accel_data.x, accel_data.y, accel_data.z);
		printf(">GX:%d,GY:%d,GZ:%d\r\n", gyro_data.x, gyro_data.y, gyro_data.z);

		user_delay_ms(10);
	}
}

// Performance monitoring task
void perf_task(void * arg) {
	char stats_buffer[255];
	while (1) {
		user_delay_ms(5000);
		vTaskGetRunTimeStats(stats_buffer);
		ESP_LOGI(PERF, "Task Run Time Stats:\n%s\n", stats_buffer);
	}
}

// Initialize BMI160 with DMA
void init_bmi160_dma(void) {
	esp_err_t ret;

	user_delay_ms(500);
	ret = init_UART();
	if (ret == ESP_OK) {
		ESP_LOGI(TAG, "UART initialized successfully");
	} else {
		ESP_LOGE(TAG, "UART initialization failed: %s", esp_err_to_name(ret));
		return;
	}

	user_delay_ms(500);

	// Configure CS GPIO
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << PIN_NUM_CS),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
	ESP_ERROR_CHECK(gpio_config( & io_conf));
	gpio_set_level(PIN_NUM_CS, 1);
	ESP_LOGI(TAG, "GPIO initialized successfully");

	user_delay_ms(500);
	// SPI bus configuration
	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};

	// SPI device configuration
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 500000,
		.mode = 0,
		.spics_io_num = -1,
		.queue_size = 7,
		.pre_cb = pre_transfer_callback,
		.post_cb = post_transfer_callback,
	};

	// Initialize SPI bus with DMA
	ret = spi_bus_initialize(SPI_HOST, & buscfg, DMA_CHAN);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
		return;
	}
	ESP_LOGI(TAG, "SPI initialized successfully");

	user_delay_ms(500);

	// Add SPI device
	ret = spi_bus_add_device(SPI_HOST, & devcfg, & spi);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
		return;
	}
	ESP_LOGI(TAG, "SPI device added successfully");

	user_delay_ms(2000);
	// Verify CHIP_ID
	uint8_t chip_id = 0;
	int8_t res = bmi160_spi_read(0, 0x00, & chip_id, 1);
	if (res == BMI160_OK) {
		ESP_LOGI(TAG, "CHIP_ID: 0x%02x", chip_id);
		if (chip_id != 0xD1) {
			ESP_LOGE(TAG, "Invalid CHIP_ID: expected 0xD1, got 0x%02x", chip_id);
			return;
		}
	} else {
		ESP_LOGE(TAG, "Failed to read CHIP_ID: %d", res);
		return;
	}

	// Check ERR_REG before reset
	uint8_t err_reg = 0;
	res = bmi160_spi_read(0, 0x02, & err_reg, 1);
	if (res == BMI160_OK) {
		ESP_LOGI(TAG, "ERR_REG before reset: 0x%02x", err_reg);
	} else {
		ESP_LOGE(TAG, "Failed to read ERR_REG: %d", res);
	}

	// Soft reset
	uint8_t cmd = 0xB6;
	res = bmi160_spi_write(0, 0x7E, & cmd, 1);
	if (res == BMI160_OK) {
		ESP_LOGI(TAG, "Soft reset command sent successfully");
	} else {
		ESP_LOGE(TAG, "Failed to send soft reset: %d", res);
		return;
	}

	user_delay_ms(1000); // Delay after reset

	// Check ERR_REG after reset
	res = bmi160_spi_read(0, 0x02, & err_reg, 1);
	if (res == BMI160_OK) {
		if (err_reg == 0xFF) {
			ESP_LOGW(TAG, "ERR_REG read 0xFF, retrying...");
			user_delay_ms(100);
			res = bmi160_spi_read(0, 0x02, & err_reg, 1);
		}
		ESP_LOGI(TAG, "ERR_REG after reset: 0x%02x", err_reg);
	} else {
		ESP_LOGE(TAG, "Failed to read ERR_REG: %d", res);
	}

	// Setup BMI160 structure
	bmi160.id = BMI160_SPI_INTF;
	bmi160.read = bmi160_spi_read;
	bmi160.write = bmi160_spi_write;
	bmi160.delay_ms = user_delay_ms;

	// Initialize BMI160
	int8_t rslt = bmi160_init( & bmi160);
	if (rslt != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 init failed: %d", rslt);
		return;
	}
	ESP_LOGI(TAG, "BMI160 initialized successfully");

	uint8_t status;
	do {
		res = bmi160_spi_read(0, 0x1B, & status, 1);
		if (res != BMI160_OK) {
			ESP_LOGE(TAG, "Failed to read STATUS register: %d", res);
			return;
		}
		ESP_LOGI(TAG, "STATUS register: 0x%02x", status);
		user_delay_ms(100);
	} while (!(status & 0x0C)); // Wait for accel/gyro data ready (bits 3 and 2)

	user_delay_ms(500);

	// Configure accelerometer
	bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
	bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	// Configure gyroscope
	bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
	bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	user_delay_ms(500); // Delay before applying config

	// Apply sensor configuration
	rslt = bmi160_set_sens_conf( & bmi160);
	if (rslt != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 config failed: %d", rslt);
		// Check ERR_REG after failure
		res = bmi160_spi_read(0, 0x02, & err_reg, 1);
		if (res == BMI160_OK) {
			ESP_LOGE(TAG, "ERR_REG after config failure: 0x%02x", err_reg);
		}
		return;
	}
	ESP_LOGI(TAG, "BMI160 configured successfully");

	rslt = bmi160_perform_foc();
	if (rslt != BMI160_OK) {
		ESP_LOGE(TAG, "FOC calibration failed: %d", rslt);
		return;
	}
	ESP_LOGI(TAG, "Accelerometer calibrated with FOC");

	user_delay_ms(500);

	calibrate_gyro();
	calibrate_accel();
}

void app_main(void) {
	init_bmi160_dma();

	xTaskCreate(perf_task, "perf_task", 4096, NULL, 2, NULL);
	xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}