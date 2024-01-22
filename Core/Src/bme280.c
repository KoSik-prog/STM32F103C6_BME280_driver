/*
 * bme280.c
 *
 *  Created on: 20-01-2024
 *      Author: Marcin Kosela (KoSik)
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "bme280.h"

struct BME280 bme280;
uint8_t spiBusyFlag = 0;

#define bme280_20bit_reg(b1, b2, b3) ( \
	((int32_t)(b1) << 12) \
	| ((int32_t)(b2) << 4) \
	| ((int32_t)(b3) >> 4) \
)

#define bme280_12bit_reg(b1, b2) ( \
	  ((int32_t)(b1) << 8) \
	| (int32_t)(b2) \
)

/*
 * Function create config setting:
 * @param tSb - Controls inactive duration tstandby in normal mode
 * @param filter - Controls the time constant of the IIR filter
 * @param spi3w - Enables 3-wire SPI interface when set to ‘1’
 * */
void bme280_setConfig(uint8_t tSb, uint8_t filter, uint8_t spi3w) {
	bme280.config = (tSb & 0b00000111) << 5 | (filter & 0b00000111) << 2
			| (spi3w & 0b00000001);
}

/*
 * Function create ctrl_meas setting:
 * @param osrsT - Controls oversampling of temperature data
 * @param osrsP - Controls oversampling of pressure data
 * @param mode - Controls the power mode of the device
 * */
void bme280_setCtrlMeas(uint8_t osrsT, uint8_t osrsP, uint8_t mode) {
	bme280.ctrlMeas = (osrsT & 0b00000111) << 5 | (osrsP & 0b00000111) << 2
			| (mode & 0b00000011);
}

/*
 * Function create ctrl_hum setting:
 * @param osrsH - Controls oversampling of humidity data
 * */
void bme280_setCtrlHumi(uint8_t osrsH) {
	bme280.ctrlHum = (osrsH & 0b00000111);
}

/*
 * Init Function:
 * @param hspi - SPI handle pointer
 * @param port - port definition pointer to chip select (CS) pin
 * @param pin - chip select (CS) pin definition
 * @retval Status
 * */
HAL_StatusTypeDef bme280_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port,
		uint16_t pin) {
	bme280.hspi = hspi;
	bme280.CsPort = port;
	bme280.CsPin = pin;
	bme280_CsPinDisable();
	bme280.id = bme280_readRegister(BME280_ID);

	bme280_readCalibration();
	bme280_writeRegister(BME280_CONFIG, bme280.config);
	bme280_writeRegister(BME280_CTRL_HUM, bme280.ctrlHum);
	bme280_writeRegister(BME280_CTRL_MEAS, bme280.ctrlMeas);
	if (bme280.id == BME280_ID_NUMBER) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef bme280_reset(void) {
	HAL_StatusTypeDef res;
	res = bme280_writeRegister(RESET, 0xB6);
	HAL_Delay(10);
	return res;
}

void bme280_startMeasuring(void) {
	bme280_writeRegister(BME280_CTRL_MEAS, bme280.ctrlMeas);
}

uint8_t bme280_readRegister(uint8_t rreg) {
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[2];
	uint8_t rxData[2];
	for (uint8_t i = 0; i < sizeof(txData); i++) {
		txData[i] = 0x00;
	}

	bme280_CsPinEnable();
	txData[0] = rreg | BME280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 2);
	while (spiBusyFlag) {
		delayUs(1);
	}
	if (res == HAL_OK) {
		return rxData[1];
	} else {
		return 0x00;
	}
}

HAL_StatusTypeDef bme280_writeRegister(uint8_t rreg, uint8_t data) {
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[2];
	uint8_t rxData[2];
	for (uint8_t i = 0; i < sizeof(txData); i++) {
		txData[i] = 0x00;
	}
	bme280_CsPinEnable();
	txData[0] = rreg;
	txData[0] &= ~(1 << 7);
	txData[1] = data;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 2);
	while (spiBusyFlag) {
		delayUs(1);
	}
	return res;
}

HAL_StatusTypeDef bme280_readSensor(void) {
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[9];
	uint8_t rxData[9];
	int32_t temp_raw, pres_raw, humi_raw, t_fine;
	int64_t var1, var2, press;
	for (uint8_t i = 0; i < sizeof(txData); i++) {
		txData[i] = 0x00;
	}

	bme280_CsPinEnable();
	txData[0] = BME280_PRESS_MSB | BME280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 9);
	while (spiBusyFlag) {
		delayUs(1);
	}
	pres_raw = bme280_20bit_reg(rxData[1], rxData[2], rxData[3]);
	temp_raw = bme280_20bit_reg(rxData[4], rxData[5], rxData[6]);
	humi_raw = bme280_12bit_reg(rxData[7], rxData[8]);

	// TEMPERATURE
	var1 = ((((temp_raw >> 3) - ((int32_t) bme280.dig_T1 << 1)))
			* ((int32_t) bme280.dig_T2)) >> 11;
	var2 = (((((temp_raw >> 4) - ((int32_t) bme280.dig_T1))
			* ((temp_raw >> 4) - ((int32_t) bme280.dig_T1))) >> 12)
			* ((int32_t) bme280.dig_T3)) >> 14;
	t_fine = var1 + var2;
	bme280.temperature = (t_fine * 5 + 128) >> 8;

	// PRESSURE
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * ((int64_t) bme280.dig_P6);
	var2 = var2 + ((var1 * ((int64_t) bme280.dig_P5)) << 17);
	var2 = var2 + (((int64_t) bme280.dig_P4) << 35);
	var1 = ((var1 * var1 * (uint64_t)bme280.dig_P3) >> 8) + ((var1 * (int64_t)bme280.dig_P2) << 12);
	var1 = (((((int64_t)1)<<47) + var1)) * ((int64_t)bme280.dig_P1) >> 33;
	if (var1 == 0) {
		bme280.pressure = 0;
	} else {
		press = 1048576 - pres_raw;
		press = (((press << 31) - var2) * 3125) / var1;
		var1 = (((int64_t)bme280.dig_P9) * (press >> 13) * (press >> 13)) >> 25;
		var2 = (((int64_t)bme280.dig_P8) * press) >> 19;
		press = ((press + var1 + var2) >> 8) + (((int64_t)bme280.dig_P7) << 4);
		bme280.pressure = (uint64_t)press >> 8;
	}
	//HUMIDITY
	int32_t humi;
	humi = (((int32_t) t_fine) >> 1) - (int32_t) 38400;
	humi = (((((humi_raw << 14) - (((int32_t) bme280.dig_h4) << 20)
				- (((int32_t) bme280.dig_h5) * humi)) + ((int32_t) 16384)) >> 15)
				* (((((((humi * ((int32_t) bme280.dig_h6)) >> 10)
						* (((humi * ((int32_t) bme280.dig_h3)) >> 11)
								+ ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
						* ((int32_t) bme280.dig_h2) + 8192) >> 14));
	humi =
			(humi
					- (((((humi >> 15) * (humi >> 15)) >> 7)
							* ((int32_t) bme280.dig_h1)) >> 4));
	humi = (humi < 0 ? 0 : humi);
	humi = (humi > 419430400 ? 419430400 : humi);
	bme280.humidity = (uint64_t) (humi >> 12);
	return res;
}

uint64_t bme280_getTeperature(void) {
	return bme280.temperature;
}

uint64_t bme280_getPressure(void) {
	return bme280.pressure;
}

uint64_t bme280_getHumidity(void) {
	return bme280.humidity;
}

HAL_StatusTypeDef bme280_readCalibration(void) {
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[28];
	uint8_t rxData[28];
	for (uint8_t i = 0; i < sizeof(txData); i++) {
		txData[i] = 0x00;
	}

	spiBusyFlag = 1;
	bme280_CsPinEnable();
	txData[0] = BME280_CALIBRATION | BME280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 26);
	while (spiBusyFlag) {
		delayUs(1);
	}
	bme280.dig_T1 = rxData[2] << 8 | rxData[1];    //0x88 / 0x89
	bme280.dig_T2 = rxData[4] << 8 | rxData[3];    //0x8A / 0x8B
	bme280.dig_T3 = rxData[6] << 8 | rxData[5];    //0x8C / 0x8D
	bme280.dig_P1 = rxData[8] << 8 | rxData[7];    //0x8E / 0x8F
	bme280.dig_P2 = rxData[10] << 8 | rxData[9];   //0x90 / 0x91
	bme280.dig_P3 = rxData[12] << 8 | rxData[11];  //0x92 / 0x93
	bme280.dig_P4 = rxData[14] << 8 | rxData[13];  //0x94 / 0x95
	bme280.dig_P5 = rxData[16] << 8 | rxData[15];  //0x96 / 0x97
	bme280.dig_P6 = rxData[18] << 8 | rxData[17];  //0x98 / 0x99
	bme280.dig_P7 = rxData[20] << 8 | rxData[19];  //0x9A / 0x9B
	bme280.dig_P8 = rxData[22] << 8 | rxData[21];  //0x9C / 0x9D
	bme280.dig_P9 = rxData[24] << 8 | rxData[23];  //0x9E / 0x9F

	bme280_CsPinEnable();
	txData[0] = BME280_CALIBRATION_H1 | BME280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 2);
	while (spiBusyFlag) {
		delayUs(1);
	}
	bme280.dig_h1 = rxData[1];					   //0xA1

	bme280_CsPinEnable();
	txData[0] = BME280_CALIBRATION_HUMI | BME280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bme280.hspi, txData, rxData, 9);
	while (spiBusyFlag) {
		delayUs(1);
	}
	bme280.dig_h2 = rxData[2] << 8 | rxData[1];  //0xE1 / 0xE2
	bme280.dig_h3 = rxData[3];					   //0xE3
	bme280.dig_h4 = rxData[4] << 4 | (rxData[5] & 0x0F);  //0xE4 / 0xE5&0x0F
	bme280.dig_h5 = rxData[6] << 4 | (rxData[5] >> 4);  //0xE6 / 0xE5>>4
	bme280.dig_h6 = rxData[7];					   //0xE7
	return res;
}

/*
 * SPI CALLBACK - CS PIN DISABLE WHEN TRANMIT COMPLETE
 * */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (bme280.hspi == hspi) {
		bme280_CsPinDisable();
		spiBusyFlag = 0;
	}
}

void bme280_CsPinDisable(void) {
	HAL_GPIO_WritePin(bme280.CsPort, bme280.CsPin, 1);
}

void bme280_CsPinEnable(void) {
	// LOW is enable
	HAL_GPIO_WritePin(bme280.CsPort, bme280.CsPin, 0);
	spiBusyFlag = 1;
}

uint32_t getUs(void) {
	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs() - start < (uint32_t) micros) {
		asm("nop");
	}
}

