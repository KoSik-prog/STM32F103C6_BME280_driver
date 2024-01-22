<h1 align="center">BME280 Library for STM32</h1>

- SPI connection with DMA
- HAL library uses

<h2>How to use:</h2>

<h3>INIT:</h3>

    bme280_setCtrlMeas(BME280_OVRSAMP_8, BME280_OVRSAMP_8, BME280_FORCED);
    bme280_setCtrlHumi(BME280_OVRSAMP_4);
    bme280_setConfig(BME280_TSB_1000, BME280_FILTER_8, BME280_3W_SPI_OFF);

    if (bme280_init(&hspi1, BME280_CS_GPIO_Port, BME280_CS_Pin)) {
        HAL_Delay(1); //error trap
    }

<h3>READ:</h3>

    bme280_startMeasuring();
    HAL_Delay(1000);
    bme280_readSensor();
    temp = (float)bme280_getTeperature() / 100;
    press = bme280_getPressure() / 100;
    humi = (float)bme280_getHumidity() / 1024;

## :memo: License ##
This project is licensed under the MIT License. For more details, please refer to the [LICENSE](LICENSE.md) file.

<br/>
<p align="center">Made by <a href="https://github.com/kosik-prog/" target="_blank">KoSik</a><p/>
<br/>
<a href="#top">Back to top</a>