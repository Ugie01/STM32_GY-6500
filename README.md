# GY-6500 센서 라이브러리 만들기
## 현재 아래 내용들 구현
void **Scanning_I2C()**;    ->    연결된 I2C 주소 찾아서 출력

HAL_StatusTypeDef **Gy6500_init(GyData *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address)**;    ->    GY-6500 센서 연결 후 SLEEP 해제

HAL_StatusTypeDef **getAllData(GyData *dev)**;    ->    가속도, 자이로, 온도 데이터 받기

HAL_StatusTypeDef **getAccelerometer(GyData *dev)**;    ->    가속도 데이터 받기

HAL_StatusTypeDef **getGyroscope(GyData *dev)**;    ->    자이로 데이터 받기

HAL_StatusTypeDef **getTemp(GyData *dev)**;    ->    온도 데이터 받기

HAL_StatusTypeDef **setSensorState(GyData *dev, uint8_t setData)**;    ->    가속도, 자이로 데이터 ON/OFF

HAL_StatusTypeDef **setTempState(GyData *dev, bool set_data)**;    ->    온도 데이터 ON/OFF

HAL_StatusTypeDef **setGyroRange(GyData *dev, gyro_range_t range)**;    ->    자이로 데이터 범위 변환

HAL_StatusTypeDef **setAccelRange(GyData *dev, accel_range_t range)**;    ->    가속도 데이터 범위 변환

HAL_StatusTypeDef **setDLPF(GyData *dev, dlpf_mode_t mode)**;    ->    디지털 로우 패스 필터 실행
