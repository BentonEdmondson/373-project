#include "stdio.h"
#include "touch-defines.h"

// plan is to try stmpe. if it flops, switch to manual
// i2c1_sda is PB9, i2c1_scl is PB8

void writeRegister8(I2C_HandleTypeDef* i2c, uint8_t reg, uint8_t value){
	uint8_t buffer[2] = {reg, value};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c, STMPE_ADDR << 1, buffer, 2, 1000);
	if (status != HAL_OK) {
		printf("I2C write to STMPE failed with %d.\r\n", status);
	}
}

uint8_t readRegister8(I2C_HandleTypeDef* i2c, uint8_t reg) {
	// problem is don't know what restart is
	// You are supposed to i2c write the address, then do an empty i2c read
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c, STMPE_ADDR << 1, &reg, 1, 1000);
	if (status != HAL_OK) {
		printf("I2C write (for the purpose of reading) to STMPE failed with %d.\r\n", status);
	}

	uint8_t result = 0;
	// TODO: maybe the byte value here should be 1
	status = HAL_I2C_Master_Receive(i2c, (STMPE_ADDR << 1) | 1, &result, 1, 1000);
	if (status != HAL_OK) {
		printf("I2C read from STMPE failed with %d.\r\n", status);
		return 0;
	}
	return result;
}

// 1 is true, 0 is false
uint8_t touched(I2C_HandleTypeDef* i2c) {
	return readRegister8(i2c, STMPE_TSC_CTRL) & 0x80;
}

uint8_t bufferEmpty(I2C_HandleTypeDef* i2c) {
	return (readRegister8(i2c, STMPE_FIFO_STA) & STMPE_FIFO_STA_EMPTY);
}

void readPosition(I2C_HandleTypeDef* i2c, uint16_t *x, uint16_t *y, uint8_t *z) {
  uint8_t data[4];

  for (uint8_t i = 0; i < 4; i++) {
    data[i] = readRegister8(i2c, 0xD7);
  }
  *x = data[0];
  *x <<= 4;
  *x |= (data[1] >> 4);
  *y = data[1] & 0x0F;
  *y <<= 8;
  *y |= data[2];
  *z = data[3];
}

// from: https://github.com/arduino/ArduinoCore-API/blob/0c853c5cded2768122fae258d42b2b4c06cdb3b1/api/Common.cpp
uint16_t map(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t within(uint16_t x, uint16_t y, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
	return (x > x1) && (x < x2) && (y > y1) && (y < y2);
}

const uint16_t yend = 479;
const uint16_t xend = 319;
void touchHook(I2C_HandleTypeDef* i2c) {
	writeRegister8(i2c, STMPE_INT_STA, 0xFF);
	uint16_t x, y;
	uint8_t z;

	// clear the buffer and take the last thing from the buffer
	while (!bufferEmpty(i2c)) {
		readPosition(i2c, &x, &y, &z);
	}

	x = map(x, 3520, 750, 0, 319);
	y = map(y, 3700, 750, 0, 479);

	if (within(x, y, 25, 100, 50, yend-50)) {
		printf("Got a touch: (%d, %d) (within the bottom button)\r\n", x, y);
	} else if (within(x, y, 125, 200, 50, yend-50)) {
		printf("Got a touch: (%d, %d) (within the top button)\r\n", x, y);
	}
}

void initialize_touch(I2C_HandleTypeDef* i2c) {
	// TODO: only thing I can think of is that you need to read the version here for some reason
	// the problem is that I need to do some sort of request response thing, for read
	// it is a write then request

	printf("Initializing touch.\r\n");

	  uint16_t v;
	  v = readRegister8(i2c, 0);
	  v <<= 8;
	  v |= readRegister8(i2c, 1);
	  printf("STMPE version is 0x%x\r\n", v);

	  writeRegister8(i2c, STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);

	  HAL_Delay(10);

	  for (uint8_t i = 0; i < 65; i++) {
	    readRegister8(i2c, i);
	  }

	  writeRegister8(i2c, STMPE_SYS_CTRL2, 0x0); // turn on clocks!
	  writeRegister8(i2c, STMPE_TSC_CTRL,
	                 STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
	  // Serial.println(readRegister8(STMPE_TSC_CTRL), HEX);
	  writeRegister8(i2c, STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
	  writeRegister8(i2c, STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT |
	                                      (0x6 << 4)); // 96 clocks per conversion
	  writeRegister8(i2c, STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
	  writeRegister8(i2c, STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE |
	                                    STMPE_TSC_CFG_DELAY_1MS |
	                                    STMPE_TSC_CFG_SETTLE_5MS);
	  writeRegister8(i2c, STMPE_TSC_FRACTION_Z, 0x6);
	  writeRegister8(i2c, STMPE_FIFO_TH, 1);
	  writeRegister8(i2c, STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
	  writeRegister8(i2c, STMPE_FIFO_STA, 0); // unreset
	  writeRegister8(i2c, STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
	  writeRegister8(i2c, STMPE_INT_STA, 0xFF); // reset all ints
	  writeRegister8(i2c, STMPE_INT_CTRL,
	                 STMPE_INT_CTRL_POL_LOW | STMPE_INT_CTRL_EDGE | STMPE_INT_CTRL_ENABLE);

	  printf("Finished touch setup.\r\n");

//	  while (1) {
//		  uint8_t interrupt = readRegister8(i2c, 0x0A);
//		  printf("Interrupt enables: 0x%x\r\n", interrupt);
//		  if (touched(i2c)) {
//			  //while (!bufferEmpty(i2c)) {
//				  uint16_t x, y;
//				  uint8_t z;
//				  readPosition(i2c, &x, &y, &z);
//				  //printf("yes touch: (%d, %d, %d)\r", x, y, z);
//			  //}
//			  writeRegister8(i2c, STMPE_INT_STA, 0xFF);
//		  } else {
//			  uint16_t x, y;
//			  uint8_t z;
//			  readPosition(i2c, &x, &y, &z);
//			  //printf("yes touch: (%d, %d, %d)\r", x, y, z);
//			  //printf("no  touch: \r");
//		  }
//	  }
}
