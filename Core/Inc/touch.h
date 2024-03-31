#include "stdio.h"
#include "touch-defines.h"

// plan is to try stmpe. if it flops, switch to manual
// i2c1_sda is PB9, i2c1_scl is PB8

void writeRegister8(I2C_HandleTypeDef* i2c, uint8_t reg, uint8_t value){
	uint8_t buffer[2] = {reg, value};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c, STMPE_ADDR << 1, buffer, 2, 10000);
	if (status != HAL_OK) {
		printf("I2C write to STMPE failed with %d.\r\n", status);
	}
}

uint8_t readRegister8(I2C_HandleTypeDef* i2c, uint8_t reg) {
	uint8_t buffer[1] = {reg};
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(i2c, (STMPE_ADDR << 1) | 1, buffer, 1, 10000);
	if (status != HAL_OK) {
		printf("I2C read from STMPE failed with %d.\r\n", status);
	}
	return *buffer;
}

void initialize_touch(I2C_HandleTypeDef* i2c) {
	// TODO: only thing I can think of is that you need to read the version here for some reason

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
	                 STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);

	  printf("Finished touch setup.\r\n");

	  if (readRegister8(i2c, STMPE_TSC_CTRL) & 0x80) printf("experienced touch");
	  else printf("did not receive touch");
}
