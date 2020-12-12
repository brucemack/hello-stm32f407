#include <stdio.h>
#include "STM32_HAL_Interface.h"
#include "si5351.h"

extern "C" {
	I2C_HandleTypeDef hi2c1;
}

static STM32_HAL_Interface i2c_interface(&hi2c1);
static Si5351 si5351(0x60, &i2c_interface);

extern "C" {
	void CppMain_setup() {

		  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
		  if (i2c_found) {
			  printf("Init good!\n");
			  si5351.set_freq(700000000ULL, SI5351_CLK0);
		  } else {
			  printf("Si5351 initialization failed\n");
		  }
	}

	void CppMain_loop() {
	}
}





