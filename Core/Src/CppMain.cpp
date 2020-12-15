#include <stdio.h>
#include "STM32_HAL_Interface.h"
#include "si5351.h"
#include "utils/SerialDriver.h"

extern "C" {
	I2C_HandleTypeDef hi2c1;
	UART_HandleTypeDef huart1;

	HAL_StatusTypeDef writeCodec(uint8_t addr, uint8_t data);
	uint8_t readCodec(uint8_t addr);

	uint8_t encodeG0(int16_t gain);
	int16_t decodeG0(uint8_t g);
	uint8_t encodeG1(int16_t gain);
	int16_t decodeG1(uint8_t g);
}

static STM32_HAL_Interface i2c_interface(&hi2c1);
static Si5351 si5351(0x60, &i2c_interface);

// The driver for UART1
static SerialDriver sd1(&huart1);

class TestHandler : public MessageHandler {
public:
	void onMessage(const char* m) {
		sd1.send("RX[");
		sd1.send(m);
		sd1.send("]\r\n");

		// Commands
		if (m[0] == 'a') {
			// Page 0
			writeCodec(0x00, 0x00);
			//HAL_Delay(5);
			int vl = decodeG1(readCodec(0x41));
			//HAL_Delay(5);
			char msg[32];
			sprintf(msg,"DAC Volume: %d\r\n",vl);
			sd1.send(msg);
			if (vl < 48)
				vl += 1;
			writeCodec(0x41, encodeG1(vl));
			//HAL_Delay(5);
			writeCodec(0x42, encodeG1(vl));
			//HAL_Delay(5);
		}
		else if (m[0] == 'z') {
			// Page 0
			writeCodec(0x00, 0x00);
			//HAL_Delay(5);
			int vl = decodeG1(readCodec(0x41));
			//HAL_Delay(5);
			char msg[32];
			sprintf(msg,"DAC Volume: %d\r\n",vl);
			sd1.send(msg);
			if (vl > -127)
				vl -= 1;
			writeCodec(0x41, encodeG1(vl));
			//HAL_Delay(5);
			writeCodec(0x42, encodeG1(vl));
			//HAL_Delay(5);
		}
		else if (m[0] == 's') {
			// Page 1
			writeCodec(0x00, 0x01);
			int vl = decodeG0(readCodec(0x10) & 0b00111111);
			char msg[32];
			sprintf(msg,"G0: %d\r\n",vl);
			sd1.send(msg);
			if (vl < 29)
				vl += 1;
			writeCodec(0x10,encodeG0(vl));
			writeCodec(0x11,encodeG0(vl));
		}
		else if (m[0] == 'x') {
			// Page 1
			writeCodec(0x00, 0x01);
			int vl = decodeG0(readCodec(0x10) & 0b00111111);
			char msg[32];
			sprintf(msg,"G0: %d\r\n",vl);
			sd1.send(msg);
			if (vl > -6)
				vl += -1;
			writeCodec(0x10,encodeG0(vl));
			writeCodec(0x11,encodeG0(vl));
		}

	}
};

TestHandler handler;

extern "C" {

	// NEEDED FOR SERIAL WIRE DEBUG
	int _write(int file, char* ptr, int len) {
		for (int i = 0; i < len; i++)
			sd1.sendChar((*ptr++));
		return len;
	}

	// This function is called by the HAL
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		if (huart->Instance == USART1)
		{
			sd1.onReceive(huart);
		}
	}

	void CppMain_setup() {

		bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
		if (i2c_found) {
		  printf("Init good!\n");
		  si5351.set_freq(700000000ULL, SI5351_CLK0);
		} else {
		  printf("Si5351 initialization failed\n");
		}

		// Setup the serial driver
		sd1.setMessageHandler(&handler);
		sd1.setEOM("\r");
		sd1.start();
		sd1.send("Hello World!\n");
	}

	void CppMain_loop() {
	}
}





