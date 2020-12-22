#include <stdio.h>
#include <stdlib.h>
#include "STM32_HAL_Interface.h"
#include "si5351.h"
#include "utils/SerialDriver.h"
#include "utils/SystemEnv.h"
#include "utils/Debouncer.h"

#include "main.h"

extern "C" {
	I2C_HandleTypeDef hi2c1;
	UART_HandleTypeDef huart1;

	HAL_StatusTypeDef writeCodec(uint8_t addr, uint8_t data);
	uint8_t readCodec(uint8_t addr);

	void __enable_irq();
	void __disable_irq();

	extern float balance;
	extern float phaseAdjust;
}

int8_t getHPLGain();
void setHPLGain(int8_t gainDb);
int8_t getHPRGain();
void setHPRGain(int8_t gainDb);
int8_t getDACLVolume();
void setDACLVolume(int8_t gainDb2);
int8_t getDACRVolume();
void setDACRVolume(int8_t gainDb2);
int8_t getPGALGain();
void setPGALGain(int8_t gainDb);
int8_t getPGARGain();
void setPGARGain(int8_t gainDb);
int8_t getADCLVolume();
void setADCLVolume(int8_t gainDb2);
int8_t getADCRVolume();
void setADCRVolume(int8_t gainDb2);

void setVFO(unsigned int f);
unsigned int getVFO();

enum Command {
	NONE,
	STATUS,
	HP_UP,
	HP_DOWN,
	DAC_UP,
	DAC_DOWN,
	PGA_UP,
	PGA_DOWN,
	ADC_UP,
	ADC_DOWN,
	VFO_UP1000,
	VFO_DOWN1000,
	VFO_SET,
	BALANCE_UP,
	BALANCE_DOWN,
	PHASE_UP,
	PHASE_DOWN
};

static STM32_HAL_Interface i2c_interface(&hi2c1);
static Si5351 si5351(0x60, &i2c_interface);
// The driver for UART1
static SerialDriver sd1(&huart1);
// Pending command
static volatile Command pendingCommand = Command::NONE;
static int pendingInt = 0;
// Current VFO
unsigned int vfoFreq = 7200000;

// For debouncing the encoder
class Env : public kc1fsz::SystemEnv {
public:
	virtual uint32_t getTimeMs() {
		return HAL_GetTick();
	}
};

static Env env;
static kc1fsz::Debouncer encDebouncer(&env, 10);

void showStatus() {

	// Freq
	printf("PLL Frequencies:\r\n");
	printf("  VFO %d\r\n", getVFO());
	// Gains
	printf("CODEC Gains:\r\n");
	printf("  HPL/HPR: %d/%d, DACL/DACR: %d/%d, PGAL/PGAR: %d/%d, ADCL/ADCR: %d/%d\r\n",
		(int)getHPLGain(),
		(int)getHPRGain(),
		(int)getDACLVolume(),
		(int)getDACRVolume(),
		(int)getPGALGain(),
		(int)getPGARGain(),
		(int)getADCLVolume(),
		(int)getADCRVolume());
	printf("  Balance (x100): %d\r\n", (int)(balance * 100.0));
	printf("  Phase (x100): %d\r\n", (int)(phaseAdjust * 100.0));

	// Sticky flags 1
	writeCodec(0x00, 0x00);
	uint8_t flags = readCodec(0x2c);
	printf("CODEC Status flags:\r\n");

	// Sticky flags 2
	flags = readCodec(0x2c);

	printf("  DACL Overflow   : ");
	if (flags & 0b10000000) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");
	printf("  DACR Overflow   : ");
	if (flags & 0b01000000) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");

	printf("  ADCL Overflow   : ");
	if (flags & 0b00001000) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");
	printf("  ADCR Overflow   : ");
	if (flags & 0b00000100) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");

	printf("  HPL Over Current: ");
	if (flags & 0b10000000) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");
	printf("  HPR Over Current: ");
	if (flags & 0b01000000) {
		printf("Detected");
	} else {
		printf("Clear");
	}
	printf("\r\n");
}

class TestHandler : public MessageHandler {
public:
	void onMessage(const char* m) {

		sd1.send("RX[");
		sd1.send(m);
		sd1.send("]\r\n");

		// Commands
		if (m[0] == 'q') {
			pendingCommand = Command::STATUS;
		}
		else if (m[0] == 'a') {
			pendingCommand = Command::HP_UP;
		}
		else if (m[0] == 'z') {
			pendingCommand = Command::HP_DOWN;
		}
		else if (m[0] == 's') {
			pendingCommand = Command::DAC_UP;
		}
		else if (m[0] == 'x') {
			pendingCommand = Command::DAC_DOWN;
		}
		else if (m[0] == 'd') {
			pendingCommand = Command::PGA_UP;
		}
		else if (m[0] == 'c') {
			pendingCommand = Command::PGA_DOWN;
		}
		else if (m[0] == 'f') {
			pendingCommand = Command::ADC_UP;
		}
		else if (m[0] == 'v') {
			pendingCommand = Command::ADC_DOWN;
		}
		else if (m[0] == 'g') {
			pendingCommand = Command::VFO_UP1000;
		}
		else if (m[0] == 'b') {
			pendingCommand = Command::VFO_DOWN1000;
		}
		else if (m[0] == 'h') {
			pendingCommand = Command::BALANCE_UP;
		}
		else if (m[0] == 'n') {
			pendingCommand = Command::BALANCE_DOWN;
		}
		else if (m[0] == 'j') {
			pendingCommand = Command::PHASE_UP;
		}
		else if (m[0] == 'm') {
			pendingCommand = Command::PHASE_DOWN;
		}
		else if (m[0] == 'w') {
			pendingCommand = Command::VFO_SET;
			pendingInt = atoi(m + 2);
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

		// Setup the serial driver
		sd1.setMessageHandler(&handler);
		sd1.setEOM("\r");
		sd1.start();
		sd1.send("KC1FSZ SDR Console V1.0\r\n");

		bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
		if (i2c_found) {
			setVFO(7200000);
		} else {
		  printf("Si5351 initialization failed\r\n");
		}

		// Band select 0/0
		HAL_GPIO_WritePin(GPIO_0_GPIO_Port, GPIO_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, GPIO_PIN_RESET);
	}

	int8_t inc(int8_t v, int8_t max) {
		if (v < max) {
			return v + 1;
		} else {
			return v;
		}
	}

	int8_t dec(int8_t v, int8_t min) {
		if (v > min) {
			return v - 1;
		} else {
			return v;
		}
	}

	void CppMain_loop() {

		__disable_irq();

		Command c = pendingCommand;
		pendingCommand = Command::NONE;

		if (c == Command::HP_UP) {
			int8_t v = inc(getHPLGain(), 29);
			setHPLGain(v);
			setHPRGain(v);
		}
		else if (c == Command::HP_DOWN) {
			int8_t v = dec(getHPLGain(), -6);
			setHPLGain(v);
			setHPRGain(v);
		}
		else if (c == Command::DAC_UP) {
			int8_t v = inc(getDACLVolume(), 48);
			setDACLVolume(v);
			setDACRVolume(v);
		}
		else if (c == Command::DAC_DOWN) {
			int8_t v = dec(getDACLVolume(), -127);
			setDACLVolume(v);
			setDACRVolume(v);
		}
		else if (c == Command::PGA_UP) {
			int8_t v = inc(getPGALGain(), 95);
			setPGALGain(v);
			setPGARGain(v);
		}
		else if (c == Command::PGA_DOWN) {
			int8_t v = dec(getPGALGain(), 0);
			setPGALGain(v);
			setPGARGain(v);
		}
		else if (c == Command::ADC_UP) {
			int8_t v = inc(getADCLVolume(), 40);
			setADCLVolume(v);
			setADCRVolume(v);
		}
		else if (c == Command::ADC_DOWN) {
			int8_t v = dec(getADCLVolume(), -24);
			setADCLVolume(v);
			setADCRVolume(v);
		}
		else if (c == Command::VFO_UP1000) {
			setVFO(getVFO() + 1000);
		}
		else if (c == Command::VFO_DOWN1000) {
			setVFO(getVFO() - 1000);
		}
		else if (c == Command::VFO_SET) {
			setVFO(pendingInt);
		}
		else if (c == Command::BALANCE_UP) {
			balance += 0.01;
		}
		else if (c == Command::BALANCE_DOWN) {
			balance -= 0.01;
		}
		else if (c == Command::PHASE_UP) {
			phaseAdjust += 0.01;
		}
		else if (c == Command::PHASE_DOWN) {
			phaseAdjust -= 0.01;
		}

		if (c != Command::NONE) {
			showStatus();
		}

		__enable_irq();

	}

	int lastA = 0;
	int lastB = 0;
	long lastSample = 0;

	void CppMain_enc(int a, int b) {

		if (HAL_GetTick() - lastSample > 5) {
			lastSample = HAL_GetTick();
			// Look for the transition
			//encDebouncer.loadSample(true);
			//if (encDebouncer.isEdge()) {
			if (a != lastA) {
				if (b != a) {
					pendingCommand = Command::VFO_UP1000;
				} else {
					pendingCommand = Command::VFO_DOWN1000;

				}
			}

			lastA = a;
			lastB = b;
		}
	}
}

void setVFO(unsigned int f) {
	vfoFreq = f;
	// NOTE: This is quadrature so we multiply by 4
	si5351.set_freq((unsigned long long)vfoFreq * 400ULL, SI5351_CLK0);
}

unsigned int getVFO() {
	return vfoFreq;
}

// ----- HPL/HPR Gain ----------------------------------------------------------
//
// Valid values: -6dB -> 29dB
//
int8_t getHPLGain() {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x10) & 0b00111111;
	// Sign extension
	if (v & 0b00100000) {
		v |= 0b111000000;
	}
	return v;
}

void setHPLGain(int8_t gainDb) {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x10);
	uint8_t newv = (v & 0b11000000) | (gainDb & 0b00111111);
	// Write back the existing value plus the gain
	writeCodec(0x10, newv);
}

int8_t getHPRGain() {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x11) & 0b00111111;
	// Sign extension
	if (v & 0b00100000) {
		v |= 0b111000000;
	}
	return v;
}

void setHPRGain(int8_t gainDb) {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x11) & 0b11000000;
	uint8_t newv = (v & 0b11000000) | (gainDb & 0b00111111);
	// Write back the existing value plus the gain
	writeCodec(0x11, newv);
}

// ----- DAC Digital Volume ---------------------------------------------------
//
// Valid range: +24dB -> -63.5dB in 0.5dB increments.
// So the gain will be multiplied by two.

int8_t getDACLVolume() {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	return readCodec(0x41);
}

void setDACLVolume(int8_t gainDb2) {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	writeCodec(0x41, gainDb2);
}

int8_t getDACRVolume() {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	return readCodec(0x42);
}

void setDACRVolume(int8_t gainDb2) {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	writeCodec(0x42, gainDb2);
}

// ----- PGAL/PGAR Gain ----------------------------------------------------------
//
// Valid values: 0dB -> 47.5dB in 0.5db Increments
// The gain will be multiplied by two
//
int8_t getPGALGain() {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x3b) & 0b01111111;
	// Sign extension
	if (v &  0b01000000) {
		v |= 0b11000000;
	}
	return v;
}

void setPGALGain(int8_t gainDb) {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x3b) & 0b10000000;
	// Write back the existing value plus the gain
	writeCodec(0x3b, v | (gainDb & 0b01111111));
}

int8_t getPGARGain() {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x3c) & 0b01111111;
	// Sign extension
	if (v &  0b01000000) {
		v |= 0b11000000;
	}
	return v;
}

void setPGARGain(int8_t gainDb) {
	// Read the existing value of the register
	writeCodec(0x00, 0x01);
	uint8_t v = readCodec(0x3c) & 0b10000000;
	// Write back the existing value plus the gain
	writeCodec(0x3c, v | (gainDb & 0b01111111));
}

// ----- ADC Digital Volume ---------------------------------------------------
//
// Valid range: -12dB -> +20dB in 0.5dB steps
// So gain will be multiplied by two

int8_t getADCLVolume() {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	return readCodec(0x53);
}

void setADCLVolume(int8_t gainDb2) {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	writeCodec(0x53, gainDb2);
}

int8_t getADCRVolume() {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	return readCodec(0x54);
}

void setADCRVolume(int8_t gainDb2) {
	// Read the existing value of the register
	writeCodec(0x00, 0x00);
	writeCodec(0x54, gainDb2);
}

