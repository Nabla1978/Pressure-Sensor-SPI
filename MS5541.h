#ifndef MS5541_H
#define MS5541_H

#include <stdint.h>
#include <Print.h>

class MS5541 : public Print {
public:
	// Constructor for harware SPI
	MS5541(uint8_t dc_pin, uint8_t reset_pin, uint8_t cs_pin, uint8_t hardware_spi = 1);

	// Constructor for software SPI.
	MS5541(uint8_t dc_pin, uint8_t reset_pin, uint8_t cs_pin, uint8_t sdin_pin, uint8_t sclk_pin);

	// Call this first
	void begin(void);

	// Clear lcd without changing location
	void clear(void);

	// Change current position to (character) column and row
	void setCursor(uint8_t column, uint8_t row);

	// Change current location to 0 <= row <= 5,
	// 0 <= pixel_column <= 83
	void gotoRc(uint8_t row, uint8_t pixel_column);

	// Send data to lcd. Will draw data as one pixel wide, 8 pixel high.
	// LSB up.
	void data(uint8_t data);

	// Small numbers. 0<= num <=9 for number and num = 10 for decimal
	// point. Optional parameter shift will move the numbers up/down.
	// shift shold be 0,1,2,3 for the digit to be visible.
	void smallNum(uint8_t num, uint8_t shift = 1);

	void clearRestOfLine(void);
	void bitmap(uint8_t *data, uint8_t rows, uint8_t columns);

private:
	void send(uint8_t dc, uint8_t data);
	void command(uint8_t data);
	virtual void write(uint8_t ch);
	void inc_row_column(void);
	uint8_t dc;
	uint8_t cs;
	uint8_t reset;
	uint8_t hardware_spi_num;
	uint8_t sdin;
	uint8_t sclk;
	uint8_t current_row, current_column;
};

#define MS5541_LINES 6
#define MS5541_COLS  14
#define MS5541_WIDTH  84
#define MS5541_HEIGHT 48


#endif
