#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI/twi.h"
#include "serialF0/serialF0.h"
#include <util/delay.h>
#include "IMU/LSM6DSMTR.h"

#define TWI_ADRESS 0b1101011   // The adress of the I2C device

#define F_CPU 2000000UL

int main(void){

  PORTE.DIRSET = PIN0_bm | PIN1_bm;
  
  /*
   *  TWIx is the TWI module of the Xmega device you want to use.
   *  BAUD_X00K is the speed of the I2C/TWI communication. You can use BAUD_100K and BAUD_400K.
   *  You can find out which one you need from the datasheet of the device you want to communicate with.
   *  For single master operations you can leave TIMEOUT_DIS as is. Other options are: TIMEOUT_50US, 
   *                                                                 TIMEOUT_100US and TIMEOUT_200US.
   */
  PMIC.CTRL     |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;		// set low and medium level interrupts
	sei();					//Enable interrupts
	// init stream of serial using CPU frequency
	init_stream(F_CPU);

  enable_TWI(&TWIE, BAUD_400K, TIMEOUT_DIS);
  
  // uint8_t write = x // x is the data that is going to be written to a register of the I2C device
  // uint8_t read;     // The data read from the I2C device register is going to be stored in this variable 
  uint8_t high_x;
  uint8_t low_x;
  uint8_t high_y;
  uint8_t low_y;
  int16_t result_x;
  int16_t result_y;

  uint8_t write = 0x05 << 4;
  write_8bit_register_TWI(&TWIE, TWI_ADRESS, write, CTRL1_XL);

  while(1){
    // write_8bit_register_TWI(&TWIx, TWI_ADRESS, write, REG1);
    // uint8_t error = read_8bit_register_TWI(&TWIE, TWI_ADRESS, &read, REG2);
    // printf("who am i reg is: %d\n", read);
    // printf("Error value is: %d\n\n", error);

    read_8bit_register_TWI(&TWIE, TWI_ADRESS, &low_y, OUTY_L_XL);
    read_8bit_register_TWI(&TWIE, TWI_ADRESS, &high_y, OUTY_H_XL);
    read_8bit_register_TWI(&TWIE, TWI_ADRESS, &low_x, OUTX_L_XL);
    read_8bit_register_TWI(&TWIE, TWI_ADRESS, &high_x, OUTX_H_XL);

    result_x = (high_x << 8) | low_x;
    result_y = (high_y << 8) | low_y;

    printf("%d %d\n", result_y, result_x);
    // printf("Error value is: %d\n\n", error);
  }
}