/*
 * config.h
 *
 * Created: 09-03-2016 15:20:54
 *  Author: neutron
 
 I have defined some functions directly in .h files (like UART, DS etc) because of which I am not able to
 include all .h files in config.h, because linker error happens by multiple definition instances.
 
 So the solution is that for such files I should move them to respective .c files and keep only def's in 
 .h file, But this will break compatibility with many other projects, but it won't be a big issue
 during next compile I will have to add the respective .c file or the lib to that project.
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

//#define PRINT_DEBUG_MSG //if defined all serial debug msgs will be printed, however this could 
						//cause problem for software on PC which acquire data via uart by issuing certain
						//cmds, because debug msg can interfere in the flow of output data.

//#define SERIAL_ECHO		//this will enable hardware serial ECHO on console, but will quite weird if console software also has one.

extern unsigned char ate_enabled;		//defined in the main file. cause we cannot initialize a variable in header file.

#define SERIAL_CR_DETECT		//this will enable experimental feature of detecting carriage return in serial driver itself

#define lcd_128x64

#define I2C_BUS_CLK	100000UL		//in HZ

#define INA226_ENABLE			//if sometimes the device goes into bootloop because of i2c bus hang just comment out this code.

#endif /* CONFIG_H_ */