#include "config.h"

#include "Arduino.h"
//#define ARDUINO 101 //this is defined in project properties
//#define F_CPU 16000000UL //it is defined in Project properties

#include "../../uartlib.h"
#include <util/delay.h>

#include "i2c.h"

#include "GPIO.h"

#include "myStack.h"

#include "Adafruit_SSD1306.h"

#include "fifo.h"

#include "INA226.h"
//#define INA226_ADDRESS 0x40

#define OLED_ADD 0x3C

#define F_STR(x) (__FlashStringHelper*)PSTR(x) //to be used only for Arduino print class based functions

Adafruit_SSD1306 lcd;
INA226 iSensor;

void loop();
void initPlatform();
void i2c_internal_pullup(bool state);		//later this should be included in i2c_lib
void showPinOut();

void anti();
void clockwise();


GPIO blueLed(4);
GPIO redLed(13);
GPIO knob_clk(9);
GPIO knob_dat(7);
GPIO knob_btn(3);//knob_btn(4);

uint16_t pc0=0;
uint16_t pc2=0;
uint8_t state=0,data;

fifo_byte buffer(16);

#define CLOCK		2
#define ANTICLOCK	3

#define POT_CHANNEL_A	0
#define POT_CHANNEL_B	1

#define COARSE_ADC_CH		A0
#define COARSE_BUF_ADC_CH	A1
#define FINE_ADC_CH			A2
#define V_TO_I_ADC_CH		A6
#define TEMP_ADC_CH			A3

uint32_t btn_sw_interval=0;
#define BUTTON_DEBOUNCE_DELAY	200 //200 Milli Seconds 
#define DISPLAY_UPDATE_PERIOD	50	//30 Milli Seconds

#define ANALOG_REF_VOLTAGE_mV		5000			//5000 for external power and 4320 for USB.
#define ANALOG_2_VOLT_MULTIPLIER	((float)ANALOG_REF_VOLTAGE_mV/1023)		//but I doubt this would be in float, which would introduce more error.
																			//now solved by type casting.		
																			//by making the analog ref variable we may not need to have different
																			//function arrangements while changing AVref source.

#define ANALOG_REF_VOLTAGE_INTERNAL_mV	1100
#define ANALOG_2_VOLT_MULTIPLIER_INTERNAL	((float)ANALOG_REF_VOLTAGE_INTERNAL_mV/1023)

#define RESISTOR_VALUE			(float)1.17				//1.12 OHMS

uint8_t pot_channel = POT_CHANNEL_A;

void pot_decrease(uint8_t channel);
void pot_increase(uint8_t channel);

uint8_t pot_channel_val[2]={0};
char channel_code[2]={'A','B'};

#define TPL0102_ADDRESS	0x50		//When all Address pins grounded

uint16_t conv_2_volts(uint8_t adc_channel, uint16_t vref_in_millivolts);
void displayUpdate();
uint32_t display_update_interval=0;

void resetPots();

float getSupplyVoltage();
float supplyVoltage=5000;		//in milli-Volts, default value is 5000mV

int setLoadCurrent(uint16_t current);
uint16_t iValueSetting=0;
void increase();
void decrease();
uint8_t temperature;

GPIO output_sw(2);
uint8_t loadON=0;

#define	ABSOLUTE_MAX_TEMP	90
#define	WARNING_TEMP		60

uint8_t int1_count=0;
uint16_t knob_click_value=1;		//default click value is 1 per knob click in any direction.

#include <avr/wdt.h>

#define MAX_SUPPORTED_CURRENT 2000		//in mA

//there are two errors because of which my reading were not matching, 
//1.The Voltage ref of VCC is not constant 5000mV as a result the multiplication factor 4.88 to get voltage is not proper
//2.The 1 OHM resistor is not exactly 1 OHM it is around 1.12 OHMs 

//I found that voltage measure on Nano when connected to my computer was 4.12Volts, could be because of IR drop of USB cable
//also maybe some diodes in series on the board and Current was around 100mA at peak.

//Also I didn't find any loading when I tested second channel at 50% and read value was same as calculated value, se we can
//say that there was no loading, atleast not with LM358.

//There are some more comments about the test results which you can find in the schematic called "current controller"(multisim).

//Another thing to notice was that the peltier junction device when connected in series didn't let the current more than 630mA@5V
//because it might have heated up as a result its resistance increased and capped the current draw capacity.

//My current capacity was earlier capped at 1.3A@5V, So I checked in multisim simulation that the opamp needs to provide around 40mA
//to able to draw around 3A@5V which I think technically is not possible because LM358 is rated for around 20mA otherwise output will be 
//start drooping. So I thought by increasing the gain of Transistor I could reduce the current draw so I planned to use BC549 as 
//darlington input stage for 2N3055, but it still didn't work, I thought of another idea to use TWO opamps with outputs coupled
//to be able to drive the 40mA current. But now I just checked that the voltage drop because of my test cable itself was more than
//2.5V So it was obvious that current couldn't be increased. I had also planned to increase the OPAMPS supply voltage at a point
//so that its output voltage should not get affected by its RAIL limit.

//Even when the wire was changed I couldn't reach much higher currents so I increased the opamp voltage as a result I was able to
//reach higher current. By increasing the Opamp voltage I was able to increase the current to 2.03A@4.8V. However it should be noted
//that the when I was using even short good wires the internal resistance of the internals of PSU dropped the voltage to 4.5V when
//it was actually showing 5V@1.2A. So even if my voltage was 4.8V it should have been actually 4V or less.
//DT:29-05-2016
//Find updated answers to this in Readme log.


//DT:19-05-2016:
//after assembly need to test if it we can run the device even by disconnecting the trace between GND and LOAD section.
//also need to see the resistor value if needs to be calibrated for current sensor, but there is chance that the small amount of current
//extra shown by the load meter could be because of the basecurrent flowing from the LM358 Opamp output into the MMBT3904 and then to the 2N3055.
//DT:29-05-2016
//Find updated answers to this in Readme log.


//////////////////////////// copied from DDAT LAtest; done in a hurry; fix later //////////////////////
#define CMD_REPLY_BUFFER_LENGTH		64		//initially it was 32, but in order to accommodate i2cwrite instruction, it has been increased to 64.

char btCmdReplyBuffer[CMD_REPLY_BUFFER_LENGTH]={0};
uint8_t uartBufCharCount=0;

//lets try to keep all command keywords within 4 letters, to improve performance by reducing search time,
//however it will be better if we can using strcmp_P and make sure that commands are executed when followed by cr,lf etc. --done
const char PROGMEM cmd1[]="iset";		//set current value
const char PROGMEM cmd2[]="ion";		//turn on load
const char PROGMEM cmd3[]="ioff";		//turn off load
const char PROGMEM cmd4[]="stat?";		//will return status ON or OFF
const char PROGMEM cmd5[]="iset?";		//will return iset value
const char PROGMEM cmd6[]="ibus?";		//will return ishunt value
const char PROGMEM cmd7[]="vbus?";		//will return vbus value

void readSerialCmd();
uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen);
void replyOK();

uint8_t case_sensitivity_status=0;

uint8_t backspace_det=0;

float iShuntVal = 0;
float vBusVal = 0;

////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	wdt_reset();		//reset Watchdog timer
	wdt_disable();
	wdt_enable(WDTO_4S);	//enable Watchdog timer with a Value of 2 seconds, It was earlier affected a lot because of 
	//i2c bus delays because of higher pullup values. Now solved by using enabling software pullup also.
	
	
	initPlatform();		//Initialize all platform devices like UART and Display and other peripherals.
	

	
	while(1)
	{
		readSerialCmd();
		loop();
		if (serialEventRun) serialEventRun();	//Polls the Serial port for new data and updates Buffer accordingly
												// if serialEvent() function is used
	}

	return 0;
}

void initPlatform()
{
	init();	//Initialize Arduino Core
	
	Serial.begin(57600);//increases a lot or Ram and Rom usage disable after diagnostics and recompile.
	
	Serial.print(F_STR("ELECTRONIC_LOAD_INITIALISED\r\n"));
	
	i2c_init(I2C_BUS_CLK);	//it is called with 400Khz by adafruit display lib so not necessary any more, but before display is initialized many other I2C devices are
	//also initialized so we call it here. It is fine though to be set twice not a big deal
	
	i2c_internal_pullup(true);	//If this is not enabled BUS hangs most of the times. So this has enabled. Maybe pullup value is too high
	//so heavy bus capacitance of three devices is not being handled by 10k resistors alone, I should have used 4.7k or less value
	//same change should be done in PPS also.
	//this is not required since pullups are present in LCD board, however
	//if we add more devices to bus we should add external pull-ups.	Find updated answers to this in Readme log.

	#ifdef INA226_ENABLE

		iSensor.begin(INA226_ADDRESS);

		iSensor.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_204US, INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
	
		// Calibrate INA226. Rshunt = 0.04 ohm, Max excepted current = 2A ,Resistor Calibration value has been compensated by calibration with source meter
		iSensor.calibrate(0.0388, 2);
	//DT:29-05-2016
	//Find updated calibration need info to this in Readme log.

	#endif

	knob_clk.setInput();	//set knob pins as input
	knob_dat.setInput();
	knob_btn.setInputPullUp();		//set Knob Button as Input Pullup.

	PCMSK0 = (1<<PCINT1) | (1<<PCINT0);	//set PCI on digital Pin 8 & 9, an change on either pins of encoder triggers IRQ.
	PCICR = (1<<PCIE0);					//Enable the PCI0 IRQ
	
	
	
	supplyVoltage = getSupplyVoltage();
	Serial.print("Supply Voltage is: ");
	Serial.println(supplyVoltage);
	
	
	
	blueLed.Low();
	redLed.Low();
	
	output_sw.setInputPullUp();		//Hardware Pull up resistor Pad got disconnected so software pullup used.
	EICRA = (1<<ISC01);// | (1<<ISC11); //INT0 & INT1 Trigger on Falling edge
	EIFR &= ~(1<<INTF0);// | (1<<INTF1));		//bootloader might have enabled INT0/INT1 So clear the FLAG first otherwise this will set an IRQ @ Power On.
	EIMSK = (1<<INT0);// | (1<<INT1);	//ENABLE INT0 & INT1
	
	resetPots();	//reset Dpot Channel A and B., when it is placed just after i2c_init and i2c Pull up enable
	//i dont know why it makes TPL0102 go mad maybe i2c bus stability issue. but even after issuing delay it did not solve.
	
	
	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_ADD);
	
	lcd.dim(true);//dim the display brightness
	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextColor(WHITE);
	lcd.setTextWrap(true);
	lcd.setTextSize(2);
	lcd.println(F_STR("  NEUTRON"));
	lcd.println();
	lcd.print(F_STR("ELECTRONIC"));
	lcd.println(F_STR("   LOAD"));
	lcd.display();
	
	delay(2000);		//delay to show Boot text
	
	
	

	
	
	

}

void loop()
{
	
	wdt_reset();		//reset WatchDog Timer, If this loop hangs anywhere and gets stuck for more than 2 seconds it will reset the device.
	
	static uint16_t iValLastState=0;
	
	static	uint32_t warningLedTogDelay = 0;
	#define	LED_BLINK_DELAY	400			//400mS Led Blink Delay
	
	//static uint32_t output_sw_debounce_delay=0;
	
	temperature = (ANALOG_2_VOLT_MULTIPLIER_INTERNAL*analogRead(TEMP_ADC_CH))/10;		//temperature logging.
	
	
	//////////////////////////// OUTPUT ENABLE SWITCH LOGIC //////////////////////////////
	if(loadON)
	{
		if (temperature < ABSOLUTE_MAX_TEMP)		//temperature less than 90 degree celcius.
		{
			
			
			if (iValLastState != iValueSetting)
			{
				iValLastState = iValueSetting;
				setLoadCurrent(iValLastState);
			}
			
			
			if ((temperature > WARNING_TEMP) && ((millis()-warningLedTogDelay) > LED_BLINK_DELAY) )
			{
				blueLed.toggle();
				warningLedTogDelay = millis();
			}
			else if (millis()-warningLedTogDelay > LED_BLINK_DELAY)
			{
				blueLed.High();
			}
			
			
		}
		else
		{
			loadON = 0;		//Turn OFF DEVICE When temperature exceeds ABSOLUTE_MAX_TEMP
		}
		
		
	}
	else
	{
		if (iValLastState != 0)
		{
			setLoadCurrent(0);
			//resetPots();
			iValLastState = 0;
		}
		blueLed.Low();
	}
	
	
	/*
	//////////////////////////// OUTPUT ENABLE SWITCH LOGIC //////////////////////////////
	if (!output_sw.getState() && millis()-output_sw_debounce_delay > BUTTON_DEBOUNCE_DELAY)
	{
		loadON = !loadON;			//we will put this part in INTR so that no re-triggering happens(on button hold) and also it will be edge triggered
									//so no de-bouncing required.
		output_sw_debounce_delay = millis();
	}
	*/
	
	
	
	/////////////////////////////////// KNOB MULTIPLIER CODE //////////////////////////////////////////	
	if(!knob_btn.getState() && millis()-btn_sw_interval>BUTTON_DEBOUNCE_DELAY)
	{
		//pot_channel = !pot_channel;		//by switch Press Toggle Between POT Channels.
			int1_count++;
			
			if (int1_count == 4)
			{
				int1_count = 0;
			}
			
			knob_click_value = 1;
			
			for (uint8_t i=0;i<int1_count;i++)
			{
				knob_click_value *= 10;
			}

		btn_sw_interval = millis();
	}
	
	
	


	
	//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////
	
	uint8_t test;
	if(!buffer.isEmpty())
	{
		test = buffer.deque();
		
		if(test == ANTICLOCK)
		{
			
			//pot_decrease(pot_channel);
			//anti();
			decrease();
		}
		else if (test == CLOCK)
		{
			//pot_increase(pot_channel);
			//clockwise();
			increase();
		}
		
	}
	
	displayUpdate();
	
	
}


void displayUpdate()
{
	if(millis()-display_update_interval > DISPLAY_UPDATE_PERIOD)
	{
		lcd.clearDisplay();
		lcd.setTextColor(WHITE);
		lcd.setTextSize(2);
		lcd.setCursor(0,0);
		
		lcd.setTextWrap(false);
		
		/*
		lcd.print("POT-");
		lcd.print(channel_code[pot_channel]);
		lcd.print(':');
		lcd.println(pot_channel_val[pot_channel]);
		*/
		
		uint8_t arr[4]={0};
		uint8_t pos=0;
		uint16_t temp = iValueSetting;
		lcd.print("Iset:");
		
		//lcd.println(((float)iValueSetting/1000),3);
	
		while(temp>0)
		{
			arr[pos]=temp%10;
			temp/=10;
			pos++;
		}
		
		for(uint8_t i=4;i>0;i--)
		{
			if (i-1 == int1_count)
			{
				lcd.setTextColor(BLACK,WHITE);
			} 
			else
			{
				lcd.setTextColor(WHITE);
			}
			lcd.print(arr[i-1]);
			
			if (i==4)
			{
				lcd.setTextColor(WHITE);
				lcd.print('.');
			}
			
		}
		
		lcd.println();
		
		
		lcd.setTextColor(WHITE);
		lcd.print(F_STR("Cur:"));
		
		#ifdef INA226_ENABLE
			iShuntVal = iSensor.readShuntCurrent();
			lcd.print(iShuntVal,3);
		#endif
		
		lcd.print('A');
		
		lcd.setCursor(0,34);
		lcd.print(F_STR("Vin:"));
		
		#ifdef INA226_ENABLE
			vBusVal = iSensor.readBusVoltage();
			lcd.print(iSensor.readBusVoltage(),2);
		#endif
		lcd.print('V');	
		
		/*
		lcd.setCursor(0,50);
		lcd.print(F_STR("Pow:"));
		lcd.print(iSensor.readBusPower(),2);
		lcd.print('W');
		*/
		
		
		analogReference(INTERNAL);
		
		lcd.setCursor(0,50);
		lcd.print(F_STR("Temp:"));
		lcd.print(temperature);
		lcd.print("C");
		
		//here code to read back Analog data and print on LCD.
/*		
		lcd.setTextSize(1);
		lcd.println();
		
		lcd.print("A0:");
		lcd.print(ANALOG_2_VOLT_MULTIPLIER*analogRead(A0));
		lcd.print(" A6:");//A1
		lcd.print(ANALOG_2_VOLT_MULTIPLIER*analogRead(A6));
*/		
		lcd.display();
		
		display_update_interval = millis();			//remember delays can cause problems for knob. so we use millis to do this.
													//or even better try reducing ADC freq, but see that reducing ADC freq doesn't
													//add to the delays.(it won't if it is Interrupt based)
													//Another important thing to note is that on these Arduino Mini, Nano
													//board the ADC is pretty Noisy because they don't have separate Supplies for 
													//AVCC a we have on-our boards which is filtered through a LC filter.
													//Here all VCC lines are coupling together as a result the Switching noise
													//of Supply Line are coupled to it. However there might also be significant
													//contribution because of the Reference taken as VCC.
													
													//So we have to make an intelligent algorithm which will take Reference as
													//VCC until we reach below 1V, then it will switch internal BG Voltage 1.1V
													//for Channel A, and for Channel B, it will Always take Vref-EXT as Reference
													//we have to sufficient delays for Internal Voltages to stabilize after changing
													//reference source.
													
													//but point to be noted is that I don't think it will be of much help of using
													//Vref-EXT connected to WA-POTA since ATMega328 Datasheet Specifies that the VRef
													//-EXT cannot go below 1V.
	}
}




void i2c_internal_pullup(bool state)
{
	GPIO sda(A4);
	GPIO scl(A5);
	
	if(state == true)
	{
		sda.setInputPullUp();
		scl.setInputPullUp();	
	}
	else
	{
		sda.setInputPullDown();
		sda.setInputPullDown();
	}
}

void reset_millis()
{
	
	extern volatile unsigned long timer0_overflow_count;
	extern volatile unsigned long timer0_millis;
	timer0_overflow_count=0;
	timer0_millis=0;
	
}

uint16_t minutes()
{
	return (millis()/1000)/60; //check this that division operation can seriously decrease performance
                            //because there is not hardware division algorithm
                            //try to take minutes and seconds as global values and run only seconds that will update
                           //the minutes value also once in every 60 seconds instead of once on every call.
}

uint8_t seconds()
{
	return (millis()/1000)%60; //check this that modulo operation can seriously decrease performance
}                               //find an optimized way try a mix of >> and subtraction instead


ISR(INT1_vect)
{
	/*
	while(!knob_btn.getState());
	int1_count++;
	
	if (int1_count == 4)
	{
		int1_count = 0;
	}
	
		knob_click_value = 1;
		//this area is very buggy. If you push the button too fast or immediately after pressing this
		//you start turning knob then wdt_reset() can't be executed and system will reset so, we need
		//to find a  solution for this. Also it is not working very well, very unstable.
		//another thing which I have noticed is that the wires below it are by chance getting shorted
		//and re-triggering this IRQ. we should use some insulation below it.
		
		//For better stability we can make this portion of code into polled instead of IRQ driven.
		for (uint8_t i=0;i<int1_count;i++)
		{
			knob_click_value *= 10;
		}
	*/
//	_delay_ms(100);		//noise cancellation delay
}

ISR(INT0_vect)
{
	loadON = !loadON;
	while(!output_sw.getState());
		_delay_ms(5);		//noise cancellation delay, using delay in the interrupt causes the device to hang sometimes
}

ISR(PCINT0_vect)
{
		
	data = PINB & 0x03;
	if(state != data)
	{
		if(data == 0)
		{
			if(state == 1)
			{
				buffer.enque(ANTICLOCK);
			}
		}
		else if (data == 1)
		{
			if (state == 0)
			{
				buffer.enque(CLOCK);
			}
			
		}
		else if (data == 3)
		{
			if (state == 2)
			{
				buffer.enque(ANTICLOCK);
			}
			
		}
		else if (data == 2)
		{
			if (state == 3)
			{
				buffer.enque(CLOCK);
			}
			
		}
		
		//try adding low pass filter to reduce glitch--done
		state = data;		//added a low pass filter but even that couldn't help much.
		
	}
	
	//I understood that when Inside this ISR the delay() doesn't work properly but it works as opposed what I thought previously
	//that it will get stuck but surprising I saw that it was working but maybe it wasn't working as it was supposed but in a 
	//an irregular way, for example the 100mS specified delay might would have taken even longer than 100.
	
	
	//I found it you can read the micros() def which is used by delay() now in that there is a condition which checks if TIFR flag
	//is cleared or up for the current run when TCNT value is less than 255 but since we are inside the ISR the timer_ISR wont
	//execute as a result the flag wont be cleared as a result the variable 'm' is always increment on each call to micros()
	//as a result the delay() ends up sooner that it should be, that is why when I used 100millisecond delay using _delay_ms()
	//it doesn't work that well.
	
	//May be I have not fully understood that yet, it seems that the 'm' value is always reloaded from timer_0_overflow_count
	//which will not increment if timer0ISR doesn't run. So the only way I think to find out the cause of early exit of delay
	//instead of getting stuck is by doing a debug. (For debug we have to write a minimal code, use Timer2 ISR and use delay with 
	//in that). We cannot use any communication polling stuff like I2C or SPI, absolutely minimal code.
	
	//another thing to note is that since now program complexity has slightly grown which eventually slows down thing even without
	//any delay here it works fairly well and stable.
	
	//_delay_ms(5);
	//delay(100);				//now the delay works here. but doesn't improve much still can try the low pass, but you can disable this delay and slowly move the knob to see the bounce effect.
					//you can also experiment by choosing only one pin as INT source instead of both.--done--didn't help.
					//100ms is the most optimal delay, below or above this can make the system unstable.
		
		//even though when moving it very fast it slips, and sometimes shows a few anti-clocks, in real-life implementation
		//we will not count how many anti-clocks and clocks it turned, but if it turned really fast in one direction
		//and go a few glitches of the opposite direction, take it this way like while increasing from 0 to 100 it will
		//increase from 0 to 97 instead because 100 fast turns  + a few glitches in opposite direction
		//but we won't be able to notice it. So it won't matter at all at that speed.
}

void showPinOut()
{
	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextSize(1);
	lcd.println(F_STR("12,11...3,2,G,R,RX,TX"));
	lcd.println(F_STR("13,3.3,AREF,A0-A7"));
	lcd.print(F_STR("           5V,R,G,RAW"));
	lcd.display();
}

void resetPots()
{
	uint8_t data[2]={0,0};
	uint8_t acr = 0xC0;
	i2c_write(TPL0102_ADDRESS,0,data,2);
	i2c_write(TPL0102_ADDRESS,0x10,&acr,1);		//Write 0 into Non_volatile Registers for Channel A and B, then make the access type Volatile
												//so after reset always the value loaded would be initially 0 for both channels even before 
												//any I2C commands is issued to change the value.
												//After Enabling this option TPL0102 read write got mad, I have debug this issue.
												//So currently it has been disabled.
}

uint8_t setVoltageDivider(uint8_t scaling_factor, uint8_t tpl0102_address, uint8_t channel)		//scaling factor is given as a percentage of H_terminal of DPOT.
{
	uint8_t dpot_value;
	
	if (scaling_factor <= 100)		//since it is uint type no point of using a condition of if(scaling_factor >= 0)
	{
		if(scaling_factor == 100)
		{
			dpot_value=255;
		}
		else if (scaling_factor == 0)
		{
			dpot_value=0;
		}
		else
		{
			dpot_value = (scaling_factor*255)/100;		//calculate the dpot register value for the given scaling factor.
		}
	} 
	else
	{
		return 0;		//error: out of range, percentage to be expected with in 0-100.
	}
	
	i2c_write(tpl0102_address,channel,&dpot_value,1);		//channel is 0 for A and 1 for B; we can use Macro for this
															//if we are going to call this function very often then either 
															//we should disable NON-VOLATILE WRITE in ACR register of TPL0102
															//or we should read each time the ACR register to check if a
															//NON-VOLATILE write is in progress before writing.
	return 1;
}


uint16_t conv_2_volts(uint8_t adc_channel, uint16_t vref_in_millivolts)
{
	return (analogRead(adc_channel)*vref_in_millivolts)/1024;	//dividing by 8192, then by 125, same as dividing by 1024 then by 1000
}

float getSupplyVoltage()
{
	float volts;
	volatile uint16_t val;		//setting voltatile so that dummy read is not optimized
	setVoltageDivider(20,TPL0102_ADDRESS,POT_CHANNEL_A);	//set COARSE channel to 20% of Supply.
	analogReference(INTERNAL);
	delay(100);	//stabilize the Analog REF
	val= analogRead(COARSE_ADC_CH);//after changing channel first read gives error(according to old ref), so do a dummy read once
	val=0;
	for (uint8_t i=0;i<3;i++)
	{
		val+=analogRead(COARSE_ADC_CH); 
	}
	
	val/=3;	//taking average.
	
	volts = (float(5*1100UL*val))/1023;	//5 is multiplied here because we took 20% reading so 5 times of it is original.
										//always make sure that in dividend side there is at-least one UL.
	return volts;
}

int setLoadCurrent( uint16_t current )		//in mA
{
	uint8_t pot[2];
	
	if (current > MAX_SUPPORTED_CURRENT)		//max current settable is 2A for now
	{
		return 0;
		Serial.println(F_STR("Error! Max Limit is 2A"));
	}
	
	//here what i need to accomplish is that when the required voltage is calculated below 
	//then we should be able to set the output voltage to that point using some mathematical calculation
	//on the output of  WB (Wiper B) of TPL0102.
	
	//then by using the Rotary encoder i will set the required current which will shown above and the calculation
	//will be done here and values sent to POTs in percentage or values.
	//rotary encoder push button will be used to toggle between current setting digits.
	//enable switch will activate the load by setting the output.
	//on push enable again it will set WB to 0 and disable the load.
	
	uint16_t voltageRequired = RESISTOR_VALUE * current;		//voltageRequired will be in MilliVolts
	
	if (current <= 100)
	{
		pot[0] = 15;											//fix the value here because we only want to change the POT B for small currents.
		pot[1] = (65025*voltageRequired)/(supplyVoltage*pot[0]);	
		
	
	}
	else
	{
		pot[0] = (65025*voltageRequired)/(supplyVoltage*127);		//eq1
		pot[1] = (65025*voltageRequired)/(supplyVoltage*pot[0]);		//eq2
	
	}
	
	

	/*
	
	The transfer function for the voltage required is explained below:
	voltageRequired = (supplyVoltage*pot0Val/255)*(pot1Val/255)
	First the voltage is scaled down by channel 0 and that voltage is further scaled down by Channel 1.
	Thus the equation above was formed.
	So in eq1 we get a value which is as close as possible to required value when pot[1] is assumed to be set at 127 i.e. center
	because in that case we will be able to trim up and down both ways.
	
	Then the value that we have we know might have errors because of fraction loss so we will
	compensate it now by utilizing the channel B value in eq2 by substituting pot[0] value calculated from eq2.
	you can check a document of Microsoft Math in this project folder.

	However for currents smaller than 200mA we will fix the pot[0] to a particular value like 10 first and only calculate pot[1]
	same thing we can do for higher current by fixing pot[1] and only calculate pot[1], but current scenario would server better.
	
	*/
	
	/*
	float percentage_actual = (voltageRequired*100)/supplyVoltage;
	
	uint8_t percentage = percentage_actual;
	uint8_t fraction = (percentage_actual - percentage)*100UL;
	*/
	#ifdef PRINT_DEBUG_MSG
		Serial.print("Current set is :");
		Serial.println(current);
	
		Serial.print("Voltage required is :");
		Serial.println(voltageRequired);
	
		Serial.print("Pot 0 :");
		Serial.println(pot[0]);
	
		Serial.print("Pot 1 :");
		Serial.println(pot[1]);
	
		uint16_t achievedVoltage = ((supplyVoltage*pot[0]/255)*pot[1])/255;
	
		Serial.print("Achieved Voltage :");
		Serial.println(achievedVoltage);
	#endif
	
	i2c_write(TPL0102_ADDRESS,0,pot,2);		//writing to pot for testing.
/*
	analogReference(INTERNAL);
	
	for(uint8_t i=0;i<3;i++)
	{
		Serial.print("COARSE_ADC_CH_V:");
		Serial.println(analogRead(COARSE_ADC_CH)*1.074);
		
		Serial.print("COARSE_ADC_BUF_CH_V:");
		Serial.println(analogRead(COARSE_BUF_ADC_CH)*1.074);
		
		Serial.print("FINE_ADC_CH_V:");
		Serial.println(analogRead(FINE_ADC_CH)*1.074);
		
		Serial.print("V_TO_I_ADC_CH_V:");
		Serial.println(analogRead(V_TO_I_ADC_CH)*1.074);
	}
*/


	//when i changed the RESISTOR_VALUE macro from 1.12 to 1.17 the current shown in the LOAD side meter is more
	//close to set current but it there is small difference between the current that is shown in PPS and Load
	//this could be because of either the fact that a small of current is also flowing from the opamp through
	//the emitter or also because that either one of them is not calibrated properly. But since my PPS was
	//calibrated using a SourceMeter in Tessolve it could the first cause or the Load meter calibration has to be done.
	return 1;
}



void increase()
{
	if(iValueSetting < MAX_SUPPORTED_CURRENT-knob_click_value)
	{
		iValueSetting+=knob_click_value;	
	}
	
}

void decrease()
{
	if (iValueSetting > 0+knob_click_value)
	{
		iValueSetting-=knob_click_value;
	}
	
}


//////////////////////////////////////// copied from DDAT latest in a hurry; fix later/////////////////////////////////////////////////////////
void readSerialCmd()
{
		//char *pos;
		char numeric[7];
		uint8_t token_char = '\r';
		uint8_t num;
		
		/*
			Don't Provide both CR and LF after cmds provide only CR. LF is not honoured and is taken up as a character thus, if anywhere
			whole word match is required that condition will fail.
		*/

#ifdef SERIAL_CR_DETECT
	if(Serial.crStatusDetected())	//experimental feature.
#endif
		if(Serial.available())
		{
			#ifdef PRINT_DEBUG_MSG
				Serial.println(F_STR("data available"));
			#endif
			
				
				uartBufCharCount = readUntilToken(btCmdReplyBuffer,token_char,CMD_REPLY_BUFFER_LENGTH);	
				
				if (case_sensitivity_status)
				{
					strlwr(btCmdReplyBuffer); //convert string to lower case
				}
				
								
				#ifdef PRINT_DEBUG_MSG
					Serial.print("Char Count ");
					Serial.println(uartBufCharCount);
				#endif
				
				if(uartBufCharCount == 0)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Character found");
					#endif
					
					return;				//i.e. the '\r' was not found in string/array
				}
				
				/*
				//Read until return is detected or 12 bytes from serial Fifo to buffer.
				uartBufCharCount = Serial.readBytes(btCmdReplyBuffer,12);
				
				
				
				//terminate the string at the point where we find the token character which is '\r' <CR> in this case.
				uint8_t char_pos;
				for(char_pos=0;char_pos<uartBufCharCount;char_pos++)
				{
					if(btCmdReplyBuffer[char_pos]==token_char)
					{
						btCmdReplyBuffer[char_pos]='\0';		//terminate the string at that point so that unnecessary string processing
						break;									//can be avoided.
					}
					
				}
				
				//when 'char_pos' value is same as total character count that means it never found the character,
				//otherwise a break instruction would have been executed. 
				if(char_pos==uartBufCharCount)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Carriage return detected");
					#endif
					return;				//i.e. the '\r' was not found in string/array
				}
				*/
				
			#ifdef PRINT_DEBUG_MSG
				uint8_t i=0;
				while(i<=uartBufCharCount)
				{
					Serial.print(btCmdReplyBuffer[i], HEX);
					Serial.print(' ');
					i++;
				}
				//serial debug messages here
			#endif // PRINT_DEBUG_MSG			
			
			
			 if(strncmp_P(btCmdReplyBuffer,cmd1,4)==0)
			 {
				 uint16_t iSetVal = 0;
				 strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
				 
				 iSetVal=atoi(numeric);
				 
				 #ifdef PRINT_DEBUG_MSG
					 Serial.print(F_STR("iset val is "));
					 Serial.println(iSetVal);
				 #endif
				 
				 if (iSetVal > MAX_SUPPORTED_CURRENT)
				 {
					 Serial.println(F_STR("ERROR_1"));
					 return;
				 }
				 
				 iValueSetting = iSetVal;
				 
				 replyOK();
			 }
			
			
		
			else if(strcmp_P(btCmdReplyBuffer,cmd2)==0)
			{
						
				loadON = 1;
				
				replyOK();
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd3)==0)
			{
				
				loadON = 0;
				
				replyOK();
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd4)==0)
			{
				
				if (loadON)
				{
					Serial.println(F_STR("ON"));
				}
				else
				{
					Serial.println(F_STR("OFF"));
				}
				
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd5)==0)
			{
				
				Serial.print(F_STR("ISET:"));
				Serial.print(iValueSetting);
				Serial.print(F_STR("mA"));
				
				
			}
			
			#ifdef INA226_ENABLE
				else if(strcmp_P(btCmdReplyBuffer,cmd6)==0)
				{
					
					Serial.print(F_STR("IBUS:"));
					Serial.print(iShuntVal);
					Serial.print(F_STR("mA"));
				}
				
				else if(strcmp_P(btCmdReplyBuffer,cmd7)==0)
				{
					
					Serial.print(F_STR("VBUS:"));
					Serial.print(vBusVal);
					Serial.print(F_STR("V"));
				}
			#endif
			
			else
			{
				Serial.println(F_STR("ERROR(!)"));	
			}
									
		}
		
}


uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen)
{
	/*	The Serial.readBytesUntil() doesn't work that well for me because it does a signed character check, which sometimes
		could be a problem for me because it doesn't work well sometime, especially when I tell it read looking for CR it goes mad
		and returns random characters.
		
		So i decided to do my own version.
	*/
	
	/*
		Another most important thing is that we should not use a mixture of both my uart_function and Serial class
		it creates lot of problems and wastes lot of time to debug, use only one of them in whole program.
		
		One more thing is the Serial.readUntil thing also filled the buffer with the token which was the problem
		some places I have used code which looks to match the whole word in that case it fails.
		
		I used to get only half the codes present in buffer like for hello i got only 6C 6F 0D 00 because 
		the hexbyte used uart_write() internally which creates problem since it uses blocking mode not interrupt
		mode like Serial class and interrupt is already enabled for UDRE so it messes things up. So don't use it
		here anymore.
		
		But the Serial.readBytesUntil is advanced in that it can wait for until we actually send the character 
		to the uart and the max wait period can be set to timeout value. So it can actually let you get all
		the characters into a buffer which we specify until the token is detected, and buffer can be bigger than
		even serialreceive buffer of 64bytes.
		
		But since we are doing parsing only after detecting CR so we don't need this feature, not useful for me
		right now. May be we can use this if required for exclusive SPI and I2C mode to specify data strings.
		
		But my code in this function is slightly faster than Serial.readBytesUntil because it doesn't wait for
		any timeout and does use timedRead();
	*/ 
	
	uint8_t char_count=0;
	
//	buffer[0]=0;	//since we don't know what data is present in that location we clear it. So that we don't have to use
					//any complicated tricks in the while condition.
	
	if (maxlen < 1)
	{
		return 0;
	}
	
	//delay(1000);
	
	while(Serial.available()>0 && char_count < maxlen)
	{
		buffer[char_count]=Serial.read();
		
		if (buffer[char_count]==token)
		{
			buffer[char_count]='\0';		//terminate the string
			return char_count;				//return characters read, doesn't include NULL character.
		}
		
		char_count++;
	}
	
	buffer[char_count]='\0';
	
	return 0;		//returning zero tells that token was not found however we have read whatever characters we could get 
					//from buffer, i.e. emptied the buffer or we just ran out of maxlength specified.
}

void replyOK()
{
	Serial.println("OK");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////