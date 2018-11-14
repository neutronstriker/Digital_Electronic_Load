
/*
 * Readme.txt
 *
 * Created: 28-05-2016 18:02:26
 *  Author: neutron
 */ 
 I have again done the same mistake I did on PPS, I have enabled WDT in code and now I am unable to upload code because
 bootloader doesn't know about WDT. SO I have to connect USBASP and flash the nano board with the bootloader file atmega328_WD_MODS_16MHZ_57600.hex present
 in AVR Bootloader folder.

 Only then I will be able to upload code.

 I should also change the bootloaders of other nanos on the same day.

 The above thing is done on DT28-05-2016


 Dt29-05-2016:
 I faced recently a lot a device hanging issues these started earlier also but since I didnot do any extensive testing It was not being
 able to surface. So I found out that this was rarely happening because the knob button and its multiplier changing code.

 Since I had changed the wdt period to 4s instead of 2S. I figured out this was happening because of I2C bus. So I used
 different pull up resistors, As I thought 10k may be too high pull up and as a result the bus capacitance was not meeting
 the required rise time and fall time at 400khz. So I enabled internal pullups still the problem was there. So I reduced 
 the resistors to 2.2k still after sometime the problem surfaced again. 
 
 Finally I found out by checking is that INA226 was creating the problem. Because SSD1306 OLED driver has I2C internal Clamping for
 3.3V, but INA226 and TPL0102 was powered by 5V, but TPL0102 was written on event driven basis however INA226 and SSD1306 was accessed
 on loop.

 So as soon as SSD1306 was enabled I2C bus voltage was not stable it was around 3.8Vrms but after using 2.2k resistors it was 2.58Vrms
 So what I did was that I changed the supply voltage for INA226 to 3.3V it worked for sometime but again the problem started and the
 final bug was discovered which was wiring and length matching issue.  Well not exactly length matching but quite like it as Only for INA226
 the I2C SDA and SCL lines for not comming from a single point and wire lengths were not same. One wire was coming from A4, A5 pins and other
 was comming from OLED I2C pins. So one of them getting delayed.

 So I removed the I2C line which was coming from OLED i2C pin and connected from source forming a proper star configuration.

 So we should always make sure either it is a multi drop or star but make sure the I2C bus line lengths for both DATA and CLK for any device
is always same.

 So After this problem was solved so  I tried to put the supply voltage back to 5V but still there was a problem.
 So i Put it back to 3.3V.

 Now it is working but sometimes still it hangs and watchdog resets the device after 4S.

 Also the clock of I2C bus has been reduced to 100khz in order to cope up with these bus voltage and rise time issues.

 I also did the experiment of remove the gnd connection to load and then weird things started happening, even when we were supplying
 around 1mV only it was sourcing around 160mA on 5V. And when current was set went even mad.

 So I connected the ground back. 

 I also did the reverse connection test when the device was off and PPS was set to 2.5V, PPS went into OVERLOAD. Even when ELOAD was online
 PPS said OVERLOAD at 10mA set on ELOAD and Voltage on PPS 1.98V.

 I checked the bias current issue, and found that the current from OPAMP to MMBT3904 was negligibly small until saturation was hit.
 After saturation was hit it increased to a max 42mA as shown in Display by INA226 but Multimeter showed 46.5mA and also 
 I found that this means we need to calibrate either one of PPS or ELOAD. So there is a calibration issue between PPS and ELOAD.
 The extra current shown in ELOAD is not because of OPAMP output unless there in noload supply or It is saturated.


 Date:03-April-2017:
 Changed the INA226 sensor address to 0x41 in init_platform() because I am using external sensor breakout board which is programmed to do that.

 Date:03-June-2017:
 INA226 based Hang issue fixed now, it seems SCL to ina226 was not soldered properly also I changed the supply voltage of INA226 to 5V (Arduino
 NANO 5V OUTPUT) instead of 3.3V. Also enable the debounce delay in ISR0 which was disabled previously.


 Dt:29-10-2018:
 Added Serial command capability for setting current and turning on and off the load.
 This was done in a hurry for a specific task, so we will need to clean it up sometime when possible.

 Dt:14-11-2018:
 Added temp?, tset and help commands
 which will get the LM35 temperature value, set the safe operating temperature threshold and display commands usage