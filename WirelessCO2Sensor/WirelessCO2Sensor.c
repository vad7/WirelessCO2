/*
* WirelessCO2Sensor.c
*
* Created: 01.11.2013 15:09:29
*  Author: Vadim Kulakov, vad7@yahoo.com
*
* ATtiny44A
*/ 
#define F_CPU 8192000UL   // OSCCAL = 0x52 (VCC = 3.3V)
// Fuses: BODLEVEL = 1V8
 
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#define LED1_PORT					PORTA		// Info led
#define LED1						(1<<PORTA1)
#define LED1_ON						LED1_PORT |= LED1
#define LED1_OFF					LED1_PORT &= ~LED1

#define KEY1						(1<<PORTB2) // = IR Receiver out
#define KEY1_PRESSING				!(PINB & KEY1)
#define KEY1_PressingTimeMin		10 // longer than 125ms

#define CO2SensorState				(PINA & (1<<PORTA0))

//#define LDR_ENABLED

#define RF_ON1						DDRA &= ~(1<<PORTA7); PORTA |= (1<<PORTA7)
#define RF_ON2						DDRA |= (1<<PORTA7)
#define RF_OFF1						DDRA &= ~(1<<PORTA7); PORTA &= ~(1<<PORTA7)
#define RF_OFF2						DDRA |= (1<<PORTA7); 

#define ERR_RF_Send					0x10 // Send failure, after short bursts = fan offset (mask 0x0F)
#define ERR_RF_SetAddr				0x20 //	Set addresses failure,
#define ERR_RF_NotResp				0x30 //	RF module not response,
#define ERR_CO2Sensor				0x40 //	CO2 Sensor reading failure

#define MAX_FANS					10 // Maximum fans
volatile uint8_t LED_Warning		= 0; // Flashes, long (mask 0xF0):	0 - all ok, or ERR_*
volatile uint8_t LowLight			= 0;
uint8_t	LowLightThreshold;
volatile uint8_t Timer				= 40; // 5 sec
uint16_t CO2Level					= 0;
#define CO2LevelAverageArrayLength	4
uint16_t CO2LevelAverageArray[CO2LevelAverageArrayLength];
uint8_t CO2LevelAverageCnt			= 255;
int8_t FanSpeedOverride				= 0;
int8_t FanSpeedOverrideArray[MAX_FANS];
int8_t FanSpeedForce[MAX_FANS];
uint8_t KeyOkTimeout				= 0;

#define IRCommands					4
#define IRIsCommandReceived			3 // Flag indicating that IRKey contains received IR command
#define IRKey_Up					1
#define IRKey_Down					2
#define IRKey_Ok					3
#define IRKey_FanOverride			4
uint8_t IRSignalTimer				= 0;
uint8_t IRReceived					= 0; // 0 - waiting new, 1 - receiving first pulse, 2 - receiving, 3 - received
uint8_t IRHead, IRArrayBit, IRReadedByte, IRSignalTimerLast;
uint16_t IRHash;
uint8_t Key1Pressed					= 0;
uint8_t IRKey;
uint8_t SetupIR						= 0; // Setup IR Keys
uint8_t Setup						= 0; // 1 - select setup menu, 2 - set parameter, 3 - fans override
uint8_t SetupItem;						 // setup menu item: 1 - Exit, 2 - Set Low light threshold, 3 - Edit EEPROM, 4 - Set OSCCAL (out: OCR0B), 5 - Fans override - select fan, 6 - fan override
uint8_t SetupType;
uint8_t datapos = 0, data = 0;
uint8_t rf_last_addr = 0;

#define EPROM_OSCCAL				0x00
#define EPROM_LowLightThreshold		0x01 // 0x98, Light ON threshold
#define EPROM_NumberFans			0x02 // 4
#define EPROM_TransmitPeriod		0x03 // 80, *0.125 sec
#define EPROM_LowLightMaxFanSpeed	0x04 // 5
#define EPROM_ShowCO2SensorError	0x05 // 1
#define EPROM_FanSpeedDelta			0x06 // 5, Delta for setting lower speed
#define EPROM_RF_Channel			0x07 // 120
#define EPROM_FanSpeedThreshold		0x10 // byte array (CO2 ppm divided by 16): 0x1F, 0x22, 0x25, 0x32, 0x38, 0x44
#define FanSpeedMax			6	 // Length EPROM_FanSpeedThreshold	in bytes = Max speed
// Fan speed: CO2 < 500 ppm = 0; < 550 ppm = 1; < 600 ppm = 2; < 800 ppm = 3; < 900 ppm = 4; < 1100 ppm = 5; > = 6 (max)
#define EPROM_RFAddresses			0x20 // LSB for every fan: 0xC1, 0xC2, 0xC3, 0xC4
#define EPROM_IRCommandHead			0x30 // length of head pulse
#define EPROM_IRCommandArray		0x31 // array of commands
#define EPROM_IRCommandArrayEnd		EPROM_IRCommandArray + IRCommands * sizeof(IRHash)
#define EPROM_FanSpeedChangeArray	0x40 // Speed skip array: <fan num 0>,<RF channel (0xFF current)>,<numbers of speed values>,<speed for change>,<new speed>, <fan num 1>,..., <0xFF> for end of array
#define EPROM_Length				0xFF

typedef struct {
   uint16_t CO2level;
   uint8_t FanSpeed; 
   uint8_t Flags; // // Mask: 0x80 - Setup command, 0x01 - Lowlight
} __attribute__ ((packed)) send_data;

#if(1)
void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); //wdt_reset();
}
void Delay1ms(uint8_t ms) {
	while(ms-- > 0) { 
		_delay_ms(1); //wdt_reset(); 
	}
}
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) { 
		_delay_ms(100); //wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void FlashNumber(uint8_t Number)
{ // HEX
	FlashLED(Number / 16, 5, 15);
	Delay100ms(20);
	FlashLED(Number % 16, 5, 5);
	Delay100ms(20);
}

uint8_t EEPROM_read(uint8_t ucAddress) // ATtiny24A/44A only!
{
	while(EECR & (1<<EEPE)) ; // EEWE
	EEAR = ucAddress;
	EECR |= (1<<EERE);
	return EEDR;
}
void EEPROM_write(uint8_t ucAddress, uint8_t ucData) // ATtiny24A/44A only!
{
	while(EECR & (1<<EEPE)) ; // EEWE
	cli();
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEAR = ucAddress;
	EEDR = ucData;
	EECR |= (1<<EEMPE); //(1<<EEMWE);
	EECR |= (1<<EEPE); //(1<<EEWE);
	sei();
}

uint8_t EEPROM_compare_array(uint8_t *array, uint8_t ucAddress, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
		if(*(array + i) != EEPROM_read(ucAddress + i)) return 0;
	return 1;
}

#endif

#include "..\nRF24L01.h"

#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);	//  Watchdog 0.125 s
uint8_t LED_WarningOnCnt = 0, LED_WarningOffCnt = 0, LED_Warning_WorkLong = 0, LED_Warning_WorkShort = 0, LED_Warning_NoRepeat = 0;

ISR(WATCHDOG_vect)
{
	SETUP_WATCHDOG;
	if(Timer) Timer--;
	if(KeyOkTimeout) KeyOkTimeout--;
	// LED_Warning: 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes
	if(LED_WarningOnCnt) {
		LED1_ON;
		LED_WarningOnCnt--;
	} else if(LED_WarningOffCnt) {	
		LED1_OFF;
		LED_WarningOffCnt--;
	} else if(LED_Warning_WorkLong) { // long flashes
		LED_Warning_WorkLong--;
		LED_WarningOnCnt = 10;
		if(LED_Warning_WorkLong == 0) {
			LED_WarningOffCnt = 6;
			goto xSetPause;
		} else LED_WarningOffCnt = 3;
	} else if(LED_Warning_WorkShort) { // short flashes
		LED_Warning_WorkShort--;
		LED_WarningOnCnt = 2;
		LED_WarningOffCnt = 2;
xSetPause:	if(LED_Warning_WorkShort == 0) LED_WarningOffCnt = 20;
	} else if(LED_Warning) {
		LED_Warning_WorkLong = (LED_Warning & 0xF0) >> 4;
		LED_Warning_WorkShort = LED_Warning & 0x0F;
		if(LED_Warning_NoRepeat) {
			LED_Warning_NoRepeat = 0;
			LED_Warning = 0;
		}
	}
}

ISR(ADC_vect) // Light sensor
{
	LowLight = ADCH >= LowLightThreshold;
}

ISR(PCINT0_vect) // Sensor
{
	if(CO2SensorState)
	{ // Hi state
		TCNT1 = 0;
	} else {
		CO2Level = (CO2Level + (TCNT1 / 4 - 4)) / 2; // tick = 0.125 ms, 350 ppm = 177ms, 2000 ppm = 1002ms
	}
}

ISR(TIM1_OVF_vect) // Sensor error show
{
	if(LED_Warning == 0 && Setup == 0) {
		LED_Warning = ERR_CO2Sensor;
		LED_Warning_NoRepeat = 1;
	}
}

#define HASH_UPDATE IRHash = (IRHash << 4) + IRHash + IRReadedByte // hash=hash*17+b (better: hash=hash*33+b)
//#define HASH_UPDATE IRHash = _crc_ccitt_update(IRHash, IRReadedByte)
ISR(TIM0_OVF_vect) // IR
{
	if(++IRSignalTimer == 0) { // overflow - command end
		if(IRReceived != IRIsCommandReceived) {
			if(KEY1_PRESSING) { // key pressed
				if(Key1Pressed != 255) Key1Pressed++;
				IRReceived = 0;
			} else {
				if(IRReceived == 2 && IRHead) {
					IRReceived = IRIsCommandReceived; 
					HASH_UPDATE;
				} else IRReceived = 0;
				if(Key1Pressed < KEY1_PressingTimeMin) Key1Pressed = 0;
			}
		}
	}
}

ISR(PCINT1_vect) // IR
{
	uint8_t bit;
	if(IRReceived == 0) {
		IRReceived = 1; // Start
		IRSignalTimerLast = 0;
		IRArrayBit = 0;
		IRReadedByte = 0;
		IRHash = 5381; // hash init
		//IRHash = 0xFFFF; // crc init
		goto xEnd;
	} else if(IRReceived < IRIsCommandReceived) {
		if(IRReceived == 1) { // Check first pulse width 
			uint8_t head = IRSignalTimer / 8;
			if(head == IRHead) {
				IRReceived = 2;
			} else if(SetupIR) {
				IRHead = head;
				IRReceived = 2;
			} else {
				IRReceived = 0;
			}
		}
		if(IRSignalTimer > IRSignalTimerLast) // bit = a / b > 1.5
			bit = (uint8_t)(IRSignalTimer - IRSignalTimerLast) > IRSignalTimerLast / 2;
		else
			bit = (uint8_t)(IRSignalTimerLast - IRSignalTimer) > IRSignalTimer / 2;
		IRReadedByte = (IRReadedByte << 1) | bit;
		if(++IRArrayBit > 7) {
			IRArrayBit = 0;
			HASH_UPDATE;
			IRReadedByte = 0;
		}
	IRSignalTimerLast = IRSignalTimer;
xEnd:	IRSignalTimer = 0;
	}
}

void Set_LED_Warning(uint8_t d)
{
	if(LED_Warning == 0 && Setup == 0) {
		LED_Warning_NoRepeat = 1;
		LED_Warning = d;
	}
}

void ShowFanSpeedOverride(int8_t speed)
{
	LED_WarningOnCnt = 0;
	LED_WarningOffCnt = 4;  // 0.5sec
	LED_Warning_NoRepeat = 1;
	if(speed >= 0)
		LED_Warning = speed;
	else {
		LED_Warning = -speed + 0x10;
	}
}

void rf_reset_chip(void)
{
	RF_OFF1;
	Delay100ms(20);
	RF_OFF2;
	Delay100ms(10);
	RF_ON1;
	Delay100ms(2);
	RF_ON2;
	Delay100ms(2);
	NRF24_init(EEPROM_read(EPROM_RF_Channel));
	NRF24_SetMode(NRF24_TransmitMode);
	Delay100ms(1);
	rf_last_addr = 0;
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler division factor: 1
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	DDRA = LED1; // Out
	NRF24_DDR |= NRF24_CE | NRF24_CSN | NRF24_SCK | NRF24_MOSI; // Out
	PORTB = (1<<PORTB0) | (1<<PORTB1); // Pullup not used
	// Timer 8 bit		NRF24L01_Buffer	Unknown identifier	Error
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00); // Timer0 prescaller: 8
	OCR0A = 49; // OC0A - Fclk/(prescaller*(1+TOP)) = 20480hz
	OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	// Timer 16 bit
	TCCR1A = (1<<WGM11) | (0<<WGM10);  // Timer1: Fast PWM Top ICR1 (14)
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10); // Timer1: /1024
	ICR1 = 16000; // Max sensor value * 4. Sensor value must be less than 255 * 16 = 4080
	if(EEPROM_read(EPROM_ShowCO2SensorError)) {
		TIMSK1 = (1<<TOIE1); // Timer/Counter1, Overflow Interrupt Enable
	}
	// ADC
#ifdef LDR_ENABLED
	PORTA = (1<<PORTA7); // pull up: Photo resistor LDR1
 	ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
 	ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
 	ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
#endif
	// Pin change
 	GIMSK = (1<<PCIE0) | (1<<PCIE1); // Pin Change Interrupt Enable 0, 1
 	PCMSK0 = (1<<PCINT0); // Pin Change Mask Register 0 - Sensor
 	PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10); // Pin Change Mask Register 0 - Keys
	LowLightThreshold = EEPROM_read(EPROM_LowLightThreshold);
	if(LowLightThreshold == 0xFF) {
		EEPROM_write(EPROM_OSCCAL, OSCCAL);
		EEPROM_write(EPROM_NumberFans, 1);
		EEPROM_write(EPROM_LowLightThreshold, 0x90);
	}
 	OSCCAL = EEPROM_read(EPROM_OSCCAL);
	// Prepare for IR receiving
	GIMSK |= (1<<PCIE1); // Pin Change Interrupt Enable 1
	PCMSK1 |= (1<<PCINT10); // Pin Change Mask Register 0 - Keys
	TIMSK0 |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	IRHead = EEPROM_read(EPROM_IRCommandHead);
	RF_ON1;
	FlashLED(3,3,3);
	RF_ON2;
	Delay100ms(1);
	SETUP_WATCHDOG;
	sei();
	NRF24_init(EEPROM_read(EPROM_RF_Channel)); // After init transmit must be delayed
	for(uint8_t i = 0; i < MAX_FANS; i++) FanSpeedForce[i] = -1; // set auto
	NRF24_SetMode(NRF24_TransmitMode);
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		uint8_t i;
		sleep_cpu();
		IRKey = 0;
		if(IRReceived == IRIsCommandReceived) {
			for(i = 0; i < IRCommands; i++) {
				if(EEPROM_read(EPROM_IRCommandArray + i * sizeof(IRHash)) == (uint8_t) (IRHash % 256)
					&& EEPROM_read(EPROM_IRCommandArray + i * sizeof(IRHash) + 1) == (uint8_t) (IRHash / 256)) IRKey = i + 1;
			}
			IRReceived = 0;
		}
		switch(IRKey) {
			case IRKey_Up: {
				LED_Warning_WorkLong = 0;
				LED_Warning_WorkShort = 0;
				if(Setup == 0) {
					if(FanSpeedOverride < FanSpeedMax) {
						FanSpeedOverride++;
						ShowFanSpeedOverride(FanSpeedOverride);
						Timer = 0;
					}
				} else {
					LED_Warning_WorkLong = 0;
					LED_Warning_WorkShort = 0;
					Timer = 255;
					if(Setup == 1) {
						if(++SetupItem > 5) SetupItem = 5; // Max menu item reached
						LED_Warning = SetupItem;
					} else {
						if(SetupItem == 3) { // Modify EEPROM
							if(Setup == 2) LED_Warning = ++datapos; else LED_Warning = ++data;
						} else if(SetupItem == 4) { // fans override
							if(Setup == 2) { // select fan
								if(datapos <= EEPROM_read(EPROM_NumberFans)) datapos++;
								LED_Warning = datapos;
							} else { // fan override
								if(SetupType) { // Forced speed
									if(FanSpeedForce[datapos - 1] < FanSpeedMax) {
										ShowFanSpeedOverride(++FanSpeedForce[datapos - 1]);
									}
								} else if(FanSpeedOverrideArray[datapos - 1] < FanSpeedMax) { // set delta
									ShowFanSpeedOverride(++FanSpeedOverrideArray[datapos - 1]);
								}
								goto xSendNow;
							}
						}
					}
				}
				break;
			} case IRKey_Down: {
				if(Setup == 0) {
					if(FanSpeedOverride > -FanSpeedMax) {
						FanSpeedOverride--;
						ShowFanSpeedOverride(FanSpeedOverride);
						Timer = 0;
					}
				} else {
					LED_Warning_WorkLong = 0;
					LED_Warning_WorkShort = 0;
					Timer = 255;
					if(Setup == 1) {
						if(--SetupItem == 0) SetupItem = 1; // Min menu item reached
						LED_Warning = SetupItem;
					} else {
						if(SetupItem == 3) { // Modify EEPROM
							if(Setup == 2) LED_Warning = --datapos; else LED_Warning = --data;
						} else if(SetupItem == 4) { // fans override - select fan
							if(Setup == 2) { // select fan
								if(datapos > 1) datapos--;
								LED_Warning = datapos;
							} else { // fan override
								if(SetupType) { // Forced speed
									if(FanSpeedForce[datapos - 1] > -1) {
										ShowFanSpeedOverride(--FanSpeedForce[datapos - 1]);
									}
								} else if(FanSpeedOverrideArray[datapos - 1] > -FanSpeedMax) { // set delta
									ShowFanSpeedOverride(--FanSpeedOverrideArray[datapos - 1]);
								}
								goto xSendNow;
							}
						}
					}
				}
				break;
			} case IRKey_Ok: {
				if(Setup) {
					FlashLED(1,1,1);
					Timer = 255;
					switch(SetupItem) {
					case 1: // Exit setup
						goto xSetupExit;
					case 2: // Set Low light
						if(Setup == 1)
							LED_Warning = 0;
						else {
							Delay100ms(50);
							sleep_cpu();
							EEPROM_write(EPROM_LowLightThreshold, LowLightThreshold = ADCH);
							goto xSetupExit;
						}
						break;
					case 3: // Edit EEPROM
						LED_Warning = 0; LED_Warning_WorkLong = 0; LED_Warning_WorkShort = 0;
						if(Setup == 1) {
							LED_Warning = datapos = 1;
						} else if(Setup == 2) {
							LED_Warning = data = EEPROM_read(datapos);
						} else if(data != EEPROM_read(datapos)) {
							EEPROM_write(datapos, data);
							LED_Warning = ++datapos;
						}
						break;
					case 4: // Fans override: fan selected
						if(Setup == 2) { // fan selected
							LED_Warning = 0;
							ShowFanSpeedOverride(FanSpeedOverrideArray[datapos - 1]);
						} else { // delta entered
							LED_Warning = datapos;
						}
						break;
					case 5: // Show CO2 Level
						LED_Warning_WorkLong = 0;
						LED_Warning_WorkShort = 8;
						LED_Warning = CO2Level / 256;
						LED_Warning_NoRepeat = 1;
						do { 
							__asm__ volatile ("" ::: "memory"); // Need memory barrier
							sleep_cpu(); 
						} while(Timer && (LED_Warning || LED_WarningOffCnt || LED_Warning_WorkLong || LED_Warning_WorkShort));
						LED_Warning = CO2Level % 256;
						LED_Warning_NoRepeat = 1;
						Setup = 0;
						break;
					}
					if(Setup < 3)
						Setup++;
					else {
						Setup--;
						FlashLED(5, 1, 1);
					}
				} else {
					if(KeyOkTimeout == 0)
						KeyOkTimeout = 5; // 0.6 sec wait for the next Ok pressing
					else { // Ok was pressed 2 times within 0.5 sec
						KeyOkTimeout = 0;
						FlashLED(10, 1, 1);
						Setup = 1;
						LED_Warning = SetupItem = 1; // setup item
						Timer = 255; // ~32 sec
					}
				}
				break;
			} case IRKey_FanOverride: {
				if(Setup == 0) {
					FlashLED(3, 1, 1);
					Setup = 2;
					SetupItem = 4;
					SetupType = 0;
					LED_Warning = datapos = 1;
					Timer = 255; // ~32 sec
				} else if(SetupItem == 3) { // EEPROM edit
					if(Setup == 2) LED_Warning = (datapos += 0x10); else LED_Warning = (data += 10); // add 0x10 to pos, 10 to byte
				} else if(SetupItem == 4) { // Override menu item
					SetupType ^= 1; // switch mode Fan Override / Fan Force
					FlashLED(SetupType ? 6 : 3, 1, 1);
					LED_Warning = 0;
					Delay100ms(5);
					ShowFanSpeedOverride(SetupType ? FanSpeedForce[datapos - 1] : FanSpeedOverrideArray[datapos - 1]);
					Timer = 255; // ~32 sec
				} else {
xSetupExit:			Setup = 0;
					SetupItem = 0;
					LED_Warning = 0;
					Timer = 0;
				}
				break;
			}
		}
		if(KeyOkTimeout == 1) { // Reseting single time Ok pressed
			FanSpeedOverride = 0;
			FlashLED(5, 1, 1);
		}
		if(Key1Pressed == 255) // Press and hold key1 for ~3.2 sec
		{	// Setup IR Commands (2 (after 3.2s) and 7 short flashed will indicate it):
			FlashLED(2, 1, 1);
			Key1Pressed = 0;
			do { Delay100ms(1); } while(KEY1_PRESSING);
			if(Key1Pressed == 255) { // + ~3.2 sec
				// Set OSCCAL. Freq out 20480 Hz -> OC0B (pin 6).
				OCR0B = (OCR0A + 1) / 2 - 1; // Half Duty cycle = ((TOP+1)/2-1)
				TCCR0A |= (1<<COM0B1);  // Freq out: pin OC0B
				DDRA |= (1<<PORTA7); // out
				goto xSetupOSCCAL;
				do {
					__asm__ volatile ("" ::: "memory"); // Need memory barrier
					LED1_ON;
					sleep_cpu();
					if(!KEY1_PRESSING) {
						if(Key1Pressed > 2) {
							if(Key1Pressed < 80) { // ~ 1 sec
								OSCCAL++;
								FlashLED(1, 1, 1);
							} else {
								OSCCAL--;
								FlashLED(2, 1, 1);
							}
xSetupOSCCAL:				Key1Pressed = 0;
							Timer = 255;
						}
					}
				} while(Timer);
				FlashLED(7, 1, 1);
				EEPROM_write(EPROM_OSCCAL, OSCCAL);
				TCCR0A &= ~(1<<COM0B1);
				DDRA &= ~(1<<PORTA7); // in
				continue;
			}
			SetupIR = 0;
			goto xSetupIR;
			do {
				__asm__ volatile ("" ::: "memory"); // Need memory barrier
				sleep_cpu();
				if(IRReceived == IRIsCommandReceived) {
					if(SetupIR == 1) EEPROM_write(EPROM_IRCommandHead - 1 + SetupIR, IRHead);
					EEPROM_write(EPROM_IRCommandArray + (SetupIR-1) * sizeof(IRHash), (uint8_t) (IRHash % 256));
					EEPROM_write(EPROM_IRCommandArray + (SetupIR-1) * sizeof(IRHash) + 1, (uint8_t) (IRHash / 256));
					if(SetupIR < IRCommands) {
xSetupIR:				SetupIR++;
						FlashLED(3, 1, 1);
						LED_Warning = SetupIR;
						Delay100ms(15);
						IRReceived = 0;
						Timer = 255;
					} else {
						FlashLED(8, 1, 1);
						break;
					}
				}
			} while(Timer);
			IRReceived = 0;
			Key1Pressed = 0;
			SetupIR = 0;
			LED_Warning = 0;
		}
		if(Timer == 0)
		{
			uint16_t co2;
			if(Setup) goto xSetupExit;
xSendNow:
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				co2 = CO2Level;
			}
			if(CO2LevelAverageCnt == 255) { // first time
				for(i = 1; i < CO2LevelAverageArrayLength; i++)
					CO2LevelAverageArray[i] = co2;
			}
			if(CO2LevelAverageCnt >= CO2LevelAverageArrayLength) CO2LevelAverageCnt = 0;
			CO2LevelAverageArray[CO2LevelAverageCnt++] = co2;
			uint16_t average = 0;
			for(i = 0; i < CO2LevelAverageArrayLength; i++)
				average += CO2LevelAverageArray[i];
			uint8_t average1b = (average /= CO2LevelAverageArrayLength) / 16; 
			int8_t fanspeed = 0;
			static int8_t fanspeed_prev = 0;
			for(; fanspeed < FanSpeedMax; fanspeed++) {
				uint8_t t = EEPROM_read(EPROM_FanSpeedThreshold + fanspeed);
				if(average1b < t) { 
					// if there is a decrease of CO2 level - check delta
					if(fanspeed_prev <= fanspeed || (uint8_t)(t - average1b) >= EEPROM_read(EPROM_FanSpeedDelta)) break; 
				}
			}
			fanspeed_prev = fanspeed;
			if((fanspeed += FanSpeedOverride) > FanSpeedMax) fanspeed = FanSpeedMax;
			if(fanspeed < 0) fanspeed = 0;
			if(LowLight && fanspeed > (int8_t)(i = EEPROM_read(EPROM_LowLightMaxFanSpeed))) fanspeed = i;
			//NRF24_SetMode(NRF24_TransmitMode);
			for(uint8_t fan = 0; fan < EEPROM_read(EPROM_NumberFans); fan++)
			{
				uint8_t addr = EEPROM_read(EPROM_RFAddresses + fan);
				if(rf_last_addr != addr) {
					if(!NRF24_SetAddresses(addr)) {
						Set_LED_Warning(ERR_RF_SetAddr);
						rf_reset_chip();
						break;
					}
					rf_last_addr = addr;
				}
				int8_t fspeed = fanspeed + FanSpeedOverrideArray[fan];
				if(fspeed > FanSpeedMax) fspeed = FanSpeedMax;
				if(fspeed < 0) fspeed = 0;
				uint8_t RF_Changed = 0;
				i = EPROM_FanSpeedChangeArray;
				do {
					uint8_t j = EEPROM_read(i++);
					if(j == fan) {
						if((j = EEPROM_read(i++)) != 0xFF) { // Set RF channel
							NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_RF_CH,	j);
							RF_Changed = 1;
						}
						for(j = EEPROM_read(i++); j > 0; j--) { // Change speed if needed
							if((uint8_t) fspeed == EEPROM_read(i++)) {
								fspeed = EEPROM_read(i);
								goto xSkipArrayEnd;
							} else i++;
						}
					} else if(j == 0xFF) { 
						break;
					} else i += EEPROM_read(i + 1) * 2 + 2;
				} while(1);
xSkipArrayEnd:
				((send_data*) &NRF24_Buffer)->CO2level = co2;
				((send_data*) &NRF24_Buffer)->FanSpeed = FanSpeedForce[fan] != -1 ? FanSpeedForce[fan] : fspeed;
				((send_data*) &NRF24_Buffer)->Flags = LowLight;
				uint8_t err = NRF24_Transmit(NRF24_Buffer);
				if(RF_Changed) NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_RF_CH, EEPROM_read(EPROM_RF_Channel)); // restore RF channel
				uint8_t err2 = err == 2 ? ERR_RF_NotResp : (ERR_RF_Send + fan + 1);
				if(err)	{
					Set_LED_Warning(err2);
					rf_reset_chip();
					break;
				}
			}
			//NRF24_Powerdown();
			Timer = EEPROM_read(EPROM_TransmitPeriod);
		}
	}
}

/*
  Настройки:
  1. Настройка ИК команд - нажать кнопку (PB2 - pin 5) и держать более 3.2 сек.
		Далее в порядке очередности вводятся ИК команды: 1 - Up, 2 - Down, 3 - Ok, 4 - Fans override
		Номер вводимой команды промигивается
		Таймаут - 30 сек.
		4. Калибровка внутреннего генератора. Up/Down - изменение частоты, Ok - Сохранение. Вывод частоты 20480 Hz на OC0B - pin 6.
  2. Установка корректировки скорости для каждого вентилятора - ИК команда 4 - Fans override
		Сначала выбирается вентилятор - количество выспышек равно номеру вентилятора, подтверждение - нажать Ok.
		Далее задается дельта скорости вентилятора - светодиод промигивает значение, если меньше 0, то сначала идет 1 длинная вспышка.
		Подтверждение выбора - нажать Ok.
		При нажатии на ИК 4 (Fans override) переключение режима выбора одной скорости.
		Выход после 30 секунд бездействия.
  3. Настройки - нажать Ok два раза в течении 0.6 сек:
		Up/Down - выбор пункта, Ok - подтверждение выбора
		1. Выход
		2. Установка порога темноты по текущей освещенности через 5 сек после нажатия Ok, после подтверждения выбора
		3. Редактирование EEPROM, начиная с ячейки 1.
			1. Выбор ячейки Up/Down, подтверждение выбора - Ok (вывод в шестнадцатеричном формате, сначала старшие 4 бита, затем младшие)
			2. Изменение значения в ячейке, Ok сохранение
		4. Установка корректировки скорости для каждого вентилятора (см. п.2)
		5. Отображение текущего уровня CO2
*/