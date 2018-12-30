/*
 * WirelessCO2Fan.c
 *
 * Created: 01.11.2013 13:06:02
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny44A
 * 
 */ 
#define F_CPU 8000000UL   // OSCCAL = 0x81 (VCC = 3.3V)
// Fuses: BODLEVEL = 1V8

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
//#include <util/crc16.h>

#define LED1_PORT					PORTA	// Info led
#define LED1						(1<<PORTA1)
#define LED1_ON						LED1_PORT |= LED1
#define LED1_OFF					LED1_PORT &= ~LED1

#define KEY1						(1<<PORTB2) // = IR Receiver out
#define KEY1_PRESSING				!(PINB & KEY1)
#define KEY1_PressingTimeMin		10 // longer than 125ms

#define PWM_MAX						50 // 20000hz
#define FanPWM						OCR0B
#define FanSwitch_PORT				PORTB
#define FanSwitch_Mask				((1<<PORTB1) | (1<<PORTB0)) // Low bit = low capacitance, 3 steps
#define PWM_Fan_Power				(1<<PORTB0) // if EPROM_Speed_0_as_Off == 1 
#define PWM_Fan_On					FanSwitch_PORT |= PWM_Fan_Power
#define PWM_Fan_Off					FanSwitch_PORT &= ~PWM_Fan_Power
#define FanSpeedOverrideTimeOn		44000 // 91 min (*0.125 sec)
uint8_t FanType;
uint8_t FanSpeedMax;
uint8_t FanSpeed					= 0; // 0..FanSpeedMax
uint8_t FanSpeedOverride			= 0;
uint16_t FanSpeedOverrideTimer		= 0;

#define IRCommands					3
#define IRKey_Up					0
#define IRKey_Down					1
#define IRKey_FanOverride			2
#define IRIsCommandReceived			3
uint8_t IRSignalTimer				= 0;
uint8_t IRReceived					= 0; // 0 - waiting new, 1 - receiving first pulse, 2 - receiving, 3 - received
uint8_t IRHead = 0, IRArrayBit, IRReadedByte, IRSignalTimerLast;
uint16_t IRHash;
uint8_t Key1Pressed					= 0;
uint8_t SetupIR						= 0; // > 0 = setup item
uint8_t LED_mode;

volatile uint8_t Timer				= 0;
uint8_t TimerCnt30Sec				= 0;
uint8_t TimerLowLight				= 255;
uint8_t LowLightLast				= 0;

#define EPROM_OSCCAL				0x00
#define EPROM_RFAddress				0x01 // Receive address LSB (one of: 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6)
#define EPROM_FanType				0x02 // 0 - PWM (out: OC0B), 1 - switch in 3 steps (out: PB0, PB1)
#define EPROM_IRReceiver			0x03 // 1 - IR Receiver available on PORTB2
#define EPROM_PWM_Zero				0x04 // Zero value for PWM fan
#define EPROM_FanSpeedDivider		0x05 // Divider for received speed: 1 for PWM, 2 - otherwise
#define EPROM_FanSpeedMax			0x06 // 6 for PWM, 3 - otherwise
#define EPROM_FanSpeedInit			0x07 // Default speed after power on
#define EPROM_LedMode				0x08 // 0b001 - flash speed when received, 0b010 - Led turned on in override mode, 0b100 - Show retranslate error 
#define EPROM_ReceivedFanSpeedMin	0x09 // Fan speed can't be less than this value when received.
#define EPROM_RF_Channel			0x0A // 120
#define EPROM_Speed_0_as_Off		0x0B // if = 1 -> use switch port as off switch when speed = 0
#define EPROM_FanOffAtNight			0x0C // if = 1 -> when night - fan off
#define EPROM_TimeLightToChange		0x0D // 30, *1/2 minutes, How long time light must off/on to switch fan off/on
#define EPROM_PWM_MAX_Minus_1		0x0E // Max value for PWM fan
#define EPROM_IRCommandHead			0x10 // length of head pulse
#define EPROM_IRCommandArray		0x11 // array of commands
#define EPROM_IRCommandArrayEnd		EPROM_IRCommandArray + IRCommands * sizeof(IRHash)
#define EPROM_Retranslate			0x20 // Number of devices to retranslate received packet 
#define EPROM_RetranslateRFAddr		0x21 // Addresses for retranslating
#define EPROM_RetranslateRFAddrEnd	0x22

typedef struct {
	uint16_t CO2level;
	uint8_t FanSpeed;
	uint8_t Flags; // Mask: 0x80 - Setup command, 0x01 - Lowlight
} __attribute__ ((packed)) send_data;

#define FLAG_LowLight				0x01

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

static uint8_t EEPROM_read(uint8_t ucAddress) // ATtiny24A/44A only!
{
	while(EECR & (1<<EEPE)) ; // EEWE
	EEAR = ucAddress;
	EECR |= (1<<EERE);
	return EEDR;
}
static void EEPROM_write(uint8_t ucAddress, uint8_t ucData) // ATtiny24A/44A only!
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

#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0); //  Watchdog 0.125 s
uint8_t LED_Warning = 0, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0;

ISR(WATCHDOG_vect)
{
	SETUP_WATCHDOG;
	if(Timer) Timer--;
	if(++TimerCnt30Sec == 240) { // 30 sec
		TimerCnt30Sec = 0;
		if(TimerLowLight < 255) TimerLowLight++;
	}
	if(FanSpeedOverrideTimer) if(--FanSpeedOverrideTimer == 0) FanSpeedOverride = 0;
	if(LED_WarningOnCnt) {
		LED1_ON;
		LED_WarningOnCnt--;
	} else if(LED_WarningOffCnt) {
		LED1_OFF;
		LED_WarningOffCnt--;
	} else if(LED_Warning) { // short flashes
		LED_WarningOffCnt = 2;
		LED_WarningOnCnt = 2;
		if(--LED_Warning == 0) LED_WarningOffCnt = 15;
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

void SetFanSpeed(uint8_t speed)
{
	if(FanType == 0) { // PWM
//		FanPWM = speed * 8;
		if(EEPROM_read(EPROM_Speed_0_as_Off) == 1) {
			if(speed != 0) {
				PWM_Fan_On;
			} else {
				PWM_Fan_Off;
			}
		}
		uint8_t zero = EEPROM_read(EPROM_PWM_Zero);
		if(speed == FanSpeedMax) {
			FanPWM = PWM_MAX;
		} else {
			FanPWM = ((uint8_t)(EEPROM_read(EPROM_PWM_MAX_Minus_1) - zero)) * speed / (FanSpeedMax - 1) + zero;
		}
	} else {
		if(speed > FanSpeedMax) speed = FanSpeedMax;
		FanSwitch_PORT = (FanSwitch_PORT & ~FanSwitch_Mask) | (speed & FanSwitch_Mask);
	}
	LED_WarningOffCnt = 5;
	LED_Warning = (LED_mode & 0b001 ? speed : 0) + 1;
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler division factor: 1
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	DDRA = LED1; // Out
	PORTA = (1<<PORTA0); // pullup not used pins
	NRF24_DDR |= NRF24_CE | NRF24_CSN | NRF24_SCK | NRF24_MOSI; // Out
	// Timer 8 bit
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00); // Timer0 prescaller: 8
	TIMSK0 |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	OCR0A = PWM_MAX; // OC0A - Fclk/(prescaller*(1+TOP)) 
	OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	// ADC
 	//ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
 	//ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
 	//ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	FanType = EEPROM_read(EPROM_FanType);
	if(FanType == 0xFF) {
		EEPROM_write(EPROM_OSCCAL, OSCCAL);
		EEPROM_write(EPROM_FanType, 0);
	}
	OSCCAL = EEPROM_read(EPROM_OSCCAL);
	FanSpeedMax = EEPROM_read(EPROM_FanSpeedMax);
	if(FanType == 0)
	{ // PWM
		DDRA |= (1<<PORTA7); // Out
		if(EEPROM_read(EPROM_Speed_0_as_Off) == 1) {
			DDRB |= PWM_Fan_Power;
			PORTB |= FanSwitch_Mask ^ PWM_Fan_Power; // pull up not used
		} else {
			PORTB |= FanSwitch_Mask; // pull up not used
		}
		TCCR0A |= (1<<COM0B1); // Start PWM out
	} else {
		PORTA |= (1<<PORTA7);  // pullup not used
		DDRB = FanSwitch_Mask;  // Out
	}
	// Prepare for IR receiving
	GIMSK |= (1<<PCIE1); // Pin Change Interrupt Enable 1
	PCMSK1 |= (1<<PCINT10); // Pin Change Mask Register 0 - Keys
	if(EEPROM_read(EPROM_IRReceiver)) { 
		IRHead = EEPROM_read(EPROM_IRCommandHead); // must be != 0
	} else {
		PORTB |= (1<<PORTB2); // pullup for Key1
	}
	LED_mode = EEPROM_read(EPROM_LedMode);
	NRF24_init(EEPROM_read(EPROM_RF_Channel)); // After init transmit must be delayed
	SETUP_WATCHDOG;
	sei();
	SetFanSpeed(EEPROM_read(EPROM_FanSpeedInit));
xStartReceiving:
	while(!NRF24_SetAddresses(EEPROM_read(EPROM_RFAddress))) {
		FlashLED(1, 50, 20);
	}
	NRF24_SetMode(NRF24_ReceiveMode); // Receive mode
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		if(IRReceived == IRIsCommandReceived)
		{
			for(uint8_t i = 0; i < IRCommands; i++) {
				if(EEPROM_read(EPROM_IRCommandArray + i * sizeof(IRHash)) == (uint8_t) (IRHash % 256) 
					&& EEPROM_read(EPROM_IRCommandArray + i * sizeof(IRHash) + 1) == (uint8_t) (IRHash / 256))
				{
					if(i == IRKey_FanOverride) {
						if(Timer == 0) {
							if(FanSpeedOverride == 0) {
								Timer = 5; // 0.6 sec wait for the next pressing
								FanSpeedOverride = 1;
							} else FanSpeedOverride = 0;
							SetFanSpeed(FanSpeedOverride ? FanSpeedMax : FanSpeed);
							FanSpeedOverrideTimer = FanSpeedOverrideTimeOn;
						} else { // 
							ATOMIC_BLOCK(ATOMIC_FORCEON) FanSpeedOverrideTimer = 0;
						}
					} else if(FanSpeedOverride) {
						if(i == IRKey_Up) {
							if(FanSpeed < FanSpeedMax) FanSpeed++;
						} else if(i == IRKey_Down) {
							if(FanSpeed > 0) FanSpeed--;
						}
						SetFanSpeed(FanSpeed);
					}
					break;
				}
			}
			IRReceived = 0;
		}
		if(Key1Pressed == 255) // Press and hold key1 for ~3.2 sec - Change FanOfAtNight flag
		{	
			FlashLED(2, 2, 1);
			Key1Pressed = 0;
			do {
				__asm__ volatile ("" ::: "memory"); // Need memory barrier
				Delay100ms(1);
				if(Key1Pressed == 255) break;
			} while(KEY1_PRESSING);
			if(Key1Pressed != 255) {
				uint8_t i;
				EEPROM_write(EPROM_FanOffAtNight, (i = EEPROM_read(EPROM_FanOffAtNight) ^ 1));
				FlashLED(i, 10, 10);
			} else { // Press and hold key1 for ~3.2 sec - Enter setup
				// Setup:
				// 1. Set Zero PWM, Max PWM, Retranslate on/off, remote setup
				// Short press "+", long press "-"
				FanSpeedOverride = 0;
				if(FanType == 0) { // PWM fan
					FlashLED(3, 2, 1);
					Key1Pressed = 0;
					do {
						__asm__ volatile ("" ::: "memory"); // Need memory barrier
						Delay100ms(1);
						if(Key1Pressed == 255) break;
					} while(KEY1_PRESSING);
					if(Key1Pressed == 255) { // >= ~3.2 sec
						FlashLED(4, 2, 1);
						uint8_t Setup = 0;
						Key1Pressed = 0;
						do { Delay100ms(1); } while(KEY1_PRESSING);
						if(Key1Pressed == 255) { // >= ~3.2 sec
							FlashLED(5, 2, 1);
							Setup = 1;
						}
						FanPWM = EEPROM_read(Setup ? EPROM_PWM_MAX_Minus_1 : EPROM_PWM_Zero);
						goto xSetupPWMZero;
						do {
							__asm__ volatile ("" ::: "memory"); // Need memory barrier
							sleep_cpu();
							if(!KEY1_PRESSING) {
								if(Key1Pressed > 1) { 
									if(Key1Pressed == 255) { // Set retranslate
										FlashLED(6, 2, 1);
										Key1Pressed = 0;
										do { Delay100ms(1); } while(KEY1_PRESSING);
										if(Key1Pressed == 255) { // 2 times more 3 sec
											uint8_t i;
											if((i = EEPROM_read(EPROM_Retranslate) + 1) > 2) i = 0; // Max retranslate cnt
											EEPROM_write(EPROM_Retranslate, i);
											FlashLED(i, 10, 10);
											Delay100ms(10);
										}
										goto xSetupPWMZero;
									}
									if(Key1Pressed < 80) { // ~ 1 sec
										if(FanPWM < PWM_MAX) {
											FanPWM++;
											FlashLED(1, 1, 1);
										}
									} else if(FanPWM > 0) {
										FanPWM--;
										FlashLED(2, 1, 1);
									}
xSetupPWMZero:						Key1Pressed = 0;
									Timer = 255;
								}
							}
						} while(Timer);
						FlashLED(7, 1, 1);
						EEPROM_write(Setup ? EPROM_PWM_MAX_Minus_1 : EPROM_PWM_Zero, FanPWM);
						goto xSetupExit;
					}
				}
				if(IRHead) {
					SetupIR = 1;
					goto xSetupIR;
					do {
						__asm__ volatile ("" ::: "memory"); // Need memory barrier
						sleep_cpu();
						if(LED_Warning == 0) LED_Warning = SetupIR;
						if(IRReceived == IRIsCommandReceived) {
							if(SetupIR == 1) EEPROM_write(EPROM_IRCommandHead - 1 + SetupIR, IRHead);					
							EEPROM_write(EPROM_IRCommandArray + (SetupIR-1) * sizeof(IRHash), (uint8_t) (IRHash % 256));
							EEPROM_write(EPROM_IRCommandArray + (SetupIR-1) * sizeof(IRHash) + 1, (uint8_t) (IRHash / 256));
							if(SetupIR < IRCommands) {
								SetupIR++;
xSetupIR:						FlashLED(7, 1, 1);
								Delay100ms(10);
								IRReceived = 0;
								Timer = 255;
							} else {
								FlashLED(8, 1, 1);
								break;
							}
						}
					} while(Timer);
				}
			}
xSetupExit:	IRReceived = 0;
			Key1Pressed = 0;
			SetupIR = 0;
		} else if(Key1Pressed >= KEY1_PressingTimeMin && !KEY1_PRESSING) { // Key is released
			FanSpeedOverride = 1;
			if(++FanSpeed > FanSpeedMax) FanSpeed = 0;
			EEPROM_write(EPROM_FanSpeedInit, FanSpeed);
			Key1Pressed = 0;
			SetFanSpeed(FanSpeed);
		}
		if(NRF24_Receive(NRF24_Buffer)) {
			if(FanSpeedOverride == 0) {
				FanSpeed = (((send_data*) &NRF24_Buffer)->FanSpeed) / EEPROM_read(EPROM_FanSpeedDivider);
				if(EEPROM_read(EPROM_FanOffAtNight)) {
					uint8_t lt = ((send_data*) &NRF24_Buffer)->Flags & FLAG_LowLight;
					if(lt != LowLightLast && TimerLowLight < EEPROM_read(EPROM_TimeLightToChange)) {
						lt = LowLightLast;
					}
					LowLightLast = lt;
					TimerLowLight = 0;
					if(lt) {
						FanSpeed = 0;
						FlashLED(1,1,1);
					}
				}
				uint8_t i = EEPROM_read(EPROM_ReceivedFanSpeedMin);
				if(FanSpeed < i) FanSpeed = i;
				SetFanSpeed(FanSpeed);
			}
			uint8_t addrs = EEPROM_read(EPROM_Retranslate);
			if(addrs) {
				Delay10us(50);
				NRF24_SetMode(NRF24_TransmitMode);
				for(uint8_t i = 0; i < addrs; i++) {
					if(NRF24_SetAddresses(EEPROM_read(EPROM_RetranslateRFAddr + i))) {
						uint8_t err = NRF24_Transmit(NRF24_Buffer);
						if(err && (LED_mode & 0b100)) {
							LED_Warning = 0;
							FlashLED(err, 5, 10);
							FlashLED(i + 1, 2, 2);
						}
					}
				}
				goto xStartReceiving;
			}
		}
		if(LED_WarningOnCnt == 0 && LED_WarningOffCnt == 0 && LED_Warning == 0 && LED_mode & 0b010) {
			// Led turned on if endless override, flashing every 1 sec if override timeout active
			if(FanSpeedOverride && ((uint8_t) FanSpeedOverrideTimer & 8) == 0) LED1_ON; else LED1_OFF;
		}
	}
}

/*
  * Кратковременное нажатие на кнопку - включение режима принудительной скорости,
	переключение скорости по кругу с запоминаем этой скорости при включении питания
	(скорость промигивается)
  * Длительное нажатие - через 4 секунды - 2 вспышки, если кнопку отпустить -
	то переключение режима ночного отключения вентилятора (минимальная скорость)
  * Продолжание удерживание кнопки еще на 4 секунды - если вентилятор не PWM -
	7 вспышек и вход в режим настройки пульта: 3 команды: вверх, вниз, принудительно.
  * Если PWM - 3 вспышки, если дальше удерживать кнопку на 4 сек - 4 вспышки -
	вход в настройку нулевого значения PWM, иначе переход на настройку пульта,
	если есть.
  * Если дальше удерживать (4 сек), то вход в настройку максимального значения PWM.
	- 5 вспышек.
  * Если во время настройки PWM удерживать кнопку 8 секунд - переключения режима
	ретрансляции

	Короткое нажатие (меньше 1 сек) - 1 вспышка, PWM + 1
	Длительное нажатие              - 2 вспышки, PWM - 1
	Выход и сохранение по таймауту 30 секунд.

*/

/*
	SetFanSpeed(FanSpeed);
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		if(Key1Pressed && !KEY1_PRESSING) { // Key is released
			if(++FanSpeed > FanSpeedMax) FanSpeed = 0;
			Key1Pressed = 0;
			SetFanSpeed(FanSpeed);
		}
	}
*/

/*
		// Set OSCALL
		PORTB |= FanSwitch_Mask; // pull up not used
		DDRA |= (1<<PORTA7); // Out
		TCCR0A |= (1<<COM0B1); // Start PWM out
		OCR0B = (OCR0A + 1) / 2 - 1;
		Timer = 255;
		do {
			__asm__ volatile ("" ::: "memory"); // Need memory barrier
			sleep_cpu();
			if(!KEY1_PRESSING) {
				if(Key1Pressed > 15) // long pressing
				{
					OSCCAL--;
					goto x_next;
				} else if(Key1Pressed > 1) // short pressing
				{
					OSCCAL++;
x_next:				Key1Pressed = 0;
					Timer = 255;
				}
			}
		} while(Timer);
		FlashLED(5, 2, 2);
		EEPROM_write(EPROM_OSCCAL, OSCCAL);
		while(1) ;
*/
