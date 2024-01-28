#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>

volatile uint16_t left_sensor = 0;
volatile uint16_t right_sensor = 0;
volatile uint16_t granica = 0;

void ADC_Init() 
{
	// Ustawienie referencji napiêcia na AVcc
	ADMUX |= (1 << REFS0);
	// W³¹czenie ADC i ustawienie preskalera na 128
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void read_left_sensor()
{
	// Wybór kana³u ADC - 1
	ADMUX |= (1 << MUX0);
	// Rozpoczêcie konwersji
	ADCSRA |= (1 << ADSC);
}

void read_right_sensor()
{
	// Wybór kana³u ADc - 0
	ADMUX &= ~(1 << MUX0);
	// Rozpoczêcie konwersji
	ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
	// sprawdzenie kana³u ADC
	if(ADMUX & (1 << MUX0))
	{
		left_sensor = ADC;
	}
	else
	{
		right_sensor = ADC;
	}
}

bool isPressedStartButton()
{
	if(!(PIND & (1 << PIND3)))
	{
		_delay_ms(20);
		if(!(PIND & (1 << PIND3)))
		{
			return true;
		}
	}
	return false;
}

bool isPressedPomiarKartka()
{
	if(!(PIND & (1 << PIND5)))
	{
		_delay_ms(20);
		if(!(PIND & (1 << PIND5)))
		{
			return true;
		}
	}
	return false;
}

bool isPressedPomiarTasma()
{
	if(!(PIND & (1 << PIND6)))
	{
		_delay_ms(20);
		if(!(PIND & (1 << PIND6)))
		{
			return true;
		}
	}
	return false;
}

void PWM_init()
{
	// Ustawienie trybu Fast PWM (bit WGM10 i WGM11) oraz ustawienie preskalera na 256 (bit CS12)
	TCCR1A |= (1 << WGM10) | (1 << WGM11);
	TCCR1B |= (1 << CS12);
		
	// Ustawienie wyjœcia na pinie OC2A (PB1 oraz PB2)
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	
}

bool LeftSensorOK(uint16_t sensor)
{
	if(sensor < granica)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

bool RightSensorOK(uint16_t sensor)
{
	if(sensor < granica)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void LeftMotor(uint8_t predkosc)
{
	OCR1A = predkosc; //PB1 D9
	PORTB |= (1 << PORTB0); //PB0 D8
	PORTD &= ~(1 << PORTD7); //PD7 D7
}
void RightMotor(uint8_t predkosc)
{
	OCR1B = predkosc; //PB2 D10
	PORTD |= (1 << PORTD4); //PD4 D4
	PORTD &= ~(1 << PORTD2); //PD2 D2
}
void StopMotors()
{
	PORTB &= ~(1 << PORTB0);
	PORTD &= ~(1 << PORTD7);
	PORTD &= ~(1 << PORTD4);
	PORTD &= ~(1 << PORTD2);
}
int main(void)
{
	//definiowanie zmiennych
	uint8_t wypelnienie1 = 150;
	uint8_t wypelnienie2 = 100;
	uint16_t pomiar_kartka = 0;
	uint16_t pomiar_tasma = 0;
	bool isMoving = false;

	//ustawienie pinów jako output
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1); //PWM
	DDRB |= (1 << DDB2); //PWM
	DDRD |= (1 << DDD4);
	DDRD |= (1 << DDD7);
	DDRD |= (1 << DDD2);
	
	//ustawienie pinów jako input - przyciski
	DDRD &= ~(1 << DDD3);
	DDRD &= ~(1 << DDD5);
	DDRD &= ~(1 << DDD6);
	
	//podci¹gniêcie rezystorów pullup - do przycisków
	PORTD |= (1 << PORTD3);
	PORTD |= (1 << PORTD5);
	PORTD |= (1 << PORTD6);
	
	//inicjalizacja ADC
	ADC_Init();
	
	//inicjalizacja PWM
	PWM_init();
	
	//w³¹czenie globalnych przerwañ
	sei();
	
	while (1)
	{
		read_left_sensor();
		read_right_sensor();
		
		//kalibracja czujników - ustalenie granicy
		if(isPressedPomiarTasma())
		{
			pomiar_tasma = ADC;
		}
		
		if(isPressedPomiarKartka())
		{
			pomiar_kartka = ADC;
		}
		
		granica = (pomiar_kartka + pomiar_tasma)/2;
		
		//g³ówny program - warunki jazdy pojazdu
		
		if(isPressedStartButton() && isMoving == false)
		{
			if(LeftSensorOK(left_sensor) == true && RightSensorOK(right_sensor) == true) //jeœli oba czujniki widz¹ liniê
			{
				//jazda prosto
				LeftMotor(wypelnienie2);
				RightMotor(wypelnienie2);
			}
			else if(LeftSensorOK(left_sensor) == false)
			{
				//jazda po ³uku w prawo
				LeftMotor(wypelnienie1);
				RightMotor(wypelnienie2);
			}
			else if(RightSensorOK(right_sensor) == false)
			{
				//jazda po ³uku w lewo
				LeftMotor(wypelnienie2);
				RightMotor(wypelnienie1);
			}
			isMoving = true;
		}
		else if(isPressedStartButton() && isMoving == true)
		{
			StopMotors();
			isMoving = false;
		}
	}
}

