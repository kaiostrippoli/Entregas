/**
 * IMT - Rafael Corsi
 * 
 * PIO - 07
 *  Configura o PIO do SAM4S (Banco A, pino 19) para operar em
 *  modo de output. Esse pino está conectado a um LED, que em 
 *  lógica alta apaga e lógica baixa acende.
*/

#include <asf.h>

/*
 * Prototypes
 */

/** 
 * Definição dos pinos
 * Pinos do uC referente aos LEDS.
 *
 * O número referente ao pino (PIOAxx), refere-se ao
 * bit que deve ser configurado no registrador para alterar
 * o estado desse bit específico.
 *
 * exe : O pino PIOA_19 é configurado nos registradores pelo bit
 * 19. O registrador PIO_SODR configura se os pinos serão nível alto.
 * Nesse caso o bit 19 desse registrador é referente ao pino PIOA_19
 *
 * ----------------------------------
 * | BIT 19  | BIT 18  | ... |BIT 0 |
 * ----------------------------------
 * | PIOA_19 | PIOA_18 | ... |PIOA_0|
 
 * ----------------------------------
 */
#define PIN_LED_BLUE 19
<<<<<<< HEAD
#define PIN_LED_GREEN_RED 20

=======
#define BUTTON_B2 3
>>>>>>> origin/master

/**
 * Main function
 * 1. configura o clock do sistema
 * 2. desabilita wathdog
 * 3. ativa o clock para o PIOA
 * 4. ativa o controle do pino ao PIO
 * 5. desabilita a proteção contra gravações do registradores
 * 6. ativa a o pino como modo output
 * 7. coloca o HIGH no pino
 */

void Set_PIO()
{
	// 29.17.4 PMC Peripheral Clock Enable Register 0
	// 1: Enables the corresponding peripheral clock.
	// ID_PIOA = 11 - TAB 11-1
<<<<<<< HEAD
	PMC->PMC_PCER0 = ID_PIOA;
	PMC->PMC_PCER0 = ID_PIOC;

	//31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).
=======
	PMC->PMC_PCER0 = 1 << ID_PIOB;
	PMC->PMC_PCER0 = 1 << ID_PIOA;


	//31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).
	PIOB->PIO_PER = (1 << BUTTON_B2);
>>>>>>> origin/master
	PIOA->PIO_PER = (1 << PIN_LED_BLUE );
	PIOA->PIO_PER = (1 << PIN_LED_GREEN_RED );

	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII).
	PIOA->PIO_WPMR = 0;   
	
	// 31.6.4 PIO Output Enable Register
	// value =
	//	 	1 : Enables the output on the I/O line.
	//	 	0 : do nothing
	
	PIOA->PIO_OER =  (1 << PIN_LED_BLUE );
<<<<<<< HEAD
	PIOA->PIO_OER =  (1 << PIN_LED_GREEN_RED );
	PIOC->PIO_OER =  (1 << PIN_LED_GREEN_RED );

=======
	PIOB->PIO_ODR = (1 << BUTTON_B2);
>>>>>>> origin/master
	// 31.6.10 PIO Set Output Data Register
	// value =
	// 		1 : Sets the data to be driven on the I/O line.
	// 		0 : do nothing
	//PIOA->PIO_SODR = (1 << PIN_LED_BLUE );
	
<<<<<<< HEAD
}

void Set_LEDON( Pio *pin, int led)
{
	if(pin == PIOC)
	{
		pin->PIO_SODR = (1 << led);
	}
	else if(pin == PIOA)
	{
		pin->PIO_CODR = (1 << led);
	}
}

void Set_LEDOFF( Pio *pin, int led)
{
	if(pin == PIOC)
	{
		pin->PIO_CODR = (1 << led);
	}
	else if(pin == PIOA)
	{
		pin->PIO_SODR = (1 << led);
	}
}

void Change_LED(Pio *pin, int led)
{
	volatile int tmp = pin->PIO_ODSR;
	
		if(pin == PIOC)
		{
			if((tmp >> led) == 1)
			{
				pin->PIO_CODR = (1 << led);
			}
			else
			{
				pin->PIO_SODR = (1 << led);
			}
		}
		else if(pin == PIOA)
		{
			if((pin->PIO_ODSR << led) == 1)
			{
				pin->PIO_SODR = (1 << led);
			}
			else
			{
				pin->PIO_CODR = (1 << led);
			}
		}
}

void Blink_LED(int time_ms)
{
	Set_LEDOFF(PIOC, PIN_LED_GREEN_RED);
	Set_LEDON(PIOA, PIN_LED_BLUE);
	//PIOC->PIO_CODR = (1 << PIN_LED_GREEN_RED );
	//PIOA->PIO_CODR = (1 << PIN_LED_BLUE );
	delay_ms(time_ms);
	Set_LEDOFF(PIOA, PIN_LED_BLUE);
	Set_LEDON(PIOA, PIN_LED_GREEN_RED);
	//PIOA->PIO_SODR = (1 << PIN_LED_BLUE );
	//PIOA->PIO_CODR = (1 << PIN_LED_GREEN_RED);
	delay_ms(time_ms);
	Set_LEDOFF(PIOA, PIN_LED_GREEN_RED);
	Set_LEDON(PIOC, PIN_LED_GREEN_RED);
	//PIOA->PIO_SODR = (1 << PIN_LED_GREEN_RED );
	//PIOC->PIO_SODR = (1 << PIN_LED_GREEN_RED);
	delay_ms(time_ms);
}

void Blink_Invert(int time_ms)
{
	Change_LED(PIOC, PIN_LED_GREEN_RED);
	Change_LED(PIOA, PIN_LED_BLUE);
	//PIOC->PIO_CODR = (1 << PIN_LED_GREEN_RED );
	//PIOA->PIO_CODR = (1 << PIN_LED_BLUE );
	delay_ms(time_ms);
	Change_LED(PIOA, PIN_LED_BLUE);
	Change_LED(PIOA, PIN_LED_GREEN_RED);
	//PIOA->PIO_SODR = (1 << PIN_LED_BLUE );
	//PIOA->PIO_CODR = (1 << PIN_LED_GREEN_RED);
	delay_ms(time_ms);
	Change_LED(PIOA, PIN_LED_GREEN_RED);
	Change_LED(PIOC, PIN_LED_GREEN_RED);
	//PIOA->PIO_SODR = (1 << PIN_LED_GREEN_RED );
	//PIOC->PIO_SODR = (1 << PIN_LED_GREEN_RED);
	delay_ms(time_ms);
}


int main (void)
{

	/**
	* Inicializando o clock do uP
	*/
	sysclk_init();
	
	/** 
	*  Desabilitando o WathDog do uP
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	Set_PIO();
=======
	PIOB->PIO_PUER = (1 << BUTTON_B2); 
	PIOB->PIO_IFER = (1 << BUTTON_B2); // ATIVANDO O DEBOUNCING
>>>>>>> origin/master
	
	/**
	*	Loop infinito
	*/
		while(1){
<<<<<<< HEAD
			Blink_Invert(100);
=======
			if (((PIOB->PIO_PDSR >> BUTTON_B2) & 1) == 0)
			{
				PIOA->PIO_CODR = (1 << PIN_LED_BLUE );			
				//delay_ms(1000);

			}
			else
			{
							PIOA->PIO_SODR = (1 << PIN_LED_BLUE );
					//		delay_ms(1000);
			}
			
			//PIOA->PIO_SODR = (1 << PIN_LED_BLUE );
			//delay_ms(1000);
>>>>>>> origin/master
            /*
             * Utilize a função delay_ms para fazer o led piscar na frequência
             * escolhida por você.
             */
            //delay_ms();
		
	}
}



