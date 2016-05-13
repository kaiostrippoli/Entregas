/************************************************************************
* Timer Counter
*
* Exemplo de utilização do modo RC Compare
* Nesse exemplo o uC entra em modo de stand-by e aguarda por três diferentes
* interrupcoes :
*
* 1 - Timer
*	Muda o estado do led em uma frequencia pré definida (inicial 4Hz)
*
* 2 - Botao 1
*	Altera a frequencia do led em 2Hz para cima
*	
* 3 - Botao 2
*	Altera a frequencia do led em 2Hz para baixo
*
************************************************************************/

#include "asf.h"
#include "Driver/pio_maua.h"
#include "Driver/pmc_maua.h"


#define PIN_PUSHBUTTON_1_MASK	PIO_PB3
#define PIN_PUSHBUTTON_1_PIO	PIOB
#define PIN_PUSHBUTTON_1_ID		ID_PIOB
#define PIN_PUSHBUTTON_1_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE

#define PIN_PUSHBUTTON_2_MASK	PIO_PC12
#define PIN_PUSHBUTTON_2_PIO	PIOC
#define PIN_PUSHBUTTON_2_ID		ID_PIOC
#define PIN_PUSHBUTTON_2_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE

#define PIN_LED_BLUE	19
#define PIN_LED_RED		20
#define PIN_LED_GREEN	20
#define PIN_BUTTON_2	3
#define PIN_BUTTON_3	12

#define ID_BUT_2		ID_PIOB
#define ID_BUT_3		ID_PIOC

#define MASK_LED_BLUE	(1 << PIN_LED_BLUE)
#define MASK_LED_GREEN	(1 << PIN_LED_GREEN)
#define MASK_LED_RED	(1 << PIN_LED_RED)
#define MASK_BUT_2		(1 << PIN_BUTTON_2)
#define MASK_BUT_3		(1 << PIN_BUTTON_3)

#define ID_TIMER		ID_TC0


/** 
 * Definição dos ports
 * Ports referentes a cada pino
 */
#define PORT_LED_BLUE	PIOA
#define PORT_LED_GREEN	PIOA
#define PORT_LED_RED	PIOC
#define PORT_BUT_2		PIOB
#define PORT_BUT_3		PIOC

/** IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

#define Freq_Init_Blink 4	//Hz

/**
 *  Handle Interrupcao botao 1
 */
static void Button2_Handler(uint32_t id, uint32_t mask)
{
tc_write_rc(TC0, 0, tc_read_rc(TC0, 0) + 2730);
}

/**
 *  Handle Interrupcao botao 2.
 */
static void Button3_Handler(uint32_t id, uint32_t mask)
{
tc_write_rc(TC0, 0, tc_read_rc(TC0, 0) - 2730);
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */
void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0,0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */

	if (((PIOA->PIO_ODSR >> PIN_LED_GREEN) & 1) == 0) {
		//PIOA->PIO_SODR = (1 << PIN_LED_GREEN );
		pio_set(PIOA,(1<<PIN_LED_GREEN));
	}
	else
	{
		//PIOA->PIO_CODR = (1 << PIN_LED_GREEN );
		pio_clear(PIOA, (1 << PIN_LED_GREEN));
	}

}

/**
 *  \brief Configure the Pushbuttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void configure_buttons(void)
{	
	pmc_enable_periph_clk(ID_BUT_2);
	pmc_enable_periph_clk(ID_BUT_3);
	/**
	* Configura entrada
	*/ 
	pio_set_input(PORT_BUT_2, MASK_BUT_2, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(PORT_BUT_3, MASK_BUT_3, PIO_PULLUP | PIO_DEBOUNCE);
	
	/*
	 * Configura divisor do clock para debounce
	 */
	pio_set_debounce_filter(PORT_BUT_2,MASK_BUT_2, 100);
	pio_set_debounce_filter(PORT_BUT_3,MASK_BUT_3, 100);
	
	/* 
	*	Configura interrupção para acontecer em borda de descida.
	*/
	pio_handler_set(PORT_BUT_2,ID_BUT_2,MASK_BUT_2,PIO_IT_FALL_EDGE,Button2_Handler);
	pio_handler_set(PORT_BUT_3,ID_BUT_3,MASK_BUT_3,PIO_IT_FALL_EDGE,Button3_Handler);
				
	/*
	*	Ativa interrupção no periférico B porta do botão
	*/	
	pio_enable_interrupt(PORT_BUT_2,MASK_BUT_2);
	pio_enable_interrupt(PORT_BUT_3,MASK_BUT_3);
	
	/*
	*	Configura a prioridade da interrupção no PORTB E PORTC
	*/
	NVIC_SetPriority(PIOB_IRQn, 10);
	NVIC_SetPriority(PIOC_IRQn, 10);
	
	/*
	*	Ativa interrupção no port B e C
	*/
	NVIC_EnableIRQ(PIOB_IRQn);
	NVIC_EnableIRQ(PIOC_IRQn);
		
}


/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
// [main_tc_configure]
static void configure_led(void)
{
		pio_set_output(PORT_LED_BLUE  , MASK_LED_BLUE	,1,0,0);
		pio_set_output(PORT_LED_GREEN , MASK_LED_GREEN  ,1,0,0);
		pio_set_output(PORT_LED_RED	  , MASK_LED_RED	,1,0,0);
}


static void configure_tc(void)
{
	/*
	* Aqui atualizamos o clock da CPU que foi configurado em sysclk init
	*
	* O valor atual está em : 120_000_000 Hz (120Mhz)
	*/
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	
	/*
	*	Ativa o clock do periférico TC 0
	* 
	*/
	pmc_enable_periph_clk(ID_TIMER);

	/*
	* Configura TC para operar no modo de comparação e trigger RC
	* devemos nos preocupar com o clock em que o TC irá operar!
	*
	* Cada TC possui 3 canais, escolher um para utilizar.
	*
	* Configurações de modo de operação :
	*	#define TC_CMR_ABETRG (0x1u << 10) //: TIOA or TIOB External Trigger Selection 
	*	#define TC_CMR_CPCTRG (0x1u << 14) //: RC Compare Trigger Enable 
	*	#define TC_CMR_WAVE   (0x1u << 15) //: Waveform Mode 
	*
	* Configurações de clock :
	*	#define  TC_CMR_TCCLKS_TIMER_CLOCK1 : Clock selected: internal MCK/2 clock signal 
	*	#define  TC_CMR_TCCLKS_TIMER_CLOCK2 : Clock selected: internal MCK/8 clock signal 
	*	#define  TC_CMR_TCCLKS_TIMER_CLOCK3 : Clock selected: internal MCK/32 clock signal 
	*	#define  TC_CMR_TCCLKS_TIMER_CLOCK4 : Clock selected: internal MCK/128 clock signal
	*	#define  TC_CMR_TCCLKS_TIMER_CLOCK5 : Clock selected: internal SLCK clock signal 
	*
	*	MCK		= 120_000_000
	*	SLCK	= 32_768		(rtc)
	*
	* Uma opção para achar o valor do divisor é utilizar a funcao
	 tc_find_mck_divisor()
	*/
	tc_init(TC0,0,TC_CMR_CPCTRG|TC_CMR_TCCLKS_TIMER_CLOCK5);
	
	/*
	* Aqui devemos configurar o valor do RC que vai trigar o reinicio da contagem
	* devemos levar em conta a frequência que queremos que o TC gere as interrupções
	* e tambem a frequencia com que o TC está operando.
	*
	* Devemos configurar o RC para o mesmo canal escolhido anteriormente.
	*	
	*   ^ 
	*	|	Contador (incrementado na frequencia escolhida do clock)
	*   |
	*	|	 	Interrupcao	
	*	|------#----------- RC
	*	|	  /
	*	|   /
	*	| /
	*	|-----------------> t
	*
	*
	*/
	tc_write_rc(TC0, 0, 8192);
	
	/*
	* Devemos configurar o NVIC para receber interrupções do TC 
	*/
	NVIC_EnableIRQ(ID_TC0);
	
	/*
	* Opções possíveis geradoras de interrupção :
	* 
	* Essas configurações estão definidas no head : tc.h 
	*
	*	#define TC_IER_COVFS (0x1u << 0)	Counter Overflow 
	*	#define TC_IER_LOVRS (0x1u << 1)	Load Overrun 
	*	#define TC_IER_CPAS  (0x1u << 2)	RA Compare 
	*	#define TC_IER_CPBS  (0x1u << 3)	RB Compare 
	*	#define TC_IER_CPCS  (0x1u << 4)	RC Compare 
	*	#define TC_IER_LDRAS (0x1u << 5)	RA Loading 
	*	#define TC_IER_LDRBS (0x1u << 6)	RB Loading 
	*	#define TC_IER_ETRGS (0x1u << 7)	External Trigger 
	*/
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	
	tc_start(TC0, 0);
}


/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/**Configura o LED */
	configure_led();

	/** Configura o timer */
	configure_tc();
	
	/* Configura os botões */
	configure_buttons();

	while (1) {
	
	
		/* Entra em modo sleep */
		
	}
}
