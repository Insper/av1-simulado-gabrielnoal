#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 
#define LED_PIO_ID        ID_PIOC              
#define LED_PIO_IDX       8                    
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   

#define BUT_PIO           PIOA                 
#define BUT_PIO_ID        ID_PIOA              
#define BUT_PIO_IDX       11                   
#define BUT_PIO_IDX_MASK  (1 << BUT_PIO_IDX)   

//LED 1
#define OLED_LED1_PIO	  PIOA
#define OLED_LED1_PIO_ID  ID_PIOA
#define OLED_LED1_PIO_IDX 0
#define OLED_LED1_PIO_IDX_MASK (1 << OLED_LED1_PIO_IDX)

//LED2
#define OLED_LED2_PIO	  PIOC
#define OLED_LED2_PIO_ID  ID_PIOC
#define OLED_LED2_PIO_IDX 30
#define OLED_LED2_PIO_IDX_MASK (1 << OLED_LED2_PIO_IDX)

//LED3
#define OLED_LED3_PIO	  PIOB
#define OLED_LED3_PIO_ID  ID_PIOB
#define OLED_LED3_PIO_IDX 2
#define OLED_LED3_PIO_IDX_MASK (1 << OLED_LED3_PIO_IDX)

//BOTÃO 1
#define OLED_BUT1_PIO	  PIOD
#define OLED_BUT1_PIO_ID  ID_PIOD
#define OLED_BUT1_PIO_IDX 28
#define OLED_BUT1_PIO_IDX_MASK (1 << OLED_BUT1_PIO_IDX)

//BOTÃO 2
#define OLED_BUT2_PIO	  PIOC
#define OLED_BUT2_PIO_ID  ID_PIOC
#define OLED_BUT2_PIO_IDX 31
#define OLED_BUT2_PIO_IDX_MASK (1 << OLED_BUT2_PIO_IDX)

//BOTÃO 3
#define OLED_BUT3_PIO	  PIOA
#define OLED_BUT3_PIO_ID  ID_PIOA
#define OLED_BUT3_PIO_IDX 19
#define OLED_BUT3_PIO_IDX_MASK (1 << OLED_BUT3_PIO_IDX)

//BUZZER
#define BUZZER_PIO             PIOC
#define BUZZER_PIO_ID          ID_PIOC
#define BUZZER_PIO_IDX         13
#define BUZZER_PIO_IDX_MASK    (1 << BUZZER_PIO_IDX)

/************************************************************************/
/* Constants                                                            */
/************************************************************************/
typedef struct {
	Pio *pio;
	int pio_id;
	int id;
	int mask;
	int initialState;
	int handlerConfig;
} component;
typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;
typedef void (*voidHandler)(void);

component led1 = {
	OLED_LED1_PIO,
	OLED_LED1_PIO_ID,
	OLED_LED1_PIO_IDX,
	OLED_LED1_PIO_IDX_MASK,
	PIO_OUTPUT_0
};
component led2 = {
	OLED_LED2_PIO,
	OLED_LED2_PIO_ID,
	OLED_LED2_PIO_IDX,
	OLED_LED2_PIO_IDX_MASK,
	PIO_OUTPUT_1
};
component led3 = {
	OLED_LED3_PIO,
	OLED_LED3_PIO_ID,
	OLED_LED3_PIO_IDX,
	OLED_LED3_PIO_IDX_MASK,
	PIO_OUTPUT_0
};

component leftButton = {
	OLED_BUT1_PIO,
	OLED_BUT1_PIO_ID,
	OLED_BUT1_PIO_IDX,
	OLED_BUT1_PIO_IDX_MASK,
	0,
	PIO_IT_RISE_EDGE,
	
};
component centerButton = {
	OLED_BUT2_PIO,
	OLED_BUT2_PIO_ID,
	OLED_BUT2_PIO_IDX,
	OLED_BUT2_PIO_IDX_MASK,
	0,
	PIO_IT_FALL_EDGE
};
component rightButton = {
	OLED_BUT3_PIO,
	OLED_BUT3_PIO_ID,
	OLED_BUT3_PIO_IDX,
	OLED_BUT3_PIO_IDX_MASK,
	0,
	PIO_IT_RISE_EDGE
};
/************************************************************************/
/* Global variables*/
/************************************************************************/

volatile char leftButtonFlag = 0;
volatile char centerButtonFlag = 0;
volatile char rightButtonFlag = 0;

volatile char string[50];

volatile char rttAlarm = 0;
volatile char blinkFlag = 0;

volatile char lcdFlag = 1;

volatile int seconds = 0;
volatile int secondsDozen = 0;

volatile int minutes = 0;
volatile int minutesDozen = 0;

volatile int hours = 0;
volatile int hoursDozen = 0;

/************************************************************************/
/* Prototypes*/
/************************************************************************/
void configureButtons(component buttons[], int size);
void configureLeds(component leds[], int size);
void enebleAllPeriph(int periphIdsList[], int size);
void pisca_led(int n, int t, component led);
void pin_toggle(component led);
void updateTime(void);
void writeLCD(void);

void init(void)
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(Rtc *rtc, uint32_t id_rtc, uint32_t irq_type);
static void RTT_init(void);



/************************************************************************/
/* Interruptions*/
/************************************************************************/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	//flag_tc_5hz = 1;
	if (leftButtonFlag && blinkFlag) pin_toggle(led1);
}
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	//flag_tc_10hz = 1;
	if (centerButtonFlag && blinkFlag) pin_toggle(led2);
}
void TC2_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	//flag_tc_10hz = 1;
	if (rightButtonFlag && blinkFlag) pin_toggle(led3);
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		updateTime();
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		rttAlarm = 1;                  // flag RTT alarme
		blinkFlag = !blinkFlag;
	}
}


void leftButtonHandler(void){
	leftButtonFlag = !leftButtonFlag;
}
void centerButtonHandler(void){
	centerButtonFlag = !centerButtonFlag;
}
void rigthButtonHandler(void){
	rightButtonFlag = !rightButtonFlag;
	
}
/************************************************************************/
/* Functions                                                            */
/************************************************************************/

void configureButtons(component buttons[], int size){
	voidHandler buttonsHandlers[3] = {
		leftButtonHandler,
		centerButtonHandler,
		rigthButtonHandler
	};
	for (int i = 0; i < size; i++)
	{
		component button = buttons[i];
		pio_configure(button.pio, PIO_INPUT, button.mask, PIO_PULLUP | PIO_DEBOUNCE );
		// Configura interrupção no pino referente ao botao e associa
		// função de callback caso uma interrupção for gerada
		pio_handler_set(
		button.pio,
		button.pio_id,
		button.mask,
		button.handlerConfig,
		buttonsHandlers[i]
		);
		// Ativa interrupção
		pio_enable_interrupt(button.pio, button.mask);
		
		// Configura NVIC para receber interrupcoes do PIO do botao
		// com prioridade i (quanto mais próximo de 0 maior)
		NVIC_EnableIRQ(button.pio_id);
		NVIC_SetPriority(button.pio_id, 4); // Prioridade 4
	}
}

void configureLeds(component leds[], int size){

	for (int i = 0; i < size; i++)
	{
		component led = leds[i];
		pio_configure(led.pio, led.initialState, led.mask, PIO_DEFAULT);
	}
}

void enebleAllPeriph(int periphIdsList[], int size){
	for (int i = 0; i <size; i++)
	{
		pmc_enable_periph_clk(periphIdsList[i]);
	}
}

void pin_toggle(component led){
	if(pio_get_output_data_status(led.pio, led.mask))
	pio_clear(led.pio, led.mask);
	else
	pio_set(led.pio,led.mask);
}

void pisca_led(int n, int t, component led){
	for (int i=0;i<n;i++){
		pin_toggle(led);
		delay_ms(t);
		pin_toggle(led);
		delay_ms(t);
	}
}

void updateTime(void){
	seconds += 1;
	if (seconds == 10){
		seconds = 0;
		secondsDozen += 1;
		if (secondsDozen == 6){
			secondsDozen = 0;
			minutes += 1;
			if (minutes == 10){
				minutes = 0;
				minutesDozen += 1;
				if (minutesDozen == 6){
					minutesDozen = 0;
					hours += 1;
					if (hours == 10){
						hours = 0;
						hoursDozen += 1;
						if (hoursDozen == 24)
						{
							hoursDozen = 0;
						}
					}
				}
			}
		}
	}
	lcdFlag = 1;
}

void writeLCD(void){
	sprintf(string, "%d%d:%d%d:%d%d", hoursDozen, hours, minutesDozen, minutes, secondsDozen, seconds);
	gfx_mono_draw_string(string, 0, 0, &sysfont);
}

/************************************************************************/
/* INITS                                                                */
/************************************************************************/
/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, uint32_t irq_type){
	calendar t = {2018, 3, 19, 12, 15, 45, 1};
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
	
		/* configura alarme do RTC */
	//rtc_set_date_alarm(RTC, 1, t.month, 1, t.day);
	rtc_set_time_alarm(RTC, 1, t.hour, 1, t.minute, 1, t.seccond);
}

static void RTT_init()//uint16_t pllPreScale, uint32_t IrqNPulses
{
	/*
	* IRQ apos Xs -> IrqNPulses*0.5
	*/
	uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
	uint32_t IrqNPulses = 10;
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	
	//Desabilita flag do alarme do RTT
	rttAlarm = 0;
}


void init(void){
	board_init();
	sysclk_init();
	
	int periphIdsList[4] = {
		ID_PIOA,
		ID_PIOB,
		ID_PIOC,
		ID_PIOD
	};
	enebleAllPeriph(periphIdsList, 4);
	
	//Configura os LEDs
	component leds[3] = {led1,led2,led3};
	configureLeds(leds, 3);

	//Configura os botões
	component buttons[3] = {leftButton,centerButton,rightButton};
	configureButtons(buttons, 3);
	
	/** Configura timer TC0, canal 1 com 5Hz */
	TC_init(TC0, ID_TC0, 0, 5);
	/** Configura timer TC0, canal 2 com 10Hz */
	TC_init(TC0, ID_TC1, 1, 10);
	/** Configura timer TC0, canal 3 com 1Hz */
	TC_init(TC0, ID_TC2, 2, 1);
	
	/** Configura RTC */
	RTC_init(RTC, ID_RTC, RTC_IER_ALREN | RTC_IER_SECEN);

	// Init OLED
	gfx_mono_ssd1306_init();
	delay_init();
	sprintf(string, "        ");
	
}

int main (void)
{
	init();
	RTT_init();

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (lcdFlag)
		{
			writeLCD();
			lcdFlag = 0;
		}
		if (rttAlarm){
		  RTT_init();
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
