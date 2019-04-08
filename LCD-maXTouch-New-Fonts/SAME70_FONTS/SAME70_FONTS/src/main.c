/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define YEAR        2019
#define MOUNTH      4
#define DAY         8
#define WEEK        12
#define HOUR        0
#define MINUTE      0
#define SECOND      0

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

#define pi 3.14
#define raio 0.65
/**
* Botao
*/
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX  11
#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT_DEBOUNCING_VALUE  79

/* variaveis globais                                                    */
/************************************************************************/
volatile Bool f_rtt_alarme = false;

volatile Bool but_flag;
char buffert = [32];
char bufferv[32];
char bufferd[32];

int pulso = 0;
int vel = 0 ;
int dist=0;



/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_iSnit(uint16_t pllPreScale, uint32_t IrqNPulses);
struct ili9488_opt_t g_ili9488_display_opt;
void RTC_init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/
void but_callback(void){
	but_flag = true;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
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
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			volatile uint32_t pul_hour;
			volatile uint32_t pul_minute;
			volatile uint32_t pul_second;
			rtc_get_time(RTC, &pul_hour, &pul_minute, &pul_second);
			rtc_set_time_alarm(RTC, 1, pul_hour, 1, pul_minute+1, 0, pul_second);
			/// MUDAR HORARIO
			
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
/// RTT
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	/* BOTAO */
	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);

	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
}
	


static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
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
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}


//// FONTS
void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}


int main(void) {
	// Desliga watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	
	board_init();
	sysclk_init();	
	configure_lcd();
	io_init();
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;

	while(1) {
		
		if(but_flag){
			
			pulso+=1;
			// zera flag
			but_flag = false;
				
		}
		
		if (f_rtt_alarme){
      
		  uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
		  uint32_t irqRTTvalue  = 4;
		  
		  vel = (2*pi *pulso)/irqRTTvalue;
		  dist += 2*pi* raio* pulso;
		  
		  sprintf(bufferv,"%02d",vel);
		  sprintf(bufferd,"%02d",dist);
		  //sprintf(bufferd,"%d: %d: %d: ",);
		  //font_draw_text(&sourcecodepro_28, "bike", 50, 50, 1);
		  //font_draw_text(&calibri_36, "Total time:", 50, 50, 1);
		  font_draw_text(&arial_72, bufferd, 50, 50, 1);
		  font_draw_text(&calibri_36, "Velocidade:", 50, 100, 1);
		  font_draw_text(&arial_72, bufferv, 50, 150, 1);
		  font_draw_text(&calibri_36, "Distancia:", 50, 250, 1);
		  font_draw_text(&arial_72, bufferd, 50, 300, 1);
		  
		  
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);         
      
		 /*
		  * caso queira ler o valor atual do RTT, basta usar a funcao
		  *   rtt_read_timer_value()
		  */
      
		  /*
		   * CLEAR FLAG
		   */
		  f_rtt_alarme = false;
		  pulso = 0;
    }
		
		
		
		
		
	}
}