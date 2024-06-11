#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//[-------------Global---------------]
#define RS 7
#define E 6
volatile uint32_t ms_ticks = 100;
volatile uint32_t encoder_count_speed = 0;
volatile int encoder_count_turnovers = 0;
volatile uint8_t motor_direction = 0;

//[-------------Prototyp---------------]
void lcd_init(void);
void timer0_init(void);
void global_interrupts_init(void);
void pwm_init(void);
void lcd_cmd(uint8_t cmd);
void lcd_data(char data);
void lcd_clear(void);
void lcd_display_time(uint32_t sec);
void lcd_pwm(int pwm_duty);
void lcd_speed_v2(double speed);
void lcd_coun(int encoder_count);


//[----Global_counting_encoder_signals----]
ISR(TIMER0_COMP_vect) {
	ms_ticks++;
}

ISR(INT0_vect) {
	encoder_count_speed++;
}

ISR(INT1_vect) {
	if (motor_direction == 1) {
		encoder_count_turnovers++;
	}
	else if (motor_direction == 2) {
		encoder_count_turnovers--;
	}
	lcd_coun(encoder_count_turnovers);
}

int main(void) {
	
	lcd_init();
	timer0_init();
	global_interrupts_init();
	pwm_init();
	
	char up_word[] = "Up  ";
	char down_word[] = "Down";
	char stop_word[] = "Stop";

	
	DDRD &= ~(1 << PD2);
	DDRD &= ~(1 << PD3);
	DDRD |= (1 << PD4) | (1 << PD5);
	
	PORTC = 0x0F;

	double speed = 0;
	double target_speed_const = 50.0;
	double target_speed = 0;
	double error = 0;
	double integral = 0;
	double Kp = 0.26;
	double Ki = 0.0003;
	uint32_t last_ticks = 0;
	uint32_t last_encoder_count_speed = 0;
	uint32_t ticks_diff = 0;
	uint32_t encoder_diff_speed = 0;
	int pwm_duty = 0;
	int floor = -1;
	int target_count, delta_count;
	
	while (1) {
		//[-------------Checking_Button_Floor---------------]
		floor = -1;
		if (!(PINC & (1 << PINC0))) {
			floor = 0;
		}
		else if (!(PINC & (1 << PINC1))) {
			floor = 1;
		}
		else if (!(PINC & (1 << PINC2))) {
			floor = 2;
		}
		else if (!(PINC & (1 << PINC3))) {
			floor = 3;
		}
		
		//[-------------Move_to_Floor---------------]
		if (floor != -1) {
			target_count = floor * 10;
			delta_count = target_count - encoder_count_turnovers;
			
			//[-------------Move_Up---------------]
			if (delta_count > 0) {
				motor_direction = 1;
				
				lcd_cmd((1 << 7) | 0);
				for (uint8_t i = 0; i < sizeof(up_word); i++) {
					lcd_data(up_word[i]);
				}

				while (encoder_count_turnovers < target_count - 6) {
					// update the speed every 5 ms
					if ((ms_ticks - last_ticks) >= 5) {
						//gradual increase in speed
						if (target_speed < target_speed_const - 5) {
							target_speed += 5.0;
						}
						else {
							target_speed = target_speed_const;
						}
						//speed calculation
						ticks_diff = ms_ticks - last_ticks;
						encoder_diff_speed = encoder_count_speed - last_encoder_count_speed;

						speed = (double)encoder_diff_speed / (double)ticks_diff * 5.0;

						last_ticks = ms_ticks;
						last_encoder_count_speed = encoder_count_speed;

						// PI
						error = target_speed - speed;
						integral += error;
						double control_signal = Kp * error + Ki * integral;

						pwm_duty += (int)control_signal;

						if (pwm_duty > 254) {
							pwm_duty = 254;
						}
						else if (pwm_duty < 0) {
							pwm_duty = 0;
						}

						OCR1B = pwm_duty;
					}
				}
				while (encoder_count_turnovers < target_count) { 
					// update the speed every 5 ms
					if ((ms_ticks - last_ticks) >= 5) {
						//gradually decreasing speed
						if (target_speed > 10) {
							target_speed -= 4.0;
						}
						else {
							target_speed = 10;
						}
						//speed calculation
						ticks_diff = ms_ticks - last_ticks;
						encoder_diff_speed = encoder_count_speed - last_encoder_count_speed;

						speed = (double)encoder_diff_speed / (double)ticks_diff * 10.0;

						last_ticks = ms_ticks;
						last_encoder_count_speed = encoder_count_speed;

						// PI controller
						error = target_speed - speed;
						integral += error;
						double control_signal = Kp * error + Ki * integral;

						pwm_duty += (int)control_signal;

						if (pwm_duty > 254) {
							pwm_duty = 254;
						}
						else if (pwm_duty < 0) {
							pwm_duty = 0;
						}

						OCR1B = pwm_duty;
					}
				}
				
				OCR1B = 0;
			}
			//[-------------Move_Down---------------]
			else if (delta_count < 0) {
				motor_direction = 2;
				
				lcd_cmd((1 << 7) | 0);
				for (uint8_t i = 0; i < sizeof(down_word); i++) {
					lcd_data(down_word[i]);
				}

				while (encoder_count_turnovers > target_count + 6) {
					// update the speed every 5 ms
					if ((ms_ticks - last_ticks) >= 5) {
						//gradual increase in speed
						if (target_speed < target_speed_const - 5) {
							target_speed += 5.0;
						}
						else {
							target_speed = target_speed_const;
						}
						//speed calculation
						ticks_diff = ms_ticks - last_ticks;
						encoder_diff_speed = encoder_count_speed - last_encoder_count_speed;

						speed = (double)encoder_diff_speed / (double)ticks_diff * 10.0;

						last_ticks = ms_ticks;
						last_encoder_count_speed = encoder_count_speed;

						// PI controller
						error = target_speed - speed;
						integral += error;
						double control_signal = Kp * error + Ki * integral;

						pwm_duty += (int)control_signal;

						if (pwm_duty > 254) {
							pwm_duty = 254;
						}
						else if (pwm_duty < 0) {
							pwm_duty = 0;
						}

						OCR1A = pwm_duty;
					}
				}
				while (encoder_count_turnovers > target_count) { 
					// update the speed every 5 ms
					if ((ms_ticks - last_ticks) >= 5) {
						//gradually decreasing speed
						if (target_speed > 10) {
							target_speed -= 4.0;
						}
						else {
							target_speed = 10;
						}
						//speed calculation
						ticks_diff = ms_ticks - last_ticks;
						encoder_diff_speed = encoder_count_speed - last_encoder_count_speed;

						speed = (double)encoder_diff_speed / (double)ticks_diff * 10.0;

						last_ticks = ms_ticks;
						last_encoder_count_speed = encoder_count_speed;

						// PI controller
						error = target_speed - speed;
						integral += error;
						double control_signal = Kp * error + Ki * integral;

						
						pwm_duty += (int)control_signal;

						if (pwm_duty > 254) {
							pwm_duty = 254;
						}
						else if (pwm_duty < 0) {
							pwm_duty = 0;
						}

						OCR1A = pwm_duty;
					}
				}
				
				OCR1A = 0;
			}
		}


		//[-------------Stop---------------]
		motor_direction = 0;
		_delay_ms(1000);
		lcd_cmd(1);
		lcd_cmd((1 << 7) | 0);
		
		for (uint8_t i = 0; i < sizeof(stop_word); i++) {
			lcd_data(stop_word[i]);
		}
		_delay_ms(1000);
		lcd_coun(encoder_count_turnovers);
	}
}

//[------------Function-----------]


void lcd_init(void){
	DDRB = 0xFF;
	DDRA |= ((1 << E)|(1 << RS));
	_delay_ms(100);
	lcd_cmd(0x30);
	lcd_cmd(0x30);
	lcd_cmd(0x30);
	lcd_cmd(0x38);
	lcd_cmd(0x0E);
	lcd_cmd(0x06);
	lcd_cmd(0x01);
}

void lcd_cmd(uint8_t cmd){
	DDRB = 0xFF;
	DDRA |= ((1 << E)|(1 << RS));
	PORTA &= ~(1<<RS);
	PORTB = cmd;
	PORTA |= (1 << E);
	_delay_us(5);
	PORTA &= ~(1 << E);
	_delay_ms(100);
}


void lcd_data(char data){
	DDRB = 0xFF;
	DDRA |= ((1 << E)|(1 << RS));
	PORTA |= (1 << RS);
	PORTB = data;
	PORTA |= (1 << E);
	_delay_us(5);
	PORTA &= ~(1 << E);
	_delay_ms(1);
}

void timer0_init(void) {
	TCCR0 |= (1 << WGM01); 
	OCR0 = 124; // Compare value for 1 ms at 1 MHz and prescaler 1024
	TIMSK |= (1 << OCIE0); 
	TCCR0 |= (1 << CS02) | (1 << CS00); 
	sei(); // Enable global interrupts
}

void pwm_init(void) {
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11);
	DDRD |= (1 << PD4) | (1 << PD5);
}

void global_interrupts_init(void) {
	MCUCR |= (1 << ISC01) | (1 << ISC11); 
	GICR |= (1 << INT0) | (1 << INT1); 
	sei(); // Enable global interrupts
}

void lcd_coun(int encoder_count){
	
	char str[5];
	itoa(encoder_count, str, 10);
	
	lcd_cmd(0x80 | 0x40);
	lcd_data('N');
	lcd_data('o');
	lcd_data('R');
	lcd_data(':');
	lcd_data('0' + ((encoder_count / 1000) % 10));
	lcd_data('0' + ((encoder_count / 100) % 10));
	lcd_data('0' + ((encoder_count / 10) % 10));
	lcd_data('0' + (encoder_count % 10));
}

void lcd_display_time(uint32_t sec) {
	uint32_t hours = sec / 3600;
	sec %= 3600;
	uint32_t minutes = sec / 60;
	uint32_t seconds = sec % 60;

	lcd_cmd(0x80 | 0x40);
	lcd_data('0' + (hours / 10));
	lcd_data('0' + (hours % 10));
	lcd_data(':');
	lcd_data('0' + (minutes / 10));
	lcd_data('0' + (minutes % 10));
	lcd_data(':');
	lcd_data('0' + (seconds / 10));
	lcd_data('0' + (seconds % 10));
}

void lcd_pwm(int pwm_duty) {
	char str[5];
	itoa(pwm_duty, str, 10);
	lcd_cmd(0x80 | 0x40);
	lcd_data('P');
	lcd_data('W');
	lcd_data('M');
	lcd_data(':');
	lcd_data('0' + ((pwm_duty / 1000) % 10));
	lcd_data('0' + ((pwm_duty / 100) % 10));
	lcd_data('0' + ((pwm_duty / 10) % 10));
	lcd_data('0' + (pwm_duty % 10));
}

void lcd_speed_v2(double speed) {
	char str[10];

	dtostrf(speed, 7, 3, str);

	lcd_cmd(0x80 | 0x00);
	lcd_data('S');
	lcd_data('P');
	lcd_data('D');
	lcd_data(':');
	lcd_data(' ');

	for (int i = 0; i < 10 && str[i] != '\0'; i++) {
		lcd_data(str[i]);
	}
}

