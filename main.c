#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF       // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


#include <xc.h>
#include <pic16f627a.h>
#include <math.h>
#include <stdbool.h>

// DEFINITION FOR CONVENIENCE
#define BUTTON1 PORTAbits.RA0
#define Pause_b PORTBbits.RB0
#define PLAY PORTAbits.RA1
#define IDLE PORTAbits.RA7
#define LED PORTBbits.RB5
#define Volup_b PORTBbits.RB6
#define Voldown_b PORTBbits.RB4
// DUTY CYCLE FOR MOTOR
#define Percent 54

// CLOCK SPEED
#define _XTAL_FREQ 16000000

const unsigned char pause_cmd[] = {0x7E, 0xFF, 0x06, 0x0E, 0x00, 0x00, 0x00, 0xEF};
const unsigned char volup_cmd[] = {0x7E, 0xFF, 0x06, 0x04, 0x00, 0x00, 0x00, 0xEF};
const unsigned char voldown_cmd[] = {0x7E, 0xFF, 0x06, 0x05, 0x00, 0x00, 0x00, 0xEF};
const unsigned char volset_cmd[] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x00, 0xEF};

bool checkbit(uint16_t data, int position){
    return data & (1 << position);
}

void init(){
        // Configure the comparator
    CMCONbits.CM = 0b111; // Turn off the comparator
    CMCONbits.CIS = 0; // Set the input to C1Vin-
    CMCONbits.C1INV = 0; // Output polarity is non-inverted
    CMCONbits.C2INV = 0; // Output polarity is non-inverted
    
    //UART Init
    TRISB1 = 1;   // RB5/RX pin configured as input
    TRISB2 = 0;   // RB7/TX pin configured as output
    RCSTAbits.SPEN = 1;     // Serial port enabled

    TXSTAbits.SYNC = 0;     // Asynchronous mode
    TXSTAbits.BRGH = 1;     // High-speed baud rate

    TXSTAbits.TXEN = 1;     // Transmit enabled
    RCSTAbits.CREN = 0;     // Receive disabled

    
    SPBRG = 25;    // Set baud rate ~ 9600
    
    //pwm config
    
    PR2 = (uint8_t)((1/(4*_XTAL_FREQ*16))-1); //period of 1 sec
    uint16_t duty_cycle = (uint16_t) (1/Percent);
    CCPR1L = duty_cycle >> 2; //set high bits of duty cycle
    //set low bits of duty cycle
    if (checkbit(duty_cycle, 0)){
        CCP1CONbits.CCP1Y = 1;
    };
    if (checkbit(duty_cycle, 1)){
        CCP1CONbits.CCP1X = 1;
    };
    TRISB3 = 0; //clear bit to enable output
    T2CON = 0b00000110;  //enable timer 2 + prescaler value 16
    
    
    // Set IO pins
    TRISA0 = 1;
    TRISA1 = 0;
    TRISA7 = 1;
    TRISB0 = 1;
    TRISB5 = 0;
    TRISB6 = 1;
    TRISB4 = 1;
    return;
}

void motor_switch(int x){
    if (x){
        CCP1CONbits.CCP1M = 0b1100;
    }
    else {
        CCP1CONbits.CCP1M = 0b0000;
    };
}
void UART_transmit(unsigned char CMD[8], unsigned char feedback, unsigned char para1, unsigned char para2){
    
    unsigned char cmd[8] = {0};
    for (int i = 0; i < 8; i++){
        cmd[i] = CMD[i];
    };
    
    //fill in parameters
    cmd[4] = feedback;
    cmd[5] = para1;
    cmd[6] = para2;
    
    for (int i = 0; i < 8; i++) {
        TXREG = cmd[i]; // Load data into the TX register
        __delay_ms(5);
    };
};

void Flash(){
    LED = 1;
    __delay_ms(5);
    LED = 0;
}

void main(void) {
    
    init();
    for (int i = 1; i < 5; i++){
        UART_transmit(volset_cmd, 0x00, 0x00, 0x14);
    };
    
    while(1){
        if (BUTTON1){
            while(BUTTON1);
            motor_switch(1);
            __delay_ms(700);
            PLAY = 1;
            __delay_ms(5);
            PLAY = 0;
            __delay_ms(500);
            while(!IDLE){
                if (Pause_b){
                    while(Pause_b);
                    UART_transmit(pause_cmd, 0x00, 0x00, 0x00);
                    __delay_ms(500);
                    motor_switch(0);
                    Flash();
                };
                if (Volup_b){
                    while(Volup_b);
                    UART_transmit(volup_cmd, 0x00, 0x00, 0x00);
                    //UART_transmit(volup_cmd, 0x00, 0x00, 0x00);
                    Flash();
                };
                if (Voldown_b){
                    while(Voldown_b);
                    UART_transmit(voldown_cmd, 0x00, 0x00, 0x00);
                    Flash();
                };
                
            };
             __delay_ms(500);
            motor_switch(0);
        };
        
        
        

    }
    return;
}
