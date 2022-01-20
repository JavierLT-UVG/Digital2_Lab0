/*
 * Archivo:         Prelab8.c
 * Dispositivo:     PIC16F887
 * Autor:           Francisco Javier López
 *
 * Programa:        Carrera entre dos jugadores, con una cuenta atrás 
 *                  representada por un semáforo y un display, donde cada 
 *                  jugador tiene un botón que controla un contador de décadas, 
 *                  al jugador que gana se le enciende una led de indicación
 * 
 * Hardware:        8 Leds en Puerto A, 8 Leds en Puerto D, 3 Leds en Puerto E, 
 *                  Display en Puerto C, 2 Leds y 3 Botones en Puerto B
 * 
 * Creado: 17 de enero de 2022
 * Última Modificación: 19 de enero de 2022
 */


// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000

// Variables
uint8_t iocbEnable = 1;         // Variable que actúa como enable de los botones
uint8_t timerEnable = 0;        // Variable que actúa como enable de los botones
uint8_t contador = 3;           // Variable que funciona como contador de 3 a 0

// Prototipos de funciones
void config_io(void);
void config_reloj(void);
void config_int(void);
void config_wpub(void);
void config_iocb(void);
void config_tmr1(void);
void semaforo(uint8_t num);
uint8_t display(uint8_t valor);

// Interrupciones
void __interrupt() isr (void)
{   
    if(PIR1bits.TMR1IF)         // Si la bandera está encendida, entrar
    {
        if(contador>=0)         // Si contador es mayor o igual a 0, decrementar
        {
            contador--;
        }
        PIR1bits.TMR1IF = 0;    // Limpiar bandera de overflow
    }
    if(INTCONbits.RBIF)
    {
        if(!PORTBbits.RB5)
        {
            iocbEnable = 1;         // Encender interrupciones para los botones
            PORTA = 0;
            PORTD = 0;              // Limpiar los puertos de los jugadores
            PORTBbits.RB0 = 0;
            PORTBbits.RB1 = 0;      // Limpiar los botones de alarma de ganador
            contador = 3;           // Restaurar variable del contador inicial
        }
        if(!PORTBbits.RB6 && iocbEnable && timerEnable)
        {
            if(!PORTA)
            {
                PORTA++;            // Incrementar en 1 PORTA si está en 0
            }
            else
            {
                PORTA = PORTA<<1;   // Left shift si PORTA es distinto a 0
            }
        }
        if(!PORTBbits.RB7 && iocbEnable && timerEnable)
        {
            if(!PORTD)
            {
                PORTD++;            // Incrementar en 1 PORTA si está en 0
            }
            else
            {
                PORTD = PORTD<<1;   // Left shift si PORTA es distinto a 0
            }
        }
        INTCONbits.RBIF = 0;        // Limpiar bandera de overflow
    }
}

// Main
void main(void) 
{
    config_io();
    config_reloj();
    config_int();
    config_wpub();
    config_iocb();
    config_tmr1();
    
    while(1)
    {
        if(PORTAbits.RA7)           // Si ya terminó el jugador 1
        {
            PORTBbits.RB0 = 1;      // Encender led de ganador
            iocbEnable = 0;         // Desactivar botones
        }
        
        if(PORTDbits.RD7)           // Si ya terminó el jugador 2
        {
            PORTBbits.RB1 = 1;      // Encender led de ganador
            iocbEnable = 0;         // Desactivar botones
        }
        
        if(contador < 4 && contador != 0)   // Si contador está entre 3 y 1, deshabilitar
        {
            timerEnable = 0;
        }
        else                                // Si contador llega a 0, habilitar botones
        {
            timerEnable = 1;
        }
        
        semaforo(contador);         // Activar semáforo basándose en contador
        PORTC = display(contador);  // Codificar al display el tiempo de contador
    }
}

// Funciones
void config_io(void)            // Configuración de entradas y salidas
{
    ANSEL   =   0;              // Puertos digitales
    ANSELH  =   0;
    
    TRISA   =   0;              // Puertos como salida
    TRISB   =   0b11100000;     // Excepto los 3 botones del puerto B
    TRISC   =   0;
    TRISD   =   0;
    TRISE   =   0;
    
    PORTA   =   0;              // Limpiar los puertos
    PORTB   =   0;
    PORTC   =   0;
    PORTD   =   0;
    PORTE   =   0;
    return;
}

void config_reloj(void)         // Configuración del oscilador
{
    OSCCONbits.IRCF2 = 1;       // 1MHz
    OSCCONbits.IRCF1 = 0;       // 
    OSCCONbits.IRCF0 = 0;       // 
    OSCCONbits.SCS = 1;         // Reloj interno
    return;
}

void config_int(void)           // Configuración de interrupciones
{
    INTCONbits.GIE  = 1;        // Activar interrupciones
    INTCONbits.PEIE = 1;        // Activar interrupciones periféricas
    INTCONbits.RBIE = 1;        // Activar interrupciones de PuertoB
    INTCONbits.RBIF = 0;        // Apagar bandera de overflow de PuertoB
    PIE1bits.TMR1IE = 1;        // Activar interrupción de Timer1
    PIR1bits.TMR1IF = 0;        // Apagar bandera de overflow de Timer1
    return;
}

void config_wpub(void)
{
    OPTION_REGbits.nRBPU = 0;   // Encender configuración de weak pullups
    WPUB    =   0b11100000;     // Encender 3 pullups del puerto b (botones)
    return;
}

void config_iocb(void)
{
    IOCB    =   0b11100000;     //Encender interrupt on change de los pullups
    return;
}

void config_tmr1(void)          // Timer1 a 1 segundo
{
    T1CONbits.T1CKPS1 = 1;      // bits 5-4  Prescaler Rate Select bits
    T1CONbits.T1CKPS0 = 0;      // bit 4
    T1CONbits.T1OSCEN = 0;      // bit 3 Timer1 Oscillator Enable Control = on
    T1CONbits.T1SYNC = 1;       // bit 2 Do not synchronize external clock input
    T1CONbits.TMR1CS = 0;       // bit 1 Internal clock (FOSC/4)
    T1CONbits.TMR1ON = 1;       // bit 0 enables timer
    TMR1H = 11;                 // preset for timer1 MSB register
    TMR1L = 220;                // preset for timer1 LSB register
    return;
}

void semaforo(uint8_t num)      // Función que enciende los leds del semáforo
{
    switch(num)
    {
        case 3:
            PORTEbits.RE0 = 1;  // Encender luz roja
            break;
        case 2:
            PORTEbits.RE1 = 1;  // Encender luz amarilla
            break; 
        case 1:
            PORTEbits.RE2 = 1;  // Encender luz verde
            break;    
        default:
            PORTEbits.RE0 = 0;  // Apagar las 3 luces
            PORTEbits.RE1 = 0;
            PORTEbits.RE2 = 0;
            break;
    }
    return;
}

uint8_t display(uint8_t valor)    // Traduce números de 0-3 a valores de display
{
    switch(valor)
    {
        case 0:
            return 0b00111111;
            break;
        case 1:
            return 0b00000110;
            break;
        case 2:
            return 0b01011011;
            break;
        case 3:
            return 0b01001111;
            break;
        default:
            return 0b00000000;  // Default es apagado
            break;
    }
}