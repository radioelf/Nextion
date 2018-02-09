/* Control de pantalla Nextion para mostrar datos recibidos por nodos CAN-bus

nodo consumo 0x0A envía los siguientes datos:
	1- tensión de red
	2- intensidad circuito luces
	3- intensidad circuito aire acondicionado
	4- intensidad circuito lavadora/termo
	5- intensidad circuito enchufes
	6- intensidad circuito cocina

nodo temperatura/humedad 1 0x0C envía los siguientes datos:
    1- temperatura en grados de la habitación de 0 a 50 grados
    2- temperatura en grados del cuartillo de 0 a 50 grados
    3- temperatura en grados del pasillo de 0 a 50 grados
    4- temperatura en grados del aseo de 0 a 50 grados
    5- humedad ambiental media de la habitación 20-95%
    6- humedad ambiental media del cuartillo 20-95%
    7- humedad ambiental media del pasillo 20-95%
    8- humedad ambiental media del aseo 20-95%

nodo temperatura/humedad 2 0x0E envía los siguientes datos:
    1- temperatura en grados del comedor de 0 a 50 grados
    2- temperatura en grados de la cocina de 0 a 50 grados
    3- temperatura en grados de la habitación plancha de 0 a 50 grados
    4- temperatura en grados del  taller de 0 a 50 grados
    5- humedad ambiental media del comedor  20-95%
    6- humedad ambiental media de la cocina  20-95%
    7- humedad ambiental media de la habitación plancha  20-95%
    8- humedad ambiental media del taller  20-95%

 Direcciones para nodo de estación meteorológica:
 dirección 0x2A opciones y 0x2B eco (NO usado)
 dirección 0x2C->petición datos del  bloque 1, float1 bytes del 0 al 3
                     temperatura 1 DHT22, float2 bytes de 4 al 7 temperatura interna BMP085
 dirección 0x2D->petición datos del  bloque 2, anemómetro byte 0 y 1,
                     pulsos lluvia 1 minuto byte 2, pulsos lluvia 1 hora bytes 3 y 4,
                     dirección viento  bytes 5 y 6, tensión VCC byte 7
 dirección 0x2E->petición datos del  bloque 3, presión bytes 0 y 3,
                     radiación solar byte 4 y 5, radiación ultravioleta bytes 6,
                     humedad  byte 7
 cada pulso de lluvia corresponde a 0.25mm
 el nodo de la estación meteorológica envió los float con el formato IEEE
 para usar float con 32 bits es necesario modificar las opciones del compilador
 XC8 linker->memory model->32

 nodo Int_out  0x20, recibe la orden para controlar 4 reles y reponde con el estado de 4 entradas
    1- byte ->bits de 0 a 3 salidas, 4 a 7 entradas
 *
 *
 * File:   main.c
 * Author: Radioelf
 * PIC18F2685
 * última modificación el 9 del 2 del 2018, 16:00  v0.0.3
 *
 */

// CONFIG1H
#pragma config OSC = HSPLL                                                      // Oscillator Selection bits xt 8Mhz X 4 = 32MHz
//#pragma config OSC = HS
#pragma config FCMEN = OFF                                                      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF                                                       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF                                                       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = BOHW                                                     // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3                                                         // Brown-out Reset Voltage bits (VBOR set to 2.1V)

// CONFIG2H
#pragma config WDT = OFF                                                         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 131072                                                   // 4s. Watchdog Timer Postscale Select bits (1:16384) +-4 ms to 131.072 seconds

// CONFIG3H
#pragma config PBADEN = OFF                                                     // PORTB A/D OFF bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF                                                    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON                                                       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON                                                      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF                                                        // desactivar el modo Low-Voltage-Programing ICSP
#pragma config BBSIZ = 1024                                                     // Boot Block Size Select bits (1K words (2K bytes) Boot Block)
#pragma config XINST = OFF                                                      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF                                                        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF                                                        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF                                                        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF                                                        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF                                                        // Code Protection bit (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF                                                        // Code Protection bit (Block 5 (014000-017FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF                                                        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF                                                        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF                                                       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF                                                       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF                                                       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF                                                       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF                                                       // Write Protection bit (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF                                                       // Write Protection bit (Block 5 (014000-017FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF                                                       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF                                                       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF                                                       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF                                                      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF                                                      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF                                                      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF                                                      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF                                                      // Table Read Protection bit (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF                                                      // Table Read Protection bit (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF                                                      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

#define _XTAL_FREQ 32000000                                                     // XT 8 Mhz x 4 PLL

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#include "library/usart.h"
#include "library/nextion.h"
#include "library/ecan.h"
#include "library/i2c.h"
#include "library/ds1337.h"

// portA
#define Clock PORTAbits.RA0                                                     // Int, petición pantalla reloj (arduino D4)
#define Nextion_ON PORTAbits.RA1                                                // Int, petición ON pantalla (arduino D5)
#define Nextion_OFF PORTAbits.RA2                                               // Int, petición OFF pantalla (arduino D6)
#define Temp_ON PORTAbits.RA3                                                   // Int, petición OFF alarma consumo (arduino D7)
// RA3                                                                          Out, no usado
// RA4                                                                          Out, no usado, (colector abierto)
// RA5                                                                          Out, no usado
// RA6 -> OSC2 (8Mhz)
// RA7 -> OSC1 (8Mhz)
// portB
#define Led_status LATBbits.LATB0                                               // Out, Led estado
#define Pulsador PORTBbits.RB1                                                  // Int, pulsador
// RB2 -> Can_TX                                                                Out
// RB3 -> Can RX                                                                Int
#define Zumbador LATBbits.LATB4                                                 // out, indicación alarma
// RB5                                                                          Out, no usado
// RB6 -> PGC                                                                   Out, program
// RB7 -> PGD                                                                   Out, program
// portC
// RC0                                                                          Out, no usado
// RC1                                                                          Out, no usado
// RC2                                                                          Out, no usado
// RC3 -> SCL                                                                   Int, I2C
// RC4 -> SDA                                                                   Int, I2C
// RC5                                                                          Out, no usado
// RC6 -> TX                                                                    Out, RS232
// RC7 -> RX                                                                    Int, RS232

unsigned char actualiza =0, RxCanFlag =0, RX_uart =0, sleep =0, fecha_act =0,
              ciclo_max_min =0, max_temp_int =0, min_temp_int =0;               // 0-255
signed char gra_tem_ext [446] ={0}, max_temp_ext =0, min_temp_ext =0;           //+1=1...+127=127, 0, -1=255, -2=254....-127=129, -128=128
unsigned char hora_max_int[6] ={0}, hora_min_int[6] ={0}, hora_max_ext[6] ={0},
              hora_min_ext[6] ={0};
unsigned char gra_tem_int [446] ={0}, gra_potencia [446]={0}, gra_presion [446]={0};// 446 pixeles de la gráfica, coordenadas x
unsigned char trama_ascii[10]={0};
unsigned short posicion =0;                                                     // 0-65535
//unsigned long                                                                 // 0-4294967295
//float                                                                         // 4 bytes

__EEPROM_DATA(0x14,0x05,0x76,0x16,0x00,0xff,0xff,0xff);                         // posición 1->siguiente p. ,2->sleep  , 3 y 4 alarma consumo (5750W)

//*****************************ISR**********************************************
void interrupt CAN_INTx_Timer0_uart_isr(void){                                  // CAN Rx, TMR0, UART
    if ((PIE3bits.RXB0IE && PIR3bits.RXB0IF) || (PIE3bits.RXB1IE && PIR3bits.RXB1IF)){// CAN recepción mode 0
        ECANCON=(ECANCON&0b00000)|(0b10000|(CANCON&0x0F));                      // banderas RX datos Can-Bus
        if (RXB0CONbits.RXFUL) RxCanFlag = 1;
        if (RXB1CONbits.RXFUL) RxCanFlag = 2;
        if (B0CONbits.RXFUL) RxCanFlag = 3;
        if (PIR3bits.RXB1IF) PIR3bits.RXB1IF = 0;                               // mode 0
        if (PIR3bits.RXB0IF) PIR3bits.RXB0IF = 0;                               // mode 0
        if (PIR3bits.RXBnIF) PIR3bits.RXBnIF = 0;                               // mode 1 ,2
    }
    if (T0IE && T0IF){                                                          // banderas de interrupción TMR0
        actualiza =1;
        T0IF=0;                                                                 // TMR0 borramos bandera
    }
    if(PIE1bits.RCIE && PIR1bits.RCIF){                                         // comprobamos si tenemos una interrupción por RX datos en UART
        if(OERR){                                                               // si tenemos error
            RX_uart =RCREG;
            RX_uart =RCREG;
            CREN = 0;
            CREN = 1;
        }else{
            RX_uart =RCREG;                                                     // obtenemos el dato
            if(RX_off !=3){                                                     // comprobamos fin de la trama (0xff 0xff 0xff)
                 if (RX_uart ==0xff){
                    RX_off++;
                }else{
                    RX_off =0;
                }
                RX_buffer[RX_con++] =RX_uart;
                if (RX_con ==10) RX_con =9;                                     // impedimos que desborde
            }
       }
       RCIF = 0;                                                                // borramos bandera rx
    }
}
//*****************************CONFIGURACIÓN************************************
void config(void){
unsigned char espera =25;
     // Config. PORTs I/O
    TRISA = 0b10001111;                                                         // 1->Input, 0->Output
    PORTA = 0x00;
    TRISB = 0b00001010;                                                         // 1->Input, 0->OutpuT
    PORTB = 0x00;
    TRISC = 0b10011000;                                                         // 1->Input, 0->Output
    PORTC = 0x00;
    ADCON0bits.ADON = 0;                                                        // OFF pines analógicos (config PBADEN = OFF))
    ADCON1 = 0b00001111;                                                        // RAx Int/Out and Vref- Vss, and Vref+ Vdd
    // Config ISR INT0 OFF
    RCONbits.IPEN = 0;                                                          // Disable priorities
    //INTCON = 0b10110000;
    INTCONbits.PEIE_GIEL = 1;                                                   // Enable peripheral interrupts
    //INTCON2bits.INTEDG0 = 1;                                                  // Interrupción INT0 se activa en flanco
    INTCONbits.INT0IE = 0;                                                      // interrupción INT0 deshabilitada
    INTCONbits.INT0IF = 0;                                                      // clear flah bit for INT0
    INTCON3bits.INT1IE = 0;                                                     // interrupción INT1 deshabilitada
    //INTCON3bits.INT1IP = 0;                                                   // Low priority
    INTCON3bits.INT1IF = 0;                                                     // Clear external interrupt flag bit
    INTCON2bits.RBPU = 1;                                                       // pull-ups RBx OFF
    // Config. Timer0
    T0CONbits.T08BIT = 0;                                                       // 16-bit
    T0CONbits.T0CS = 0;                                                         // Internal clock
    T0CONbits.PSA = 0;                                                          // ON prescaler Timer0
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 1;                                                        // 110 = 1:128 Prescale-> 1seg. overflow
    INTCONbits.T0IF = 0;                                                        // Clear the flag
    INTCONbits.T0IE = 1;                                                        // Enable the interrupt
    // Config. I2C
    I2C_Master_Init(127);                                                       // (7bits->127)(_XTAL_FREQ/(4*62500))-1 =127, inicializa I2C Master a 62.5KHz clock
    SSPCON1bits.WCOL = 0;
    // Config. UART
    UART_Init(68);                                                              // spbrg =832->9600, 68-> 115200, BRG16 = 1, BRGH = 1, SYNC = 0
    PIE1bits.RCIE = 1;                                                          // permiso INT por RX uart
    // Config. Can-Bus
    /*Config interupt CAN- Buffeur 0*/
    IPR3bits.RXB0IP = 0;                                                        // NO alta prioridad
    PIE3bits.RXB0IE = 1;                                                        // permisos INT en Buffer0
    PIR3bits.RXB0IF = 0;                                                        // bandera a 0
    /*Config interupt CAN- Buffeur 1*/
    IPR3bits.RXB1IP = 0;                                                        // NO alta prioridad
    PIE3bits.RXB1IE = 1;                                                        // permisos INT en Buffer1
    PIR3bits.RXB1IF = 0;                                                        // bandera a 0

    //PIE3bits.IRXIE = 1;                                                       // enable can receive error interrupt
    //PIE3bits.ERRIE = 1;                                                       // enable error intrrupt
    //IPR3bits.ERRIP = 0;
    //IPR3bits.IRXIP = 0;
    Led_status =1;
    Zumbador =0;
    while (espera){
        __delay_ms(100);                                                        // espera para asegurar arranque nodos
        ClrWdt();
        espera--;
    }
    Init_CANbus();
    TX_buffer_CAN[0] = 8;                                                       // longitud trama
    DS1337_init();
    if (ini_nextion()) Led_status =0;                                           // inicializamos nextion
    ClrWdt();
    //INTCONbits.GIE_GIEH = 1;                                                  // Enable Global Interrupt (prioridad alta)
}
//****************Escritura de EEPROM interna***********************************
void eeprom_tx_byte(unsigned short direccion, unsigned char dato){
   EEADRH =direccion >> 8;
   EEADR =direccion & 0xFF;
   EEDATA =dato;
   // Secuencia indicada por Microchip
   EECON1bits.EEPGD =0;
   EECON1bits.CFGS =0;
   EECON1bits.WREN =1;                                                          // habilitamos escritura EEPROM
   INTCONbits.GIE =0;                                                           // interrupciones OFF
   EECON2 =0x55;
   EECON2 =0xAA;
   EECON1bits.WR =1;                                                            // escribimos
   while (EECON1bits.WR);                                                       // esperamos que finalize la escritura
   EECON1bits.WREN =0;
   INTCONbits.GIE =1;                                                           // interrupciones ON
}
//****************Lectura de EEPROM interna*************************************
unsigned char eeprom_rx_byte(unsigned short direccion){
    EECON1 =0;
    EEADRH =direccion >> 8;
    EEADR =direccion;
    EECON1bits.RD =1;
    NOP();                                                                      // NOPs espera necesaria
    NOP();
    return EEDATA;
}
//******************Borramos buffer recepción serie*****************************
void clear_buffer_uart(void){
    INTCONbits.PEIE = 0;
    if (RX_off !=0) RX_off =0;                                                  // borramos indicación  0xff 0xff 0xff
    RX_buffer[0] =0;
    while (RX_con){
        RX_buffer[RX_con--] =0;                                                 // iniciamos y limpiamos array trama RX
    }
    INTCONbits.PEIE = 1;
}
//***********Actualizamos hora, para míximo y mánima temperatura****************
void act_hora_max_min(max_min, int_ext){
unsigned char max_min_hora[4] ={0};
    DS1337_leer_hora_fecha(&dt);                                                // actualizamos hora
    max_min_hora[0] =(dt.horas/10)+0x30;
    max_min_hora[1] =(dt.horas%10)+0x30;
    max_min_hora[2] =(dt.minutos/10)+0x30;
    max_min_hora[3] =(dt.minutos%10)+0x30;
    if (max_min){                                                               // comprobamos si es temperatura míxima o mánima
        if (int_ext){                                                           // comprobamos si es temperatura interna o externa
            hora_max_int[0] =max_min_hora[0];                                   // creamos trama hora para nextion
            hora_max_int[1] =max_min_hora[1];
            hora_max_int[2] =':';
            hora_max_int[3] =max_min_hora[2];
            hora_max_int[4] =max_min_hora[3];
            hora_max_int[5] =1;
        }else{                                                                  // temperatura externa máximo
            hora_max_ext[0] =max_min_hora[0];
            hora_max_ext[1] =max_min_hora[1];
            hora_max_ext[2] =':';
            hora_max_ext[3] =max_min_hora[2];
            hora_max_ext[4] =max_min_hora[3];
            hora_max_ext[5] =1;
        }
    }else{                                                                      // mínimo temperatura interna/externa
        if (int_ext){
            hora_min_int[0] =max_min_hora[0];
            hora_min_int[1] =max_min_hora[1];
            hora_min_int[2] =':';
            hora_min_int[3] =max_min_hora[2];
            hora_min_int[4] =max_min_hora[3];
            hora_min_int[5] =1;
        }else{
            hora_min_ext[0] =max_min_hora[0];
            hora_min_ext[1] =max_min_hora[1];
            hora_min_ext[2] =':';
            hora_min_ext[3] =max_min_hora[2];
            hora_min_ext[4] =max_min_hora[3];
            hora_min_ext[5] =1;
        }
    }
}
//***********Actualizamos hora, fecha y día de la semana************************
void act_reloj(unsigned char actualizar){
unsigned char trama_rtc[10] ={0};
    DS1337_leer_hora_fecha(&dt);                                                // actualizamos hora y fecha
    trama_rtc[0] =(dt.horas/10)+0x30;                                           // creamos trama hora para nextion
    trama_rtc[1] =(dt.horas%10)+0x30;
    trama_rtc[2] =':';
    trama_rtc[3] =(dt.minutos/10)+0x30;
    trama_rtc[4] =(dt.minutos%10)+0x30;
    trama_rtc[5] =0;
    txt_nextion((char*)"t0", (char*)trama_rtc);                                 // enviamos hora y minutos
    if (dt.dia_mes != fecha_act || actualizar ==1){                             // si tenemos un nuevo día del mes o petición de actualizar
        trama_rtc[0] =(dt.dia_mes/10)+0x30;                                     // creamos trama_rtc para nextion
        trama_rtc[1] =(dt.dia_mes%10)+0x30;
        trama_rtc[2] ='/';
        trama_rtc[3] =(dt.mes/10)+0x30;
        trama_rtc[4] =(dt.mes%10)+0x30;
        trama_rtc[5] ='/';
        trama_rtc[6] =(dt.anyo/10)+0x30;
        trama_rtc[7] =(dt.anyo%10)+0x30;
        txt_nextion((char*)"t1", (char*)trama_rtc);                             // enviamos día del mes, mes y anyo
        switch (dt.dia){
            case 1:
                trama_rtc[0] ='l'; trama_rtc[1] ='u'; trama_rtc[2] ='n';
                trama_rtc[3] ='e'; trama_rtc[4] ='s'; trama_rtc[5] =0;
            break;
            case 2:
                trama_rtc[0] ='m'; trama_rtc[1] ='a'; trama_rtc[2] ='r';
                trama_rtc[3] ='t'; trama_rtc[4] ='e'; trama_rtc[5] ='s';
                trama_rtc[6] =0;
            break;
            case 3:
                trama_rtc[0] ='m'; trama_rtc[1] ='i'; trama_rtc[2] ='e';
                trama_rtc[3] ='r'; trama_rtc[4] ='c'; trama_rtc[5] ='o';
                trama_rtc[6] ='l'; trama_rtc[7] ='e'; trama_rtc[8] ='s';
                trama_rtc[9] =0;
            break;
            case 4:
                trama_rtc[0] ='j'; trama_rtc[1] ='u'; trama_rtc[2] ='e';
                trama_rtc[3] ='v'; trama_rtc[4] ='e'; trama_rtc[5] ='s';
                trama_rtc[6] =0;
            break;
            case 5:
                trama_rtc[0] ='v'; trama_rtc[1] ='i'; trama_rtc[2] ='e';
                trama_rtc[3] ='r'; trama_rtc[4] ='n'; trama_rtc[5] ='e';
                trama_rtc[6] ='s'; trama_rtc[7] =0;
            break;
            case 6:
                trama_rtc[0] ='s'; trama_rtc[1] ='a'; trama_rtc[2] ='b';
                trama_rtc[3] ='a'; trama_rtc[4] ='d'; trama_rtc[5] ='o';
                trama_rtc[6] =0;
            break;
            case 7:
                trama_rtc[0] ='d'; trama_rtc[1] ='o'; trama_rtc[2] ='m';
                trama_rtc[3] ='i'; trama_rtc[4] ='n'; trama_rtc[5] ='g';
                trama_rtc[6] ='o'; trama_rtc[7] =0;
            break;
        }
        txt_nextion((char*)"t2", (char*)trama_rtc);                             // enviamos día de la semana
    }
}
//**********Convertimos 4 bytes a float y el float a string*********************
signed short byte_float_string(unsigned char byte_0, unsigned char byte_1, unsigned char byte_2, unsigned char byte_3){
unsigned char ch, *p, *buf, x =0, y =0;
int status;
union {
  float f;
  unsigned char b[4];
} u;
    u.b[3] = byte_0;
    u.b[2] = byte_1;
    u.b[1] = byte_2;
    u.b[0] = byte_3;
    buf = ftoa(u.f, &status);
    p =buf;
    while (*p!=0){
        ch=*p;                                                                  // accedemos al siguiente carácter
        if (y <9){                                                              // comprobamos si tenemos 2 decimales
            trama_ascii[x] = *p;
            y++;
        }else{
            trama_ascii[x]=0;                                                   // solo dos decimales
        }
        p++;
        if (trama_ascii[x] ==0x2e){                                             // comprobamos si es el punto decimal
            trama_ascii[x] =0x2C;                                               // cambiamos punto por coma
            y =7;
        }
        if (x <9) x++;                                                          // nos aseguramos de no desbordar el array
    }
    return (signed short)u.f;                                                   // retornamos el valor entero
}
//***Tendencia baromótrica/temperatura se actualiza cada 60 minutos*************
// se indica un incremento si la presión aumenta 2 mb (margen)
// se indica un descenso  si la presión decrece  2 mb (margen)
// se indica un incremento si la temperatura aumenta 5 grados (margen)
// se indica un descenso  si la temperatura decrece 5 grados (margen)
unsigned char tendencia(unsigned char a, unsigned char b, unsigned char margen){
    if (a !=b){
        if (a >b){                                                              // comprobamos si A es mayor de B
            if ((a -b) >margen){
                return 2;                                                       // incrementa A referente a B->incremento
            }
        }else{                                                                  // B es mayor de A
            if ((b -a) >margen){
                return 3;                                                       // incrementa B referente a A->descenso
            }
        }
    }
    return 1;                                                                   // iguales->estable
}
/*Obtenemos la previsión meteorológica según presión y temperatura (poco fiable!!)
BARÓMETRO       TERMÓMETRO           PREDICCIÓN
Bajando          bajando               tormentas
Bajando          estable               lluvias probables
Bajando          subiendo              nubes y claros
Estable          bajando               lluvias probables
Estable          estable               nubes y claros
Estable          subiendo              soleado
Subiendo         bajando               viento
Subiendo         estable               soleado
Subiendo         subiendo              soleado
soleado->14, Lluvias probables-> 15, tormentas->16, nubes y claros->17,  viento->18
*/
unsigned char prevision_meteo(){
unsigned char temp, pres, media_pre_ant, media_pre_act;
signed char  media_tem_ant, media_tem_act;
    media_tem_ant =(signed char) (gra_tem_ext[posicion-19] +gra_tem_ext[posicion-18] + // media temperatura anterior
    gra_tem_ext[posicion-17] +gra_tem_ext[posicion-16] +gra_tem_ext[posicion-15])/5;

    media_tem_act =(signed char) (gra_tem_ext[posicion-4] +gra_tem_ext[posicion-3] +
    gra_tem_ext[posicion-2]+ gra_tem_ext[posicion-1] +gra_tem_ext[posicion])/5; // media temperatura actual

    media_pre_ant =(unsigned char) (gra_presion[posicion-19] +gra_presion[posicion-18] +// media presión anterior
    gra_presion[posicion-17]+ gra_presion[posicion-16] +gra_presion[posicion-15])/5;

    media_pre_act =(unsigned char) (gra_presion[posicion-4]+gra_presion[posicion-3]+gra_presion[posicion-2]+
    gra_presion[posicion-1]+gra_presion[posicion])/5;                           // media presión actual

    if (media_tem_act <0)                                                       // comprobamos si la temperatura es negativa
        media_tem_act =256-(unsigned char) media_tem_act;                       // obtenemos los grados negativos en valor positivo
    if (media_tem_ant <0)                                                       // comprobamos si la temperatura es negativa
        media_tem_ant =256-(unsigned char) media_tem_ant;                       // obtenemos los grados negativos en valor positivo
    temp =tendencia (media_tem_act, media_tem_ant, 5);
    pres =tendencia (media_pre_ant, media_pre_act, 2);

    if (pres ==3){
        if (temp ==3) return 16;                                                // bajando presión y temperatura->tormenta
        if (temp ==1) return 15;                                                // bajando presión y estable temperatura->lluvias probables
        return 17;                                                              // bajando presión y bajando temperatura->nubes y claros
    }
    if (pres ==2){
        if (temp ==2) return 18;                                                // subiendo presión y bajando temperatura->viento
        return 14;                                                              // subiendo presión y temperatura->soleado
                                                                                // subiendo presión y estable temperatura->soleado
    }
    if (pres ==1){
        if (temp ==1) return 17;                                                // estable presión y temperatura->nubes y claros
        if (temp ==2) return 15;                                                // estable presión y bajando temperatura->lluvias probables
        return 14;                                                              // estable presión y subiendo temperatura->soleado
    }
    return 14;                                                                  // soleado
}
// Gestión de la gráfica de la temperatura externa/interna, gráfica de 446*200
// (446*3=1338minutos->22,3 horas), 1dia =1440 minutos, 480 ciclos
void grafica_temp(signed char dato, unsigned char ext_int){
unsigned char  temp_int;
unsigned short x;
    if (ciclo_max_min >1){                                                      // inicia con ciclo_max_min a 2
        if (ext_int){
            if (dato > max_temp_ext){
                act_hora_max_min(1, 0);                                         // actualizamos la hora de máximo valor
                max_temp_ext =dato;                                             // obtenemos el máximo valor temperatura externa
            }else{
                if (dato < min_temp_ext){
                    act_hora_max_min(0, 0);                                     // actualizamos la hora de mínimo valor
                    min_temp_ext =dato;                                         // obtenemos el mínimo valor temperatura externa
                }
            }
        }else{
            temp_int =(unsigned char) dato;
            if (temp_int > max_temp_int){
                act_hora_max_min(1, 1);                                         // actualizamos la hora de máximo valor
                max_temp_int =temp_int;                                         // obtenemos el máximo valor temperatura interna
            }else{
                if (temp_int < min_temp_int){
                    act_hora_max_min(0, 1);                                     // actualizamos la hora de mínimo valor
                    min_temp_int =temp_int;                                     // obtenemos el mínimo valor temperatura interna
                }
            }
        }
    }
    if (posicion !=445){                                                        // comprobamos si tenemos el array completo
        if (ext_int){
            gra_tem_ext[posicion]=dato;                                         // introducción los datos en el array temperatura externa
        }else{
            gra_tem_int[posicion]=temp_int;                                     // introducción los datos en el array temperatura interna
        }
    }else{
        for(x =0; x <445; x++){
            if (ext_int){
                gra_tem_ext[x] =gra_tem_ext[x+1];	                            // rotamos gráfica de derecha a izquierda temperatura externa
            }else{
                gra_tem_int[x] =gra_tem_int[x+1];	                            // rotamos gráfica de derecha a izquierda temperatura interna
            }
        }
        if (ext_int){
            gra_tem_ext[445] =dato;                                             // el ultimo valor leído lo colocamos en el ultima posición de la array
        }else{
            gra_tem_int[445] =temp_int;
        }
    }
}
// Gestión de la gráfica del la presión, gráfica de 446*200
// 1dia =1440 minutos (446*3=1338minutos)
void grafica_presion(unsigned short dato){
unsigned char y;
unsigned short x;
    if (dato >875 && dato <1076)
        y =(unsigned char) dato -875;                                           // restamos mínima 875 (presión de 0 a 200 presión máxima = 1075mb)
    else
        if (dato <875) y =1; else y =200;                                       // valores fuera de rango
    if (posicion ==445){                                                        // comprobamos si tenemos el array completo
        for(x =0; x <445; x++){
            gra_presion[x] =gra_presion[x+1];	                                // rotamos gráfica de derecha a izquierda
        }
    }
    gra_presion[posicion] =y;
}
// Gestión de la gráfica del la potencia, gráfica de 446*200
// 1dia =1440 minutos, (446*3=1338 minutos, 22,3horas)
void grafica_consumo(unsigned short dato){
unsigned char y;
unsigned short x;
    if (dato >28 && dato <5801)
        y = dato /29;                                                           // dividir por 2 (potencia máxima = 200*29=5800W
    else{
        if (dato <28) y =1; else y =200;                                        // potencia inferior a 29 =1 o mayor a 5800 =posición 200 en gráfica
    }
    if (posicion ==445){                                                        // comprobamos si tenemos el array completo
        for(x =0; x <445; x++){
            gra_potencia[x] =gra_potencia[x+1];	                                // rotamos gráfica de derecha a izquierda
        }
    }
    gra_potencia[posicion] =(unsigned char)y;                                   // introducción los datos en el array
}
//*******Enviamos los datos almacenados a la gráfica correspondiente************
unsigned char tx_grafica (unsigned char id, unsigned char canal, unsigned char grafica){
unsigned char trama_gar[13] ={'a','d','d','t',' ',' ',',',' ',',',' ',' ',' ',0}, espera =0;
unsigned char longitud[5] = {0}, TX_dato;
unsigned short x;
    itoa(longitud, posicion, 10 );                                              // convertimos entero a cadena de texto
    trama_gar[5] =id+0x30;
    trama_gar[7] =canal+0x30;
    trama_gar[9] =longitud[0];
    trama_gar[10] =longitud[1];
    if (longitud[2] !=0)
        trama_gar[11] =longitud[2];
    else
        trama_gar[11] =0;
    clear_buffer_uart();
    cmd_nextion((char*)trama_gar);                                              // petición envió de datos (add id,canal,longitud)
    do{                                                                         // ~5ms para pasar a modo RX transparente
        if (RX_off ==3){
            RX_off =0;
            if (RX_buffer[0] == 0xfe){                                          // leemos el byte 0
                break;
            }else{
                clear_buffer_uart();
            }
        }else{
            espera++;
            __delay_ms(1);
        }
    }while (espera <250);                                                       // esperamos confirmación de nextion (0xfe 0xff 0xff 0xff)
    if (espera ==250) return 1;
    espera =0;
    clear_buffer_uart();
    for(x =0; x <posicion; x++){                                                // enviar los datos
        switch (grafica){
            case 0:                                                             // gráfica temperatura externa
                if (gra_tem_ext[x] <0){                                         // comprobamos si la temperatura es negativa
                    TX_dato =256-(unsigned char) gra_tem_ext[x];                // obtenemos los grados negativos en valor positivo
                    TX_dato =100-(TX_dato*2);                                   // doblamos para mostrar en gráfica de 0 a 100
                                                                                // ajustamos de 99 a 0
                }else{
                    TX_dato =(unsigned char) gra_tem_ext[x];
                    if (TX_dato >0)                                             // si la temperatura es mayor de 0 grados
                        TX_dato = TX_dato*2;                                    // la doblamos para mostrar entre 100 y 200 (50 grados máximo)
                    TX_dato =TX_dato+100;                                       // 0 grados corresponde a la posición 100 (centro de la gráfica)
                }

            break;
            case 1:                                                             // gráfica temperatura interna, 0 a 50 grados
                if (gra_tem_int[x] >0)                                          // la doblamos para mostrar entre 100 y 200 (50 grados máximo)
                    TX_dato =gra_tem_int[x]*2;
                TX_dato =TX_dato+100;                                           // 0 grados corresponde a la posición 100 (centro de la gráfica)
            break;
            case 2:                                                             // gráfica presión
                TX_dato =gra_presion[x];
            break;
            case 3:                                                             // gráfica consumo
                TX_dato =gra_potencia[x];
            break;
        }
        putch(TX_dato);
    }
    do{                                                                         // esperamos confirmación de datos recibidos (0xfd 0xff 0xff 0xff)
        if (RX_off ==3){
            RX_off =0;
            if (RX_buffer[0] == 0xfd){
                clear_buffer_uart();
                return 0;                                                       // indicamos trama TX OK
            }else{
                clear_buffer_uart();
            }
        }else{
            espera++;
            __delay_ms(1);
        }
    }while (espera <250);
    clear_buffer_uart();
    return 1;
}
//***********Actualizamos siguiente periodo para paso a reposo******************
unsigned char update_reposo(unsigned char reposo){
    DS1337_leer_hora_fecha(&dt);
    reposo = dt.minutos +reposo;                                                // obtenemos le siguiente periodo para sleep
    if(reposo >59)                                                              // de 0 a 59
        reposo = reposo-60;
    return reposo;
}
//*******************Accedemos a la pagina del reloj****************************
void reloj_ON(void){
unsigned char espera_rtc =240, segundo =100;                                    // 240 *250=1 minutos
    cmd_nextion((char*)"sleep=0");                                              // ON nextion
    __delay_ms(150);
    cmd_nextion((char*)"page 13");                                              // accedemos a la pagina del reloj
    do{
        espera_rtc--;
        DS1337_leer_hora_fecha(&dt);
        if (segundo !=dt.segundos){
            num_nextion((char*)"n0", dt.horas, 0);                              // n0.val=hora
            num_nextion((char*)"n1", dt.minutos, 0);                            // n1.val=minuto
            num_nextion((char*)"n2", dt.segundos, 0);                           // n2.val=segundo
            num_nextion((char*)"n3", dt.dia_mes, 0);                            // n3.val=día del mes
            num_nextion((char*)"n4", dt.mes, 0);                                // n4.val=mes
            num_nextion((char*)"n5", dt.anyo, 0);                               // n5.val=anyo
            num_nextion((char*)"va0", dt.dia, 0);                               // va0.val=día de la semana
            segundo =dt.segundos;
        }
        __delay_ms(250);
    }while (espera_rtc);
    cmd_nextion((char*)"sleep=1");                                              // en reposo
}
//*****************Accedemos a la pagina del temperaturas***********************
void temperaturas(unsigned char temp_comedor){
unsigned char espera_temp =30, run =0;
    cmd_nextion((char*)"sleep=0");                                              // ON nextion
    __delay_ms(150);
    cmd_nextion((char*)"page 16");                                              // accedemos a la pagina de temperaturas
    txt_nextion((char*)"t5", (char*)trama_ascii);                               // t5.txt="temperatura externa"
    num_nextion((char*)"n1", temp_comedor, 0);                                  // n1.val=temperatura comedor
    do{
        espera_temp--;
        num_nextion((char*)"r0", run, 0);
        run = !run;
        __delay_ms(1000);
    }while (espera_temp);
    cmd_nextion((char*)"sleep=1");                                              // en reposo

}
//********************Indicación OK a traves de zumbador************************
void OK_sound(void){
    Zumbador =1;
    __delay_ms(50);
    Zumbador =0;
}
////////////////////////////////////////////////////////////////////////////////
//**************************PRINCIPAL*******************************************
////////////////////////////////////////////////////////////////////////////////
void main(void) {
unsigned char minuto =0, segundo =0, reposo, siguiente_p, act_minuto =60, auto_page =1,
              pulsador_on =0, ciclo_ini =1, reposo_m, cont_segundos =0, RX_can =0,
              int_total =0, off_alarm =0, prevision =14, espera_prevision =0,
              ciclo_media_temp =0, int_out =0, out =0, ciclo_minuto=0, nuevo_dia =0,
              tension =0, int_luces =0, int_aire =0, int_terraza =0,
              int_enchufes =0, int_cocina =0, tem_habitacion =0, tem_cuartillo=0,
              tem_pasillo =0, tem_aseo =0, hum_habitacion =0, hum_cuartillo =0,
              hum_pasillo =0, hum_aseo =0, tem_comedor =0, tem_cocina =0, tem_plancha =0,
              tem_taller =0, hum_comedor =0, hum_cocina =0, hum_plancha =0, hum_taller =0,
              temp_int_ok =0, temperatura[4] ={0}, tem_bmp085[4] ={0}, lluvia_m =0,
              vcc, presion[4] ={0}, rad_UV =0, humedad =0, ok_consumo =0,
              page[7] ={'p','a','g','e',' ','0',0}, alar_on_off =22, update_lluvia_h =0,
              update_lluvia_m =0;
signed char   tem_ext =0, tem_media =0, tem_media_act, tem_media_graf =0;
unsigned short potencia =0, alarma_pot, viento =0, lluvia_h =0, int_mA  =0,
               direcc_viento =0, pres_ext =0, rad_solar =0, acum_lluvia_h =65535,
               max_potencia =0, acumulador_temp =0;
    config();
    ClrWdt();
    siguiente_p =eeprom_rx_byte (0);                                            // obtenemos los segundos de siguiente pantalla
    reposo =eeprom_rx_byte (1);                                                 // obtenemos los minutos para pasar a reposo
    alarma_pot =((eeprom_rx_byte (3)) << 8) | eeprom_rx_byte (2);               // obtenemos el valor para la alarma de potencia máxima
    reposo_m =update_reposo(reposo);
    act_reloj(ciclo_ini);                                                       // actualizamos la hora y el calendario
    INTCONbits.PEIE = 1;                                                        // activamos interrupción de periféricos
    INTCONbits.GIE =1;
    while (1){
        if (actualiza){
            DS1337_leer_hora_fecha(&dt);                                        // actualizamos hora y fecha cada nuevo segundo
            segundo =dt.segundos;
            minuto =dt.minutos;
            actualiza =0;
            if (sleep ==0) Led_status = !Led_status;
            cont_segundos++;
            // gestión siguiente pagina
            if (cont_segundos ==siguiente_p){
                cont_segundos =0;
                if (sleep ==0){                                                 // siguiente pagina, paginas de 0 a la 9
                    auto_page =1;                                               // indicamos cambio de pagina automático
                    cmd_nextion((char*)page);
                }
            }
            // gestión datos gráficas y previsión
            if ((minuto %3) ==0 && act_minuto !=minuto){                        // cada múltiplo de 3 minutos actualizamos
                if (ciclo_minuto ==2){                                          // despreciamos los 6 primeros minutos
                    grafica_temp (tem_media_graf, 1);                           // temperatura externa
                    acumulador_temp =0;
                    ciclo_media_temp =0;
                    grafica_temp (tem_media, 0);                                // temperatura interna media
                    grafica_presion (pres_ext);                                 // presión atmosférica
                    grafica_consumo (max_potencia);                             // consumo eléctrico
                    max_potencia =29;
                    if (posicion !=445) posicion++;                             // incrementamos posición
                }else
                    ciclo_minuto++;
                if (espera_prevision ==20){
                    prevision =prevision_meteo();                               // cada 60 minutos
                    espera_prevision =0;
                }else
                    espera_prevision++;
                act_minuto =minuto;
            }
            // gestión reposo
            if (minuto ==reposo_m && sleep ==0 && alar_on_off ==22){
                pulsador_on =0;
                Led_status =0;
                cmd_nextion((char*)"sleep=1");                                  // en reposo
                sleep =1;
            }
            // gestión nuevo día
            if (fecha_act !=dt.dia_mes){                                        // si cambio el día actualizamos
                if (sleep ==1){
                    cmd_nextion((char*)"sleep=0");                              // ON nextion
                    sleep =0;
                    __delay_ms(150);
                }
                ciclo_ini =1;
                act_reloj(1);
                ciclo_max_min =0;
                auto_page =1;
                reposo_m =update_reposo(reposo);
                cmd_nextion((char*)"page 0");
                fecha_act =dt.dia_mes;
                nuevo_dia =1;
            }
            //----------------------TX DATOS CAN-Bus----------------------------
            switch (segundo){
                case 5: case 27: case 47:
                    RX_can =0x0A;                                               // petición datos nodo consumo, cada segundo 5 , 27 y 47
                break;
                case 10: case 37:
                    RX_can =0x0C;                                               // nodo temperatura/humedad 1, cada segundo 10 y 37
                break;
                case 15: case 42:
                    RX_can =0x0E;                                               // nodo temperatura/humedad 2, cada segundo 15 y 42
                break;
                case 20: case 35: case 50:
                    RX_can =0x2C;                                               // nodo meteo 1, cada segundo 20, 35 y 50
                break;
                case 25: case 40: case 55:
                    RX_can =0x2D;                                               // nodo meteo 2, cada segundo 25, 40 y 55
                break;
                case 30: case 45: case 59:
                    RX_can =0x2E;                                               // nodo meteo 3, cada segundo 30, 45 y 59
                break;
            }
            if (RX_can) Transmit_CANbus(RX_can, 0);                             // TX CAN-bus
        }
        //---------------------------RX DATOS CAN-Bus---------------------------
        if (RxCanFlag){                                                         // comprobamos si tenemos datos del CAN-bus
            RxCanFlag =0;                                                       // borramos indicación datos CAN-bus
            if (Receive_CANbus()){
                switch (RX_can){                                                // comprobamos a que nodo se le realizo la petición de datos
                    case 0x0A:                                                  // nodo consumo
                        tension =(unsigned char)(temp_D0 *1.52);                // pasamos a voltios
                        int_luces =(unsigned char)(temp_D1 /11.73);             // pasamos a amperios 0-20A,
                        int_aire =(unsigned char)(temp_D2 /11.73);
                        int_terraza =(unsigned char)(temp_D3 /11.73);
                        int_enchufes =(unsigned char)(temp_D4 /11.73);
                        int_cocina =(unsigned char)(temp_D5 /11.73);            //cada unidad corresponde a unos 85mA
                        int_mA =((temp_D1 + temp_D2 + temp_D3 + temp_D4 + temp_D5)*90);// *90 para redondeo al alza
                        int_total =(unsigned char)(int_mA/1000);
                        potencia =(unsigned short)(tension*(float)int_mA/1000);
                        if (potencia >max_potencia) max_potencia =potencia;
                        ok_consumo =1;
                    break;
                    case 0x0C:                                                  // nodo temperatura/humedad interna 1
                        tem_habitacion =temp_D0;
                        tem_cuartillo =temp_D1;
                        tem_pasillo =temp_D2;
                        tem_aseo =temp_D3;
                        hum_habitacion =temp_D4;
                        hum_cuartillo =temp_D5;
                        hum_pasillo =temp_D6;
                        hum_aseo =temp_D7;
                        temp_int_ok =tem_habitacion +tem_cuartillo +tem_pasillo +tem_aseo;
                    break;
                    case 0x0E:                                                  // nodo temperatura/humedad interna
                        tem_comedor =temp_D0;
                        tem_cocina =temp_D1;
                        tem_plancha =temp_D2;
                        tem_taller =temp_D3;
                        hum_comedor =temp_D4;
                        hum_cocina =temp_D5;
                        hum_plancha =temp_D6;
                        hum_taller =temp_D7;
                        if (temp_int_ok >4)
                            temp_int_ok = tem_comedor +tem_cocina +tem_plancha + tem_taller +temp_int_ok;
                        if (temp_int_ok >8){
                            tem_media_act =temp_int_ok/8;
                            if (tem_media_act >5){                              // 5 grados como mínima temperatura media interna
                                if (tem_media_act >tem_media-5 && tem_media_act <tem_media+5)
                                    tem_media =tem_media_act;
                            }
                        }
                        if (tem_media ==0) tem_media =tem_comedor;
                        temp_int_ok =0;
                    break;
                    case 0x2C:                                                  // nodo estación meteorológica 1
                        temperatura[0] =temp_D0;                                // temperatura externa (4 bytes)
                        temperatura[1] =temp_D1;
                        temperatura[2] =temp_D2;
                        temperatura[3] =temp_D3;
                        tem_bmp085[0] =temp_D4;                                 // temperatura sensor baromótrico bmp085 (4 bytes)
                        tem_bmp085[1] =temp_D5;
                        tem_bmp085[2] =temp_D6;
                        tem_bmp085[3] =temp_D7;
                        tem_ext =byte_float_string(temperatura[0], temperatura[1], temperatura[2], temperatura[2]);
			if (tem_ext <0) tem_ext =tem_ext+3275;                  // si es negativa realizamos la corrección
                        acumulador_temp = acumulador_temp +tem_ext;
                        ciclo_media_temp ++;
                        tem_media_graf =(signed char)(acumulador_temp/ciclo_media_temp);

                    break;
                    case 0x2D:                                                  // nodo estación meteorológica 2
                        viento = ((temp_D1 << 8) | temp_D0);                    // velocidad del viento (2 bytes)
                        direcc_viento =((temp_D6 << 8) | temp_D5);              // dirección de viento (2 bytes)
                        vcc =temp_D7;                                           // tensión microprocesador
                        if (temp_D2) lluvia_m =temp_D2;                         // pulsos lluvia 1 minuto
                        if (update_lluvia_m){
                            lluvia_m =0;
                            update_lluvia_m =0;
                        }
                        lluvia_h =((temp_D4 << 8) | temp_D3);                   // pulsos lluvia 1 hora (2 bytes)
                        if (lluvia_h) acum_lluvia_h =acum_lluvia_h +lluvia_h;
                        if (update_lluvia_h){                                   // se envió a nextion
                            if (acum_lluvia_h ==65535){
                                acum_lluvia_h =0;
                                nuevo_dia =0;
                            }
                            update_lluvia_h =0;
                        }
                        if (nuevo_dia ==1) acum_lluvia_h =65535;                // indicamos fin acumulación lluvia día
                    break;
                    case 0x2E:                                                  // nodo estación meteorológica 3
                        presion[0] =temp_D0;                                    // presión atmosfórica (4 bytes)
                        presion[1] =temp_D1;
                        presion[2] =temp_D2;
                        presion[3] =temp_D3;
                        pres_ext =(unsigned short) byte_float_string(presion[0], presion[1], presion[2], presion[3]);
                        rad_solar =((temp_D5 << 8) | temp_D4);                  // radiación solar (2 bytes)
                        rad_UV =temp_D6;                                        // radiación ultravioleta (de 0 a 11)
                        humedad =temp_D7;                                       // humedad externa
                    break;
                    case 0x20:                                                  // nodo int-out
                        int_out =temp_D0;
                        // estado entradas
                        num_nextion((char*)"va0", (!!((int_out)&1<<(4))), 0);   // enviamos el estado del bit 4
                        num_nextion((char*)"va1", (!!((int_out)&1<<(5))), 0);   // enviamos el estado del bit 5
                        num_nextion((char*)"va2", (!!((int_out)&1<<(6))), 0);   // enviamos el estado del bit 6
                        num_nextion((char*)"va3", (!!((int_out)&1<<(7))), 0);   // enviamos el estado del bit 7
                        // estado pulsadores
                        num_nextion((char*)"va4", (!!((int_out)&1<<(0))), 0);   // enviamos el estado del bit 0
                        num_nextion((char*)"va5", (!!((int_out)&1<<(1))), 0);   // enviamos el estado del bit 1
                        num_nextion((char*)"va6", (!!((int_out)&1<<(2))), 0);   // enviamos el estado del bit 2
                        num_nextion((char*)"va7", (!!((int_out)&1<<(3))), 0);   // enviamos el estado del bit 3
                    break;
                }
            }
            RX_can =0;
        }
        //-----------------------RX DATOS RS-232 Nextion------------------------
        if (RX_off ==3){                                                        // comprobamos si tenemos una trama completa
            RX_off =0;                                                          // iniciamos RX fin trama->0xff 0xff 0xff
            switch (RX_buffer[0]){
                case 0x66:                                                      // trama de indicación nueva pagina
                    if (!auto_page) cont_segundos =0;                           // comprobamos si pasamos a la pagina automáticamente
                    auto_page =0;
                    act_reloj(ciclo_ini);                                       // actualizamos la hora y el calendario
                    switch (RX_buffer[1]){
                        case 0x00:                                              // pagina principal
                            num_nextion((char*)"n0", tension, 0);               // n0.val=tensión
                            num_nextion((char*)"n1", int_total, 0);             // n1.val=intensidad total
                            num_nextion((char*)"n2", potencia, 0);              // n2.val=potencia en W
                            byte_float_string(temperatura[0], temperatura[1], temperatura[2], temperatura[2]);
                            txt_nextion((char*)"t5", (char*)trama_ascii);       // t5.txt="temperatura externa"
                            num_nextion((char*)"n3", tem_media, 0);             // n3.val=temperatura interior media
                            num_nextion((char*)"n4", viento, 0);                // n4.val=velocidad viento
                            num_nextion((char*)"va0", acum_lluvia_h, 0);        // va0.val= pulsos de lluvia acumulados en una hora
                            update_lluvia_h =1;                                 // indicamos enviado pulsos lluvia hora
                            if (prevision){
                                num_nextion((char*)"r0", 0, 0);                 // r0.val=estado previsión, 0 OK
                                num_nextion((char*)"p0", prevision, 1);         // p0.pic=imagen de 14 a 18
                            }
                            page[5] ='1';                                       // indicamos siguiente pagina
                        break;
                        case 0x01:                                              // pagina meteo1
                            num_nextion((char*)"va0", direcc_viento, 0);        // va0.val=dirección viento-> Z0
                            num_nextion((char*)"n1", viento, 0);                // n1.val=velocidad viento
                            num_nextion((char*)"n2", humedad, 0);               // n2.val=humedad
                            pres_ext =(unsigned short) byte_float_string(presion[0], presion[1], presion[2], presion[3]);
                            txt_nextion((char*)"t3", (char*)trama_ascii);       // t3.txt=presión baromótrica
                            num_nextion((char*)"va1", pres_ext, 0);             // va1.val=presión-> Z1
                            num_nextion((char*)"va3", lluvia_m, 0);             // va3.val=pulsos de lluvia último minuto
                            update_lluvia_m =1;                                 // indicamos enviado pulsos lluvia minuto
                            page[5] ='2';                                       // indicamos siguiente pagina
                        break;
                        case 0x02:                                              // pagina meteo2
                            tem_ext =byte_float_string(temperatura[0], temperatura[1], temperatura[2], temperatura[3]);
			    if (tem_ext <0) tem_ext =tem_ext+3275;              // si es negativa realizamos la corrección
                            txt_nextion((char*)"t5", (char*)trama_ascii);       // t5.txt="temperatura externa"
                            if (ciclo_max_min ==0){
                                max_temp_ext =tem_ext;
                                min_temp_ext =max_temp_ext;
                                act_hora_max_min(0, 0);                         // mínima temperatura externa
                                act_hora_max_min(1, 0);                         // máxima temperatura externa
                                ciclo_max_min =1;
                            }
                            num_nextion((char*)"va0", max_temp_ext, 0);         // va0.val=míxima temperatura externa (valores negativos 255 a 128 gestionados por nextion)
                            if (hora_max_ext[5] ==1){
                                hora_max_ext[5] =0;
                                txt_nextion((char*)"t11", (char*)hora_max_ext); // si se actualizó el valor de máximo indicamos la hora
                            }
                            num_nextion((char*)"va1", min_temp_ext, 0);         // va1.val=mínima temperatura externa (valores negativos 255 a 128 gestionados por nextion)
                            if (hora_min_ext[5] ==1){
                                hora_min_ext[5] =0;
                                txt_nextion((char*)"t12", (char*)hora_min_ext); // si se actualizó el valor de mínimo indicamos la hora
                            }
                            num_nextion((char*)"n0", rad_solar, 0);             // n0.val=radiación solar
                            num_nextion((char*)"n3", rad_UV, 0);                // n0.val=radiación ultravioleta
                            page[5] ='3';                                       // indicamos siguiente pagina
                        break;
                        case 0x03:                                              // pagina 3 temperatura interior
                            num_nextion((char*)"n0", tem_media, 0);             // n0.val=temperatura media interna
                            num_nextion((char*)"n3", hum_comedor, 0);           // n3.val=humedad comedor
                            if (ciclo_max_min ==1 && tem_comedor >0){
                                max_temp_int =tem_comedor;
                                min_temp_int =max_temp_int;
                                act_hora_max_min(0, 1);                         // mínima temperatura interna
                                act_hora_max_min(1, 1);                         // máxima temperatura  interna
                                ciclo_max_min =2;                               // arrancamos máximo/mínimo
                            }
                            if (ciclo_max_min >1){
                                num_nextion((char*)"n1", max_temp_int, 0);      // n1.val=míxima temperatura interna
                                if (hora_max_int[5] ==1){
                                    hora_max_int[5] =0;
                                    txt_nextion((char*)"t11", (char*)hora_max_int);// si se actualizó el valor de máximo indicamos la hora
                                }
                                num_nextion((char*)"n2", min_temp_int, 0);      // n2.val=mánima temperatura interna
                                 if (hora_min_int[5] ==1){
                                    hora_min_int[5] =0;
                                    txt_nextion((char*)"t12", (char*)hora_min_int);// si se actualizó el valor de mínimo indicamos la hora
                                }
                            }
                            page[5] ='4';                                       // indicamos siguiente pagina
                        break;
                        case 0x04:                                              // pagina temperaturas interior
                            num_nextion((char*)"n0", tem_comedor, 0);           // n0.val=temperatura comedor
                            num_nextion((char*)"n1", tem_cocina, 0);            // n1.val=temperatura cocina
                            num_nextion((char*)"n2", tem_habitacion, 0);        // n2.val=temperatura habitación
                            num_nextion((char*)"n3", tem_pasillo, 0);           // n3.val=temperatura pasillo
                            num_nextion((char*)"n4", tem_aseo, 0);              // n4.val=temperatura aseo
                            num_nextion((char*)"n5", tem_cuartillo, 0);         // n5.val=temperatura cuartillo
                            num_nextion((char*)"n6", tem_taller, 0);            // n6.val=temperatura taller
                            num_nextion((char*)"n7", tem_plancha, 0);           // n7.val=temperatura  plancha
                            page[5] ='5';                                       // indicamos siguiente pagina
                        break;
                        case 0x05:                                              // pagina humedad interior
                            num_nextion((char*)"n0", hum_comedor, 0);           // n0.val=humedad comedor
                            num_nextion((char*)"n1", hum_cocina, 0);            // n1.val=humedad cocina
                            num_nextion((char*)"n2", hum_habitacion, 0);        // n2.val=humedad habitación
                            num_nextion((char*)"n3", hum_pasillo, 0);           // n3.val=humedad pasillo
                            num_nextion((char*)"n4", hum_aseo, 0);              // n4.val=humedad aseo
                            num_nextion((char*)"n5", hum_cuartillo, 0);         // n5.val=humedad cuartillo
                            num_nextion((char*)"n6", hum_taller, 0);            // n6.val=humedad taller
                            num_nextion((char*)"n7", hum_plancha, 0);           // n7.val=humedad  plancha
                            if (posicion >9 || ciclo_ini ==1)
                                page[5] ='6';                                   // indicamos siguiente pagina
                            else
                                page[5] ='8';
                        break;
                        case 0x06:                                              // pagina gráfica temperatura interior/exterior
                            if (posicion >9){
                                num_nextion((char*)"r0", tx_grafica(6, 0, 0), 0);// temperatura externa canal 0
                                ClrWdt();
                                num_nextion((char*)"r1", tx_grafica(6, 1, 1), 0);// temperatura interna canal 1
                            }
                            page[5] ='7';                                       // indicamos siguiente pagina
                        break;
                        case 0x07:                                              // pagina gráfica presión atmosférica
                            if (posicion >9){
                                num_nextion((char*)"r0", tx_grafica(5, 0, 2), 0);// presión atmosférica
                            }
                            page[5] ='8';                                       // indicamos siguiente pagina
                        break;
                        case 0x08:                                              // pagina consumo
                            num_nextion((char*)"n0", int_enchufes, 0);          // n0.val=intensidad enchufes
                            num_nextion((char*)"n1", int_cocina, 0);            // n1.val=intensidad cocina
                            num_nextion((char*)"n2", int_terraza, 0);           // n2.val=intensidad termo, lavadora
                            num_nextion((char*)"n3", int_luces, 0);             // n3.val=intensidad luces
                            num_nextion((char*)"n4", int_aire, 0);              // n4.val=intensidad aire acondicionado
                            num_nextion((char*)"n5", tension, 0);               // n5.val=intensidad tensión
                            num_nextion((char*)"n6", potencia, 0);              // n6.val=intensidad potencia en W
                            num_nextion((char*)"n7", int_mA, 0);                // n7.val=intensidad total en mA
                            num_nextion((char*)"p0", alar_on_off, 1);           // p0.pic=imagen 22 ok, alarma 35 y >1Kw 36
                            if (alar_on_off !=22) break;                        // finalizamos si nos encontramos en alarma o consumo >1Kw
                            if (posicion >9 || ciclo_ini ==1)
                                page[5] ='9';                                   // indicamos siguiente pagina
                            else{
                                page[5] ='0';

                            }
                        break;
                        case 0x09:                                              // pagina gráfica consumo
                            if (posicion >9){
                                num_nextion((char*)"r0", tx_grafica(6, 0, 3), 0);// potencia
                            }
                            page[5] ='0';                                       // indicamos siguiente pagina  a la 0
                            ciclo_ini =0;
                        break;
                        case 0x0A:                                              // pagina configuración
                            page[5] =0;                                         // indicamos NO siguiente pagina
                            auto_page =1;
                            num_nextion((char*)"n0", siguiente_p, 0);           // n0.val=periodo en segundos se siguiente pantalla
                            num_nextion((char*)"n1", reposo, 0);                // n1.val=periodo en minutos de paso a reposo
                            num_nextion((char*)"n2", alarma_pot, 0);            // n2.val=alarma potencia máxima
                            txt_nextion((char*)"t9", (char*)"V.0.0.3");         // info. versión
                            num_nextion((char*)"n3", reposo_m, 0);              // info. minuto para reposo
                            num_nextion((char*)"n4", vcc, 0);                   // info. tensión microprocesador
                            act_reloj(1);
                        break;
                        case 0x0B:                                              // pagina configuración reloj
                            page[5] =0;                                         // indicamos NO siguiente pagina
                            auto_page =1;
                            DS1337_leer_hora_fecha(&dt);
                            num_nextion((char*)"n0", dt.horas, 0);              // n0.val=hora
                            num_nextion((char*)"n1", dt.minutos, 0);            // n0.val=minuto
                            num_nextion((char*)"n2", dt.dia_mes, 0);            // n0.val=día del mes
                            num_nextion((char*)"n3", dt.mes, 0);
                            num_nextion((char*)"n4", dt.anyo, 0);
                            num_nextion((char*)"n5", dt.dia, 0);
                            act_reloj(1);
                            ciclo_ini =1;                                       // indicamos actualizar ciclo completo fhora y fecha
                        break;
                        case 0x0C:                                              // pagina brillo
                            page[5] =0;                                         // indicamos NO siguiente pagina
                            auto_page =1;
                            act_reloj(1);
                        break;
                        case 0x0F:                                              // pagina int out
                            page[5] =0;                                         // indicamos NO siguiente pagina
                            auto_page =1;
                            act_reloj(1);
                            RX_can =0x20;                                       // dirección del nodo de entradas y salidas
                            TX_buffer_CAN[0] =out;                              // valor para el byte a enviar
                            Transmit_CANbus(RX_can, 1);                         // indicamos enviar un byte
                        break;
                    }
                break;
                case 0x24:                                                      // trama configuración
                    eeprom_tx_byte (0x00, RX_buffer[1]);                        // guardamos en eeprom en nuevo valor para siguiente pantalla
                    eeprom_tx_byte (0x01, RX_buffer[2]);                        // guardamos en eeprom en nuevo valor para reposo
                    alarma_pot =RX_buffer[3]*25;                                // obtenemos el valor para la alarma de potencia
                    eeprom_tx_byte (0x02, alarma_pot);
                    eeprom_tx_byte (0x03, alarma_pot>>8);
                    siguiente_p =RX_buffer[1];                                  // actualizamos nuevos valores
                    reposo_m =update_reposo(RX_buffer[2]);
                    OK_sound();
                    cmd_nextion((char*)"page 0");
                break;
                case 0x25:                                                      // trama configuración fecha y hora
                    dt.horas =RX_buffer[1];
                    dt.minutos =RX_buffer[2];
                    dt.segundos =0;
                    dt.dia =RX_buffer[6];
                    dt.dia_mes =RX_buffer[3];
                    dt.mes =RX_buffer[4];
                    dt.anyo =RX_buffer[5];
                    DS1337_escribir_hora_fecha(&dt);
                    act_reloj(1);
                    reposo_m =update_reposo(reposo);
                    OK_sound();
                    cmd_nextion((char*)"page 0");
                break;
                case 0x26:                                                      // trama control pulsadores
                    switch (RX_buffer[1]){
                        case 0x00:
                            out ^= (1<<0);                                      // cambia estado del bit0 para la salida 0
                        break;
                        case 0x01:
                            out ^= (1<<1);                                      // cambia estado del bit1 para la salida 1
                        break;
                        case 0x02:
                            out ^= (1<<2);                                      // cambia estado del bit2 para la salida 2
                        break;
                        case 0x03:
                            out ^= (1<<3);                                      // cambia estado del bit3 para la salida 3
                        break;
                    }
                    RX_can =0x20;
                    TX_buffer_CAN[0] =out;                                      // colocamos el valor a enviar e el registro
                    Transmit_CANbus(RX_can, 1);
                    OK_sound();
                break;
            }
            clear_buffer_uart();
        }
        // gestión modo reposo/trabajo
        if (sleep ==1){                                                         // si nos encontramos en reposo
            if (Nextion_ON ==0){                                                // y tenemos petición de pasar a trabajo
                cmd_nextion((char*)"sleep=0");                                  // ON nextion
                reposo_m =update_reposo(reposo);
                sleep =0;
                auto_page =1;
                __delay_ms(150);
                cmd_nextion((char*)"page 0");
            }
            if (Clock ==0) reloj_ON();                                          // si tenemos petición de pantalla de reloj
            if (Temp_ON ==0){
                byte_float_string(temperatura[0], temperatura[1], temperatura[2], temperatura[2]);
                temperaturas(tem_comedor);                                      // si tenemos peticion de pantalla de temperaturas
            }
        }else{                                                                  // si nos encontramos en trabajo
            if (Nextion_OFF ==0){                                               // y tenemos petición de pasar a reposo
                cmd_nextion((char*)"sleep=1");                                  // en reposo
                sleep =1;
                pulsador_on =0;
                Led_status =0;
            }
        }
        // gestión pulsador despertar/off alarma
        if (Pulsador ==1){
            OK_sound();
            do{
                __delay_ms(150);
                if (pulsador_on <34){
                    pulsador_on++;
                    if (pulsador_on ==33) OK_sound();                           // si el pulsador se encuentra pulsado unos 5 segundos en trabajo
                }
            }while (Pulsador ==1);                                              // esperamos liberación del pulsador
            if (potencia <1001){
                if (sleep ==1){
                    cmd_nextion((char*)"sleep=0");                              // ON nextion
                    sleep =0;
                    auto_page =1;
                    __delay_ms(150);
                    cmd_nextion((char*)"page 0");
                }
            }else{                                                              // en avisos o alarma, potencia mayor de 1Kw
                if (sleep !=0){                                                 // si nos encontramos en reposo
                    if (sleep ==1){                                             // comprobamos si nos encontramos en sleep 1 0 2
                        cmd_nextion((char*)"sleep=0");                          // ON nextion
                        __delay_ms(150);
                    }
                    sleep =0;
                    cmd_nextion((char*)"page 0");
                }
                auto_page =1;
            }
            if (pulsador_on ==33){
                reposo_m =update_reposo(reposo+5);                              // incrementamos en 5 minutos el tiempo de reposo
            }else
                reposo_m =update_reposo(reposo);                                // mantenemos la indicación de pulsador pulsado hasta reposo_m
            pulsador_on =1;
        }
        // gestión aviso consumo superior a 1Kw y alarma potencia
        if (potencia >1000 && pulsador_on ==0){                                 // comprobamos si tenemos un consumo superior a 1Kw
            if (sleep ==1){                                                     // si nos encontramos en reposo despertamos
                cmd_nextion((char*)"sleep=0");                                  // ON nextion
                sleep =2;                                                       // indicamos en reposo solo en aviso/alarma
                __delay_ms(150);
            }
            if (alar_on_off ==22) cmd_nextion((char*)"page 14");                // accedemos/actualizamos pagina de consumo >1Kw
            if (potencia >alarma_pot && pulsador_on ==0 && off_alarm ==0){
                alar_on_off =35;                                                // indicamos icono ON alarma
                Zumbador =Led_status;
                __delay_ms(150);
                Zumbador =0;
            }else{
                alar_on_off =36;                                                // imagen consumo >1Kw
            }
            if (ok_consumo ==1){                                                // comprobamos si tenemos consumo actualizado
                num_nextion((char*)"n0", potencia, 0);                          // n0.val=potencia W
                num_nextion((char*)"p0", alar_on_off, 1);                       // p0.pic=alarma
                num_nextion((char*)"va0", int_enchufes, 0);                     // va0.val=intensidad enchufes
                num_nextion((char*)"va1", int_cocina, 0);                       // va1.val=intensidad cocina
                num_nextion((char*)"va2", int_terraza, 0);                      // va2.val=intensidad termo, lavadora
                num_nextion((char*)"va3", int_luces, 0);                        // va3.val=intensidad luces
                num_nextion((char*)"va4", int_aire, 0);                         // va4.val=intensidad aire acondicionado
                ok_consumo =0;
                num_nextion((char*)"r5", ok_consumo, 0);                        // r5.val=ok datos actualizados
            }
            cont_segundos =0;
        }else{
            if (alar_on_off !=22){                                              // comprobamos si el estado anterior fue un aviso o alarma de potencia
                if (sleep ==2){
                    if (pulsador_on ==1){                                       // si nos encontramos en reposo, con aviso y pulsado
                        OK_sound();                                             // indicamos y salimos
                    }else{
                        cmd_nextion((char*)"sleep=1");                          // pasamos a reposo, si nos encontrábamos en reposo y no pulsamos
                    }
                    sleep =1;                                                   // si nos encontrábamos en reposo actualizamos estado
                }else{
                    cmd_nextion((char*)"page 0");
                }
                alar_on_off =22;                                                // indicamos icono OFF alarma
                off_alarm =0;
                auto_page =1;
                reposo_m =update_reposo(reposo);
            }
        }
    ClrWdt();
    }
}
