/* 
 * File:   ecan.h
 * Author: d.lohse
 *
 * Created on 2. Juli 2014, 16:45
 * 
 * Mod. Radioelf http://radioelf.blogspot.com.es/
 */

#ifndef ECAN_H
#define ECAN_H

//#define MY_CAN_ADDR 0x200                                                     // 00000000 00000010 00000000 00000000

#define ECANGetOperationMode() (CANCON & ECAN_OP_MODE_BITS)                     // obtenemos el modo actual

 // Macros para ajuste bufer RX
#define ECANLinkRXF0F1ToBuffer(RXF0Buffer, RXF1Buffer)      RXFBCON0 = (RXF1Buffer << 4) | RXF0Buffer
#define ECANLinkRXF2F3ToBuffer(RXF2Buffer, RXF3Buffer)      RXFBCON1 = (RXF2Buffer << 4) | RXF3Buffer
#define ECANLinkRXF4F5ToBuffer(RXF4Buffer, RXF5Buffer)      RXFBCON2 = (RXF4Buffer << 4) | RXF5Buffer
#define ECANLinkRXF6F7ToBuffer(RXF6Buffer, RXF7Buffer)      RXFBCON3 = (RXF6Buffer << 4) | RXF7Buffer
#define ECANLinkRXF8F9ToBuffer(RXF8Buffer, RXF9Buffer)      RXFBCON4 = (RXF8Buffer << 4) | RXF9Buffer
#define ECANLinkRXF10F11ToBuffer(RXF10Buffer, RXF11Buffer)  RXFBCON5 = (RXF10Buffer << 4) | RXF11Buffer
#define ECANLinkRXF12F13ToBuffer(RXF12Buffer, RXF13Buffer)  RXFBCON6 = (RXF12Buffer << 4) | RXF13Buffer
#define ECANLinkRXF14F15ToBuffer(RXF14Buffer, RXF15Buffer)  RXFBCON7 = (RXF14Buffer << 4) | RXF15Buffer

// Modos para CAN 0,1 o 2
#define ECAN_MDSEL_MODE0_LEGACY     0b00000000
#define ECAN_MDSEL_MODE1_ENHANCED   0b01000000
#define ECAN_MDSEL_MODE2_FIFO       0b10000000

#define ECAN_BSEL_B0_TXEN           0b00000100
#define ECAN_BSEL_B1_TXEN           0b00001000
#define ECAN_BSEL_B2_TXEN           0b00010000
#define ECAN_BSEL_B3_TXEN           0b00100000
#define ECAN_BSEL_B4_TXEN           0b01000000
#define ECAN_BSEL_B5_TXEN           0b10000000

#define ECAN_SJW_4TQ                0b11000000
#define ECAN_SJW_3TQ                0b10000000
#define ECAN_SJW_2TQ                0b01000000
#define ECAN_SJW_1TQ                0b00000000

#define ECAN_SEG2PHTS_FP            0b10000000  /* use Phase Segment 2 Time Select bits in BRGCON3 */
#define ECAN_SAM_ONCE               0b00000000
#define ECAN_SAM_3X                 0b01000000

#define ECAN_SEG1PH_8TQ             0b00111000
#define ECAN_SEG1PH_7TQ             0b00110000
#define ECAN_SEG1PH_6TQ             0b00101000
#define ECAN_SEG1PH_5TQ             0b00100000
#define ECAN_SEG1PH_4TQ             0b00011000
#define ECAN_SEG1PH_3TQ             0b00010000
#define ECAN_SEG1PH_2TQ             0b00001000
#define ECAN_SEG1PH_1TQ             0b00000000

#define ECAN_PRSEG_8TQ              0b00000111
#define ECAN_PRSEG_7TQ              0b00000110
#define ECAN_PRSEG_6TQ              0b00000101
#define ECAN_PRSEG_5TQ              0b00000100
#define ECAN_PRSEG_4TQ              0b00000011
#define ECAN_PRSEG_3TQ              0b00000010
#define ECAN_PRSEG_2TQ              0b00000001
#define ECAN_PRSEG_1TQ              0b00000000

#define ECAN_SEG2PH_8TQ             0b00000111
#define ECAN_SEG2PH_7TQ             0b00000110
#define ECAN_SEG2PH_6TQ             0b00000101
#define ECAN_SEG2PH_5TQ             0b00000100
#define ECAN_SEG2PH_4TQ             0b00000011
#define ECAN_SEG2PH_3TQ             0b00000010
#define ECAN_SEG2PH_2TQ             0b00000001
#define ECAN_SEG2PH_1TQ             0b00000000

// Variable glovales
unsigned char temp_EIDH;                                                        // Identificador extendido, byte alto
unsigned char temp_EIDL;                                                        // Identificador extendido, byte bajo
unsigned char temp_SIDH;                                                        // Identificador estándar byte alto
unsigned char temp_SIDL;                                                        // Identificador estándar byte bajo
unsigned char temp_DLC;                                                         // Longitud de datos de control, 0 a 8
unsigned char temp_D0;                                                          // Data byte 0 
unsigned char temp_D1;
unsigned char temp_D2;
unsigned char temp_D3;
unsigned char temp_D4;
unsigned char temp_D5;
unsigned char temp_D6;
unsigned char temp_D7;                                                          // Data byte 7

// ECAN_OP_MODE
// Estos valores de enumeración definen códigos relacionados con el módulo ECAN
// modo de operación. La rutina ECANSetOperationMode () requiere este código.
typedef enum _ECAN_OP_MODE{
    ECAN_OP_MODE_BITS    = 0xe0,                                                // para acceder a los bits opmode
    ECAN_OP_MODE_NORMAL  = 0x00,
    ECAN_OP_MODE_SLEEP   = 0x20,
    ECAN_OP_MODE_LOOP    = 0x40,
    ECAN_OP_MODE_LISTEN  = 0x60,
    ECAN_OP_MODE_CONFIG  = 0x80
} ECAN_OP_MODE;

char TX_buffer_CAN[8] = {0};

void ECANSetOperationMode(ECAN_OP_MODE mode);
void Init_CANbus(void);
unsigned char Transmit_CANbus(long address, char size);
unsigned char Receive_CANbus(void);

// Cambiar el modo
void ECANSetOperationMode(ECAN_OP_MODE mode){
    CANCON &= 0x1F;                                                             // borramos modo actual
    CANCON |= mode;                                                             // pasamos al modo seleccionado
    while( ECANGetOperationMode() != mode );                                    // esperamos que el modo seleccionado se encuentre activo.
}
// Inicializar modulo CAN
void Init_CANbus(void){
    ECANSetOperationMode(ECAN_OP_MODE_CONFIG);                                  // pasamos a modo configuración
    ECANCON = ECAN_MDSEL_MODE0_LEGACY;                                          // cambio al Modo 0, standard, ->0x00
    // Fosc = 32 ->cristal de 8 Mhz x4 = 32 Mhz-> 0.03125uS con el bit de configuración HSPLL activo
    // BRP = 7 -> 125Kbs
    // Tq = (2*(BRP+1))/Fosc -> (2*8)/32= 0.5us 

    BRGCON1 = ECAN_SJW_3TQ | 7;                                                 // SJW=3TQ,BRP=7
    BRGCON2 = ECAN_SEG2PHTS_FP | ECAN_SAM_ONCE | ECAN_SEG1PH_8TQ | ECAN_PRSEG_1TQ;
    BRGCON3 = ECAN_SEG2PH_6TQ;
    //BSEL0 = ECAN_BSEL_B0_TXEN;                                                // Configura el bufer en el modo de transmisión ajustando el bit TXnEN a '1' en el registro BSEL0.
    // Inicializar mascaras de recepción
    // la mascara 0 (M0) NO aceptar direcciones extendidas, pero SI todas las direcciones estándar
    RXM0EIDH = 0x00;                                                            // mascara de aceptación de recepción extendida, byte alto 
    RXM0EIDL = 0x00;                                                            // mascara de aceptación de recepción extendida, byte bajo
    RXM0SIDH = 0b11111111;                                                      // mascara de aceptación de recepción estándar, byte alto->11111111
    RXM0SIDL = 0b11100000;                                                      // mascara de aceptación de recepción estándar, byte bajo->11100000
    // la mascara 1 (M1) NO aceptar direcciones extendidas, pero SI todas las direcciones estándar
    RXM1EIDH = 0x00;                                                            // mascara de aceptación de recepción extendida, byte alto    
    RXM1EIDL = 0x00;                                                            // mascara de aceptación de recepción extendida, byte bajo
    RXM1SIDH = 0b11111111;                                                      // mascara de aceptación de recepción estándar, byte alto->11111111
    RXM1SIDL = 0b11100000;                                                      // mascara de aceptación de recepción estándar, byte bajo->11100000
    // El modo 0 permite el uso de filtros de recepción RXF0 a RXF5
    RXFCON0 = 0x03;                                                             // habilitamos filtro-0 y filtro-1, resto deshabilitados 
    RXFCON1 = 0x00;                                                             // deshabilitamos filtros de 8 al 15
    
    //  Filtro 0 = 0x0A->00000001 010 (0x01->00000001-0x40->010 00000) C
    //  Filtro 1 = 0x19E->0110011 110 (0x33->00110011-0xC0->110 00000)
    RXF0EIDH = 0x00;                                                            // Filtro de direcciones extendido-0 no utilizado, ajuste el byte alto a 0
    RXF0EIDL = 0x00;                                                            // Filtro de direcciones extendido-0 no utilizado, ajuste el byte bajo a 0
    //RXF0SIDH = 0x01;                                                            // Filtro de dirección estándar-0 byte alto configurado a ->00000001 
    //RXF0SIDL = 0x40;                                                            // Filtro de dirección estándar-0 byte bajo configurado a ->01000000
    RXF0SIDH = 0x00;                                                            // Filtro de dirección estándar-0 byte alto configurado a ->00000001 
    RXF0SIDL = 0x00;                                                            // Filtro de dirección estándar-0 byte bajo configurado a ->01000000

    RXF2EIDH = 0x00;                                                            // Filtro de direcciones extendido-1 no utilizado, ajuste el byte alto a 0
    RXF2EIDL = 0x00;                                                            // Filtro de direcciones extendido-1 no utilizado, ajuste el byte bajo a 0
    RXF2SIDH = 0x33;                                                            // Filtro de dirección estándar-0 byte alto configurado a 0x33
    RXF2SIDL = 0xC0;                                                            // Filtro de dirección estándar-0 byte bajo configurado a 0xC0
    
    ECANSetOperationMode(ECAN_OP_MODE_NORMAL);                                  // pasamos al modo normal
    // ajustamos los modos de recepción para los buferes del recepción 0 y 1
    RXB0CON = 0x00;                                                             
    RXB1CON = 0x00;
}

unsigned char Receive_CANbus(void){
    if (RXB0CONbits.RXFUL){                                                     // comprobamos la bandera de RX bufer 0 si tenemos datos
        temp_EIDH = RXB0EIDH;
        temp_EIDL = RXB0EIDL;
        temp_SIDH = RXB0SIDH;
        temp_SIDL = RXB0SIDL;
        temp_DLC =  RXB0DLC;
        temp_D0 =   RXB0D0;
        temp_D1 =   RXB0D1;
        temp_D2 =   RXB0D2;
        temp_D3 =   RXB0D3;
        temp_D4 =   RXB0D4;
        temp_D5 =   RXB0D5;
        temp_D6 =   RXB0D6;
        temp_D7 =   RXB0D7;
        RXB0CONbits.RXFUL = 0;                                                  // borramos bandera
        return 1;                                                               // indicamos dados recibidos bufer 0
    }else if (RXB1CONbits.RXFUL){                                               // comprobamos la bandera de RX bufer 1 si tenemos datos
        temp_EIDH = RXB1EIDH;
        temp_EIDL = RXB1EIDL;
        temp_SIDH = RXB1SIDH;
        temp_SIDL = RXB1SIDL;
        temp_DLC =  RXB1DLC;
        temp_D0 =   RXB1D0;
        temp_D1 =   RXB1D1;
        temp_D2 =   RXB1D2;
        temp_D3 =   RXB1D3;
        temp_D4 =   RXB1D4;
        temp_D5 =   RXB1D5;
        temp_D6 =   RXB1D6;
        temp_D7 =   RXB1D7;
        RXB1CONbits.RXFUL = 0;                                                  // borramos bandera
        return 2;                                                               // indicamos dados recibidos bufer 1
    }else if (B0CONbits.RXFUL_TXBIF){                                           // CheckB0
        temp_EIDH = B0EIDH;
        temp_EIDL = B0EIDL;
        temp_SIDH = B0SIDH;
        temp_SIDL = B0SIDL;
        temp_DLC = B0DLC;
        temp_D0 = B0D0;
        temp_D1 = B0D1;
        temp_D2 = B0D2;
        temp_D3 = B0D3;
        temp_D4 = B0D4;
        temp_D5 = B0D5;
        temp_D6 = B0D6;
        temp_D7 = B0D7;      
        B0CONbits.RXFUL_TXBIF = 0;
        return 3;
    }
    return 0;                                                                   // indicamos NO hemos recibido datos    
}

unsigned char Transmit_CANbus(long address, char size){
unsigned char x =0;
    if (TXB0CONbits.TXREQ ==0){                                                 // solo enviamos si se a finalizado el ultimo envió
        TXB0EIDH = 0x00;
        TXB0EIDL = 0x00;

        // 29bits
        if (address > 0x7FF){                                                   // para modo extendido
            //TXB0EIDH = (unsigned char) (address &0xFF000000)>>24;
            //TXB0EIDL = (unsigned char) (address &0x00FF0000)>>16;
            TXB0EIDL = (unsigned char)(address & 0xFF);	
            TXB0EIDH = (unsigned char)((address >> 8) & 0xFF);
            TXB0SIDL = (unsigned char)((address >> 16) & 0x03) | (unsigned char)((0x00 >> 13) & 0xE0) | 0x08;
            TXB0SIDH = (unsigned char)((address >> 21) & 0xFF);            
        }else{
        // STANDARD 11 bits
            TXB0SIDH = (unsigned char) address >> 3;                            // dirección del nodo CAN
            TXB0SIDL = (unsigned char) address << 5;
        }
        TXB0DLC = size;                                                         // longitud de TX

        TXB0D7 = TX_buffer_CAN[7];
        TXB0D6 = TX_buffer_CAN[6]; 
        TXB0D5 = TX_buffer_CAN[5];
        TXB0D4 = TX_buffer_CAN[4]; 
        TXB0D3 = TX_buffer_CAN[3]; 
        TXB0D2 = TX_buffer_CAN[2]; 
        TXB0D1 = TX_buffer_CAN[1]; 
        TXB0D0 = TX_buffer_CAN[0];

        TXB0CONbits.TXREQ = 1;                                                  // iniciamos transmisión
        while (TXB0CONbits.TXREQ){                                              // esperamos que finalice
            __delay_ms(1);
            if (x++ ==5) return 0;
        }
        return 1;
    }else{
        return 0;
    }
}

#endif  /* ECAN_H */
