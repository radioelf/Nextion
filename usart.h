////////////////////////////////////////////////////////////////////
//Archivo de cabecera para el uso del módulo USART PIC con  XC8  ///
////////////////////////////////////////////////////////////////////

#ifndef USART_H
#define	USART_H
 
void UART_Init(unsigned int  spbrg);                                                   
char getch(void);
void UART_Read_Text(char *Output, unsigned int length);
void putch(char data);
void UART_Write_Text(char *text);
char UART_TX_Empty(void);
char UART_Data_Ready();
 
////////////////////////////////////////////////////////////////////
//inicialización del módulo USART PIC modo asíncrono, a 8bits    ///
////////////////////////////////////////////////////////////////////
void  UART_Init(unsigned int spbrg){
    BRG16 =1;
    BRGH = 1; 
    SPBRG = spbrg;                                                              // Writing SPBRG Register
    SPBRGH= spbrg>>8;
    SYNC = 0;                                                                   // Setting Asynchronous Mode, ie UART
    SPEN = 1;                                                                   // Enables Serial Port
    TRISC7 = 1;                                                                 // As Prescribed in Datasheet
    TRISC6 = 1;                                                                 // As Prescribed in Datasheet
    CREN = 1;                                                                   // Enables Continuous Reception
    TXEN = 1;                                                                   // Enables Transmission
}
 
/////////////////////////////////////////////////////////////////////
//recepción de un carácter del módulo USART PIC modo asíncrono    ///
/////////////////////////////////////////////////////////////////////
char getch(void){
    while(!RCIF);
    return RCREG;
}
/////////////////////////////////////////////////////////////////////
//leer cadena de caracteres con el módulo USART PIC modo asíncrono///
/////////////////////////////////////////////////////////////////////
void UART_Read_Text(char *Output, unsigned int length){
    unsigned int x;
    for(x =0 ; x <length; x++){
        Output[x] = getch();
    }
}
/////////////////////////////////////////////////////////////////////
//transmisión de un carácter del módulo USART PIC modo asíncrono  ///
/////////////////////////////////////////////////////////////////////
void putch(char data){
    while(!TRMT);                                                               // mientras el registro TSR está lleno espera
    TXREG = data;                                                               // cuando el el registro TSR está vació se enviá el carácter
}

/////////////////////////////////////////////////////////////////////
//transmisión de un cadena de caracteres del módulo USART PIC modo asíncrono
///////////////////////////////////////////////////////////////////// 
void UART_Write_Text(char *text){
    while(*text){
        putch(*text++);
    }
}

/////////////////////////////////////////////////////////////////////
// Comprobación del registro de transmisión                       ///
/////////////////////////////////////////////////////////////////////
char UART_TX_Empty(void){
    return TRMT;
}

/////////////////////////////////////////////////////////////////////
// Comprobamos si podemos leer                                    ///
/////////////////////////////////////////////////////////////////////
char UART_Data_Ready(){
  return RCIF;
}

void UART_Num_Ascii(unsigned int num){
unsigned int  valor = num;
        if (valor >9999){
            putch((num/10000)+0x30);                                            // cinco dígitos
            num=num%10000;
        }
        if (valor >999){  
            putch((num/1000)+0x30);                                             // cuatro dígitos
            num=num%1000;
        }
        if (valor >99){
            putch((num/100)+0x30);                                              // tres dígitos
            num=num%100;
        }
    if (valor >9)putch((num/10)+0x30);                                          // dos dígitos
    putch((num%10)+0x30);                                                       // un dígito
}
#endif	/* USART_H */


