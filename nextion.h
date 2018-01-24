/*
 Nota:
 Todas las instrucciones y parámetros se envían a la pantalla nextion en ascii
 y minúsculas y la instrucción finaliza con tres bytes "0xff 0xff 0xff"
 * 
 * * Author: Radioelf
 
 */
#ifndef NEXTION_H
#define	NEXTION_H

//#define ACK_on                                                                //configuramos el tipo de datos que devolverá nextion, solo cuando el último comand serie fue correcto

#ifdef ACK_on
    const unsigned char *ack = "bkcmd=1";                                       // es necesario tener habilitada interrupción por RX datos en UART antes de ini_nextion()
#else
    const unsigned char *ack = "bkcmd=0";
#endif
    
unsigned char  RX_con =0, RX_off =0, RX_buffer[10] ={0};                        // 0-255
//******************************************************************************
void end_nextion(void){
    putch(0xFF);
    putch(0xFF);
    putch(0xFF);
}
//******************************************************************************
#ifdef ACK_on
unsigned char ack_nextion(){
unsigned char espera =255, ack =0;
    while ((RX_off !=3) && espera){                                             //esperamos final trama o desbordamiento espera                                                       
        __delay_us(100);
        ClrWdt();
        espera--;                                                    
    }
    if (RX_buffer[0] ==0x01) ack =1;
    RX_off =0;
    RX_buffer[RX_con-1] =0;
    RX_con =0;
    return ack;                                                                 
}
#endif
//******************************************************************************
unsigned char ini_nextion(){
    UART_Write_Text((char*) "");
    end_nextion();
    UART_Write_Text((char*)"sleep=0");                                          
    end_nextion();
    UART_Write_Text((char*) ack); 
    end_nextion();
    #ifdef ACK_on
        return ack_nextion();
    #else
        RCIF = 0;
        return 1;
    #endif
}
//******************************************************************************
unsigned char cmd_nextion(char *cmd){
    UART_Write_Text(cmd);
    end_nextion();
    #ifdef ACK_on
        return ack_nextion();
    #else
        return 1;
    #endif
}
//******************************************************************************
unsigned char txt_nextion(char* txt,  char* valor){
    UART_Write_Text(txt);
    UART_Write_Text((char*) ".txt=");
    putch(0x22);                                                                // comillas
    UART_Write_Text (valor);
    putch(0x22);
    end_nextion();
    #ifdef ACK_on
        return ack_nextion();
    #else
        return 1;
    #endif
}
//******************************************************************************
unsigned char num_nextion(char* num,  unsigned int valor, unsigned char tipo){
    UART_Write_Text(num);
    if (tipo)
        UART_Write_Text((char*) ".pic=");
    else
        UART_Write_Text((char*) ".val=");
    if (valor >9)
        UART_Num_Ascii (valor);
    else
        putch(valor+48);
    end_nextion();
    #ifdef ACK_on
        return ack_nextion();
    #else
        return 1;
    #endif
}
//******************************************************************************
#endif	/* NEXTION_H */
