/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */
  
#ifndef I2C_H
#define	I2C_H 

void I2C_Master_Init(unsigned char x);                                          // función para iniciar el módulo SSP en modo I2C 
void I2C_Master_Wait();                                                         // función de espera
void I2C_Master_Start();                                                        // función que inicia la comunicación I2C 
void I2C_Master_RepeatedStart();                                                // función que reinicia la comunicación I2C 
void I2C_Master_Stop();                                                         // función que detiene la comunicación I2C 
unsigned char  I2C_Master_Write(unsigned char data);                            // función escribir datos al esclavo
unsigned char I2C_Master_Read(unsigned char ack_nak);                           // función para leer datos del esclavo

void I2C_Master_Init(unsigned char x){
    SSPCON1 = 0b00101000;                                                       // I2C PIC habilitado en modo maestro velocidad=Fosc/(4*SSPADD+1)
    SSPCON2 = 0b00000000;                                                       // comunicación I2C no iniciada
    SSPADD  = x;                                                                // config. velocidad
    SSPSTAT = 0b00000000;                                                       
}

void I2C_Master_Wait(){
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));                               // espera que se cumplan las condiciones adecuadas
}

void I2C_Master_Start(){
    I2C_Master_Wait();                                                          
    SSPCON2bits.SEN = 1;                                                        // inicia la comunicación i2c
}

void I2C_Master_RepeatedStart(){
    I2C_Master_Wait();
    SSPCON2bits.RSEN = 1;                                                       // reinicia la comunicación i2c
}

void I2C_Master_Stop(){
    I2C_Master_Wait();
    SSPCON2bits.PEN = 1;                                                        // detener la comunicación i2c
}

unsigned char  I2C_Master_Write(unsigned char data){                            // master mode
    I2C_Master_Wait();
    SSPBUF = data;                                                              // cargar en el registro SSPBUF el dato a enviar
    while (BF == 1);                                                            // esperemos hasta que el ciclo de escritura esté completo 
    return SSPCON2bits.ACKSTAT;                                                 // devuelve un 0->ACK, 1->NACK                                                
}

unsigned char I2C_Master_Read(unsigned char ack_nak){
unsigned char temp;
    I2C_Master_Wait();
    RCEN =1;                                                                    // reinicia la comunicación i2c
    I2C_Master_Wait();
    temp = SSPBUF;                                                              // obtenemos el dato
    I2C_Master_Wait();
    ACKDT = (ack_nak)?0:1;                                                      // ACK -> 0 se seguirán recibiendo más datos, NAK -> 1 esclavo debe dejar de enviar datos
    ACKEN = 1;                                                                  // fin RX
    return temp;
}

#endif	/* I2C_H */

