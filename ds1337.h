/* DS1337.H
Dirección I2C, en 7 bits 1101000b, escritura 11010000b -> D0h
y  lectura 11010001b -> D1h
XT de 32768 Hz, interno para DS1337C
Los 7  (del 0-6) primeros registros codificados en binario BCD, los 4 bits menos 
significativos para las unidades y los 4 bits más significativos para las decenas:

0- segundos (desde 00 hasta 59) registro 0 (el bit 7 (CH) del DS1307 esta específico)
1- minutos (desde 00 hasta 59) registro 1
2- horas (desde 00 hasta 23 o desde 00 hasta 11 y AM/PM) registro 2->bit 0-4 hora, bit 5 AM/PM, bit 6 12*24H
3- día de la semana (desde 1 hasta 7) registro 3
4- día del mes de (desde 00 hasta 31) registro 4
5- mes (desde 1 hasta 12) registro 5 (el bit 7 siglo)
6- año (desde 00 hasta 99) registro 6

del 7 al 10, registros para la alarma 1 : segundos, minutos, horas, dia de la semana o del mes
11 al 14 registros para la alarma 2 : minutos, horas, dia de la semana o del mes
15 registro de control del estado de las las salidas INTA et SQW/INTB
EOSC se debe poner a 0 para habilitar el reloj. 
bit7    bit6   bit5   bit4   bit3   bit2   bit1   bit0
EOSC     0      0      RS2    RS1   INTCN  A2IE   A1IE
                        0      0     0       X     X     1Hz
                        0      1     0       X     X     4096 Hz
                        1      0     0       X     X     8192 Hz
                        1      1     0       X     X     32768 Hz

16 registro OFS, de estado que información sobre el estado de las banderas (flag) de las alarmas 1 y 2
Se pone OSF a 1 después un defecto del oscilador, hay que escribir a 0 después, no se puede escribir un 1.
A2F para alarma 2 y A1F para la alarma 1
bit7    bit6   bit5   bit4   bit3   bit2   bit1   bit0
OSF      0      0      0      0      0     A2F    A1F
*/
#ifndef DS1337_H
#define	DS1337_H

#define bit_set(b,n) ((b) |=   (1<<(n)))                                        // bit número n a 1 del byte d  
#define bit_clear(b,n) ((b) &= (~(1<<(n))))                                     // bit número n a 0 del byte d

//Dirección I2C de escritura y lectura
#define DS1337_I2C_ESCRIBIR    0xD0
#define DS1337_I2C_LEER        0xD1

// DS1337 direcciones de los registros
#define DS1337_SEGUNDOS_REG       	0x00
#define DS1337_MINUTOS_REG        	0x01
#define DS1337_HORAS_REG          	0x02
#define DS1337_DIA_SEMANA_REG     	0x03
#define DS1337_DIA_MES_REG        	0x04
#define DS1337_MES_REG            	0x05
#define DS1337_ANYO_REG           	0x06
#define DS1337_ALM1_SEGUNDOS_REG  	0x07
#define DS1337_ALM1_MINUTOS_REG   	0x08
#define DS1337_ALM1_HORAS_REG     	0x09
#define DS1337_ALM1_DIA_SEMANA_REG  0x0A
#define DS1337_ALM2_MINUT0S_REG  	0x0B
#define DS1337_ALM2_HORAS_REG    	0x0C
#define DS1337_ALM2_DIA_SEMANA_REG  0x0D
#define DS1337_SALIDAS_CONTROL_REG  0x0E
#define DS1337_BANDERAS_ESTADO_REG  0x0F

// ON OSCILADOR, OFF SQWV, OFF Ints
#define DS1337_CTRL_REG_INIT_VAL 0x04                                           // 00000011

//Borrar banderas OSF (Oscillator Stop), A2F (Alarm 2), y A1F (Alarm 1)
#define DS1337_CLEAR_STATUS_VAL  0x00

// Estructura donde se define la los datos de la hora y fecha
typedef struct {
   unsigned char segundos;                                                      // 0 a 59
   unsigned char minutos;                                                       // 0 a 59
   unsigned char horas;                                                         // 0 a 23  (24 horas)
   unsigned char dia;                                                           // 1 = lunes.. , 7 =domingo
   unsigned char dia_mes;                                                       // 1 a 31
   unsigned char mes;                                                           // 1 a 12
   unsigned char anyo;                                                          // 00 a 99
} fecha_hora_t;
// Estructura donde se define la los datos para la alarmas
typedef struct {
   unsigned char segundo_alar1;                                                 // 0 a 59
   unsigned char minuto_alar1;                                                  // 0 a 59
   unsigned char hora_alar1;                                                    // 0 a 23  (24-horas)
   unsigned char dia_alar1;                                                     // 1 = lunes.. , 7 =domingo
   unsigned char minuto_alar2;                                                  // 0 a 59
   unsigned char hora_alar2;                                                    // 0 a 23  
   unsigned char dia_alar2;                                                     // 1 = lunes.. , 7 =domingo
} alarmas; 

fecha_hora_t dt;
alarmas al;      

//-------Escribir el dato pasado en la dirección especificada-------------------
void DS1337_escribir_byte(unsigned char direccion, unsigned char dato) {
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(direccion);
   I2C_Master_Write(dato);
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;
}
//----Leemos la dirección pasada y retornamos con el valor----------------------
unsigned char DS1337_leer_byte(unsigned char direccion) {
unsigned char dato;
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(direccion);
   I2C_Master_RepeatedStart();
   I2C_Master_Write(DS1337_I2C_LEER);
   dato = I2C_Master_Read(0);
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;
   return(dato);
}
//----Conversión de byte a BCD (dos dígitos de 0 a 99)--------------------------
unsigned char bin2bcd(unsigned char valor) {
char dato =0;
	while (valor >= 10){
        valor -= 10;
    	dato += 0x10;
    }
    return(dato += valor);
}

//------Conversión de BCD a byte (dos dígitos de 0 a 99)------------------------
char bcd2bin(char bcd_value) {
   char temp;
   temp = bcd_value;
   temp >>= 1;
   temp &= 0x78;
   return(temp + (temp >> 2) + (bcd_value & 0x0f));
}

//------------Escribir la hora y la fecha---------------------------------------
void DS1337_escribir_hora_fecha(fecha_hora_t *dt) {
unsigned char bcd_sec, bcd_min, bcd_hrs, bcd_dia, bcd_dia_mes, bcd_mes, bcd_anyo;
//Convertimos de bits a BCD
   bcd_sec  = bin2bcd(dt->segundos);
   bcd_min  = bin2bcd(dt->minutos);
   bcd_hrs  = bin2bcd(dt->horas);                                               //por defecto modo de 24 horas
   bcd_dia  = bin2bcd(dt->dia);
   bcd_dia_mes = bin2bcd(dt->dia_mes);
   bcd_mes = bin2bcd(dt->mes);                                                  // ignoramos siglo
   bcd_anyo = bin2bcd(dt->anyo);
//escribimos en DS1337
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(DS1337_SEGUNDOS_REG);   
   I2C_Master_Write(bcd_sec);
   I2C_Master_Write(bcd_min);
   I2C_Master_Write(bcd_hrs);
   I2C_Master_Write(bcd_dia);
   I2C_Master_Write(bcd_dia_mes);
   I2C_Master_Write(bcd_mes);
   I2C_Master_Write(bcd_anyo);
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;
}
//------------Escribir alarma---------------------------------------------------
void DS1337_escribir_alarma(alarmas *al) {
unsigned char bcd_sec, bcd_min, bcd_hr, bcd_dia, bcd_min_2, bcd_hr_2, bcd_dia_2;
//Convertimos de bits a BCD
   bcd_sec  = bin2bcd(al->segundo_alar1);
   bcd_min  = bin2bcd(al->minuto_alar1);
   bcd_hr  = bin2bcd(al->hora_alar1);                                           // por defecto modo de 24 horas
   bcd_dia  = bin2bcd(al->dia_alar1);
   
   bcd_min_2  = bin2bcd(al->minuto_alar2);
   bcd_hr_2  = bin2bcd(al->hora_alar2);                                         // por defecto modo de 24 horas
   bcd_dia_2  = bin2bcd(al->dia_alar2);

//Escribimos en el registro de alarmas del DS1337, alarma 1 y 2
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(DS1337_ALM1_SEGUNDOS_REG);                                  // 0x07 ->7 
   I2C_Master_Write(bcd_sec);
   I2C_Master_Write(bcd_min);
   I2C_Master_Write(bcd_hr);
   I2C_Master_Write(bcd_dia);
   I2C_Master_Write(bcd_min_2);
   I2C_Master_Write(bcd_hr_2);
   I2C_Master_Write(bcd_dia_2);                                                 // 0x0E ->14
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;
}
//----------------Leer la hora y la fecha---------------------------------------
void DS1337_leer_hora_fecha(fecha_hora_t *dt) {
unsigned char bcd_sec, bcd_min, bcd_hrs, bcd_dia, bcd_dia_mes, bcd_mes, bcd_anyo;
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(DS1337_SEGUNDOS_REG);   
   I2C_Master_RepeatedStart();
   //I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_LEER);

   bcd_sec  = I2C_Master_Read(1);
   bcd_min  = I2C_Master_Read(1);
   bcd_hrs  = I2C_Master_Read(1);
   bcd_dia  = I2C_Master_Read(1);
   bcd_dia_mes = I2C_Master_Read(1);
   bcd_mes  = I2C_Master_Read(1);
   bcd_anyo = I2C_Master_Read(0);
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;

// convertimos BCD a bits
   dt->segundos = bcd2bin(bcd_sec  & 0x7F);
   dt->minutos = bcd2bin(bcd_min  & 0x7F);
   dt->horas   = bcd2bin(bcd_hrs  & 0x3F);
   dt->dia     = bcd2bin(bcd_dia  & 0x07);
   dt->dia_mes = bcd2bin(bcd_dia_mes & 0x3F);
   dt->mes     = bcd2bin(bcd_mes  & 0x1F);
   dt->anyo    = bcd2bin(bcd_anyo);
 
   
}
//----------------Leer alarma 1 y 2 --------------------------------------------
void DS1337_leer_alarma(alarmas *al) {
unsigned char bcd_sec, bcd_min, bcd_hr, bcd_dia, bcd_min_2, bcd_hr_2, bcd_dia_2;
   INTCONbits.GIE_GIEH = 0;
   I2C_Master_Start();
   I2C_Master_Write(DS1337_I2C_ESCRIBIR);
   I2C_Master_Write(DS1337_ALM1_SEGUNDOS_REG);  								// 0x07 ->7  
   I2C_Master_RepeatedStart();
   I2C_Master_Write(DS1337_I2C_LEER);
   bcd_sec  = I2C_Master_Read(1);												// leemos registro  0x07 ->7  
   bcd_min  = I2C_Master_Read(1);												// leemos registro  0x08 ->8
   bcd_hr  = I2C_Master_Read(1);												// leemos registro  0x09 ->9
   bcd_dia  = I2C_Master_Read(1);												// leemos registro  0x0A ->10
   bcd_min_2 = I2C_Master_Read(1);												// leemos registro  0x0B ->11
   bcd_hr_2  = I2C_Master_Read(1);												// leemos registro  0x0C ->12
   bcd_dia_2 = I2C_Master_Read(0);												// leemos registro  0x0D ->13
   I2C_Master_Stop();
   INTCONbits.GIE_GIEH = 1;

// convertimos BCD a bits
   al->segundo_alar1 = bcd2bin(bcd_sec  & 0x7F);                                // 01111111
   al->minuto_alar1  = bcd2bin(bcd_min  & 0x7F);                                // 01111111
   al->hora_alar1    = bcd2bin(bcd_hr  & 0x3F);                                 // 00111111
   al->dia_alar1     = bcd2bin(bcd_dia  & 0x07);                                // 00000111
   al->minuto_alar2  = bcd2bin(bcd_min_2 & 0x7F);                               // 00111111
   al->hora_alar2    = bcd2bin(bcd_hr_2  & 0x3F);                               // 00111111				
   al->dia_alar2     = bcd2bin(bcd_dia_2 & 0x07);                               // 00000111			
}
//---Escribimos en el registro 0x07 bit 7 A1M1 para indicar si tenemos activa
// la alarma día en curso y en el registro 0x08, bit 7 A2M2 para la alarma semanal,  
// en el registro 0x0D bit 7 A2M4 se indica alarma 2 por coincidencia de hora y minuto
// y  el registro 0x0E bit 1 A2IE para ser gestionado por el 12f675 en modo pila
void TX_alama_activa(unsigned char alarma_activa){
unsigned char A1M1 =0, A1M2 =0,A2M4 =0, Control =0;
	DS1337_escribir_byte(DS1337_BANDERAS_ESTADO_REG, 0x00);                     // borramos banderas
	A1M1 = DS1337_leer_byte(DS1337_ALM1_SEGUNDOS_REG);                          // leemos el registro 0x07-> alarma día en curos 1->ON
	A1M2 = DS1337_leer_byte(DS1337_ALM1_MINUTOS_REG);                           // leemos el registro 0x08-> alarma semanal 1->ON
	A2M4 = DS1337_leer_byte(DS1337_ALM2_DIA_SEMANA_REG);                        // leemos el registro 0x0D-> alarma 2 por coincidencia hora y día (12f675)
	Control = DS1337_leer_byte(DS1337_SALIDAS_CONTROL_REG);                     // leemos el registro 0x0E-> INTA habilitado
	switch(alarma_activa){
   		case 0:	
   			bit_clear(A1M1, 7);                                                 // bit A1M1 del registro 0x07 a 0
			bit_clear(A1M2, 7);                                                 // bit A1M2 del registro 0x08 a 0
			bit_clear(A2M4, 7);                                                 // bit A2M4 del registro 0x0D a 0
			bit_clear(Control, 1);                                              // bit A2IE del registro 0x0E a 0			
			break;																
   		case 1:                                                                 // activamos alarma día en curso
			bit_set(A1M1, 7);                                                   // bit 7->A1M1 a 1, ON alarma día en curso
			bit_clear(A1M2, 7);                                                 // bit 7->A1M2 a 0, OFF alarma semanal
			bit_set(A2M4, 7);                                                   // bit 7->A2M4 a 1, para modo pila (12f675)
			bit_set(Control, 1);                                                // bit 1->A2IE a 1, para modo pila (12f675)										
			break;
		case 2:                                                                 // activamos alarma semanal
			bit_set(A1M2, 7);                                                   // bit 7->A1M2 a 1, ON alarma semanal
			bit_clear(A1M1, 7);                                                 // bit 7->A1M1 a 0, OFF alarma día en curso
			bit_set(A2M4, 7);                                                   // bit 7->A2M4 a 1, para modo pila (12f675)
			bit_set(Control, 1);                                                // bit 1->A2IE a 1, para modo pila (12f675)	
			break;
	}
	DS1337_escribir_byte(DS1337_ALM1_SEGUNDOS_REG, A1M1); 
	DS1337_escribir_byte(DS1337_ALM1_MINUTOS_REG, A1M2);
	DS1337_escribir_byte(DS1337_ALM2_DIA_SEMANA_REG, A2M4);
	DS1337_escribir_byte(DS1337_SALIDAS_CONTROL_REG, Control);	
}
//Inicializa después de un fallo en el oscilador o restaura el registro de control
void DS1337_init(void) {
unsigned char OSF = 0, Control =0;
//Leemos el registro de estado para ver si el oscilador dejo de funcionado.
    OSF = DS1337_leer_byte(DS1337_BANDERAS_ESTADO_REG);
//obtenemos el bit OSF 
    OSF = OSF >> 7;
//leemos el registro de control
	Control = DS1337_leer_byte(DS1337_SALIDAS_CONTROL_REG);
	if(Control){
		DS1337_escribir_byte(DS1337_SALIDAS_CONTROL_REG ,0x00);			        // clok 1 Hz ON y alarmas OFF
	}
//si el bit se encuentra a 1 inicializamos valores por defecto
    if(OSF) {                                                                   // Con DS1307->OSF ==0 para ds1337 -> if(OSF)
      I2C_Master_Start();
      I2C_Master_Write(DS1337_I2C_ESCRIBIR);
      I2C_Master_Write(DS1337_SEGUNDOS_REG);
      I2C_Master_Write(0x00);                                                   // segundos
      I2C_Master_Write(0x00);                                                   // minutos
      I2C_Master_Write(0x0C);                                                   // 12->0 0 0 01100, 12:00 hora, AM, 24H
      I2C_Master_Write(0x01);                                                   // dia
      I2C_Master_Write(0x01);                                                   // dia_mes
      I2C_Master_Write(0x01);                                                   // mes
      I2C_Master_Write(0x18);                                                   // anyo ->18
      I2C_Master_Write(0x00);                                                   // alarma1 segundos
      I2C_Master_Write(0x00);                                                   // alarma11 minutos
      I2C_Master_Write(0x00);                                                   // alarma1 horas
      I2C_Master_Write(0x00);                                                   // alarma1 dia/dia_mes
      I2C_Master_Write(0x00);                                                   // alarma2 minutos
      I2C_Master_Write(0x00);                                                   // alarma2 horas
      I2C_Master_Write(0x00);                                                   // alarma2 dia/dia_mes
      I2C_Master_Write(DS1337_CTRL_REG_INIT_VAL);                               // Salida oscilador a 1hz, alarmas apagadas
      I2C_Master_Write(DS1337_CLEAR_STATUS_VAL);                                // borramos registro estado
      I2C_Master_Stop();
   }
} 
#endif	/* DS1337_H */
