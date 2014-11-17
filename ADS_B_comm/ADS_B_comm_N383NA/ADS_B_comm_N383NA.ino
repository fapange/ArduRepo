//Conor Geary
//6/9/2014

//ADS-B Heartbeat Comm Maintainer
//
//Sends an intermittent signal to the ADS-B that
//mimics a gps display

//ADS-B heartbeat message
byte message[26];
int len = 26;
//Transmit flag
unsigned char txFlag = 0;
//Timer state 
unsigned char count = 0;
int led = 0;
unsigned int Crc16Table[256];

void crcInit( void ){
  unsigned int i, bitctr, crc;
  for (i = 0; i < 256; i++){
    crc = (i << 8);
    for (bitctr = 0; bitctr < 8; bitctr++){
      crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
    }
    Crc16Table[i] = crc;
  }
}

unsigned int crcCompute(unsigned char *block,unsigned long int length){
  unsigned long int i;
  unsigned int crc = 0;
  for (i = 0; i < length; i++){
    crc = Crc16Table[crc >> 8] ^ (crc << 8) ^ block[i];
  }
  return crc;
}
//Test
void setup(){
  //Serial Debug to PC
  //Serial.begin(9600);
  
  //Timing Test LED
  pinMode(13, OUTPUT);
  crcInit();
  //Serial Connection to the ADS-B
  Serial.begin(38400);
  message[0] = 0x7E;  //Message flag byte
  message[1] = 139;   //Message ID
  //START Message Data
  
  //Flight ID: 1200 ( base_8 )
  message[2] = 0x00;//Flight ID MSB
  message[3] = 0x02;//Flight ID 
  message[4] = 0x80;//FLight ID LSB
  
  message[5] = 0;//TransMonSPE Software Version Number //Unused here
  
  message[6] = B00000000;//Reserved
  message[7] = B00000000;//Reserved
  message[8] = B00000001;//Bits 7 - 1 reserved -- Bit 0: Enable SAA
  message[9] = B00000000;//Bits 7 - 1 reserved -- Bit 0: IDENT flag //Do not set IDENT flag here, it is meant to be used from a pilot interface
  
  //Callsign Bytes
  //Must each be 48 - 57 or 65 - 90
  //If no callsign available set Character 0 to 32
  //Callsign: N383NA ( ASCII )
  message[10] = 78;
  message[11] = 51;
  message[12] = 56;
  message[13] = 51;
  message[14] = 78;
  message[15] = 65;
  message[16] = 32;
  message[17] = 32;
  
  //3 Byte ICAO Address
  //Hex:     A4679C
  //Octal:   51061745
  //Binary: 101001000110001111100101
  message[18] = 0xA4;
  message[19] = 0x67;
  message[20] = 0x9C;
  
  //Bits 7 - 2 reserved -- Bits 1 - 0 UAT Transmitter State
  message[21] = 2;//UAT Transmitter On
  //Bits 7 - 3 reserved -- Bits 2 - 0 Emergency Code
  message[22] = 0;//No emergency
  //END Message Data
  
  unsigned int crc;
  crc = crcCompute(&message[
  2], 21);
  
  //Frame Check Sequence
  message[23] = crc;
  message[24] = (crc >> 8);
  message[25] = 0x7E; //Message flag byte
  
  //Set up timer interrupt for 1 second interval
  
  //Disable global interrupts
  cli();
  
  //Timer 1 Compare Match A
  TCCR1A = B00000000;
  
  //Prescaler 256
  TCCR1B = B00001100;
  
  //Enable Timer Compare A Interrupt
  TIMSK1 = B00000010;
  TCNT1H = 0;
  TCNT1L = 0;
  
  //Set A compare value to 62500
  OCR1AH = 0xF4;
  OCR1AL = 0x24;
  
  //Reenable global interrupts
  sei();
}


void loop(){
  //If the timier interrupt triggers, send the tC panel control message
  if(txFlag == 1){
    //Timing LED status counter
    count ++;
    
    //Prevent overflow
    if(count > 2){
      count = 1;
    }
    
    //Toggle LED state per iteration
    digitalWrite(13, count%2);
    
    //Reset the transmit flag
    txFlag = 0;
    
    //Write TC control message to the Serial buffer
    Serial.write(message, len);
    //Wait until everything has been sent out of the buffer
    Serial.flush();
  }
}

//One second timer interrupt
ISR(TIMER1_COMPA_vect){
  //Set the transmit flag
  txFlag = 1;
}
