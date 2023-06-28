#include <SPI.h>
#include <nRF24L01.h>
#define STATUS 0x07
#define CONFIG 0x00
void CS_Select (void)
{
  digitalWrite(8, LOW);
}

void CS_UnSelect (void)
{
  digitalWrite(8, HIGH);
}


void CE_Enable (void)
{

  digitalWrite(7, HIGH);
}

void CE_Disable (void)
{
  digitalWrite(7, LOW);
}

// write a single byte to the particular register
/*void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{ //SPI.begin();
  uint8_t buf[2];
  buf[0] =Reg|1<<5;
  buf[1] =Data;
  Serial.print(buf[0]);

  // Pull the CS Pin LOW to select the device
  CS_Select();
  SPI.transfer(buf, 2);
  // Pull the CS HIGH to release the device
  CS_UnSelect();
}

//write multiple bytes starting from a particular register
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int sized)
{
  uint8_t buf;
  buf=Reg|1 << 5;

  // Pull the CS Pin LOW to select the device
  CS_Select();
 // SPI.begin();
  SPI.transfer(buf);
  SPI.transfer(data, sized);
  CS_UnSelect();
}*/


/** Write a value to a multi-byte register
 * @param reg The register to write
 * @param value pointer to the value to write
 * @param size number of bytes to write
 */
void nrf24_WriteRegMulti(uint8_t reg, const void *value, uint8_t sized)
{
    if (sized) {
  reg |= W_REGISTER;
    }
    //digitalWrite(chipSelectPin, CHIP_SELECT);
    CS_Select();
    SPI.transfer(reg);
    if (sized) {
  do {
      sized--;
      SPI.transfer(((uint8_t *)value)[sized]);
  } while (sized);
    }
   CS_UnSelect();
}
void nrf24_WriteReg(uint8_t reg, uint8_t value)
{
   nrf24_WriteRegMulti(reg, &value, 1);
}


/*uint8_t nrf24_ReadReg (uint8_t Reg)
{
  uint8_t data = 0;
  CS_Select();
  //SPI.begin();
  SPI.transfer(&Reg);
  SPI.transfer(&data, 1);
  CS_UnSelect();

  return data;
}*/

/* Read multiple bytes from the register */
/*void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int sized)
{
  // Pull the CS Pin LOW to select the device
  CS_Select();
  SPI.transfer(&Reg, 1);
  SPI.transfer(data, sized);


  // Pull the CS HIGH to release the device
  CS_UnSelect();
}*/
void nrf24_ReadReg_Multi (uint8_t reg, uint8_t *value, int sized)
{
     CS_Select();
    SPI.transfer(reg);
    do {
  sized--;
  ((uint8_t *)value)[sized] = SPI.transfer(0);
    } while (sized);
    CS_UnSelect();
}
uint8_t nrf24_ReadReg (uint8_t reg)
{
    uint8_t data;
    nrf24_ReadReg_Multi(reg, &data, 1);
    return data;
}

// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
  // Pull the CS Pin LOW to select the device
  CS_Select();
  //SPI.begin();
  // HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
  SPI.transfer(&cmd);

  // Pull the CS HIGH to release the device
  CS_UnSelect();
}

void nrf24_reset(uint8_t REG)
{
  if (REG == STATUS)
  {
    nrf24_WriteReg(STATUS, 0x00);
  }

  else if (REG == FIFO_STATUS)
  {
    nrf24_WriteReg(FIFO_STATUS, 0x11);
  }

  else {
    nrf24_WriteReg(CONFIG, 0x08);
    nrf24_WriteReg(EN_AA, 0x3F);
    nrf24_WriteReg(EN_RXADDR, 0x03);
    nrf24_WriteReg(SETUP_AW, 0x03);
    nrf24_WriteReg(SETUP_RETR, 0x03);
    nrf24_WriteReg(RF_CH, 0x02);
    nrf24_WriteReg(RF_SETUP, 0x0E);
    nrf24_WriteReg(STATUS, 0x00);
    nrf24_WriteReg(OBSERVE_TX, 0x00);
    nrf24_WriteReg(CD, 0x00);
    uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
    uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
    nrf24_WriteReg(RX_ADDR_P2, 0xC3);
    nrf24_WriteReg(RX_ADDR_P3, 0xC4);
    nrf24_WriteReg(RX_ADDR_P4, 0xC5);
    nrf24_WriteReg(RX_ADDR_P5, 0xC6);
    uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
    nrf24_WriteReg(RX_PW_P0, 0);
    nrf24_WriteReg(RX_PW_P1, 0);
    nrf24_WriteReg(RX_PW_P2, 0);
    nrf24_WriteReg(RX_PW_P3, 0);
    nrf24_WriteReg(RX_PW_P4, 0);
    nrf24_WriteReg(RX_PW_P5, 0);
    nrf24_WriteReg(FIFO_STATUS, 0x11);
    nrf24_WriteReg(DYNPD, 0);
    nrf24_WriteReg(FEATURE, 0);
  }
}




void NRF24_Init()
{
  // disable the chip before configuring the device
  CE_Disable();


  // reset everything
  nrf24_reset (0);

  nrf24_WriteReg(CONFIG, 0);  // will be configured later

  nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

  nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

  nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

  nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

  nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

  nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

  // Enable the chip after configuring the device
  CE_Enable();

}


// set up the Tx mode

void NRF24_TxMode(uint8_t *Address, uint8_t channel)
{
  // disable the chip before configuring the device
  CE_Disable();

  nrf24_WriteReg (RF_CH, channel);  // select the channel

  nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Write the TX address


  // power up the device
  uint8_t val = nrf24_ReadReg(CONFIG);
  val= val&(0xF2);  // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
  nrf24_WriteReg (CONFIG, val);
  //Serial.print("wtf");

  // Enable the chip after configuring the device
  CE_Enable();
}


// transmit the data

uint8_t NRF24_Transmit (uint8_t *data)
{
  uint8_t cmdtosend = 0;
  //SPI.begin();
  // select the device
  CS_Select();

  // payload command
  cmdtosend = W_TX_PAYLOAD;
  //HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);
  SPI.transfer(&cmdtosend);
 // Serial.print(cmdtosend);

  // send the payload
  //HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);
  SPI.transfer(data, 32);
  //Serial.print(data);
  // Unselect the device
  CS_UnSelect();


  uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);
 // Serial.print(fifostatus);

  // check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
  if ((fifostatus&(1 << 4))&&(!(fifostatus&(1 << 3))))
  {
    cmdtosend = FLUSH_TX;
    nrfsendCmd(cmdtosend);
    

    // reset FIFO_STATUS
    nrf24_reset (FIFO_STATUS);

    return 1;
  }

  return 0;
}


void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
  // disable the chip before configuring the device
  CE_Disable();

  nrf24_reset (STATUS);

  nrf24_WriteReg (RF_CH, channel);  // select the channel

  // select data pipe 2
  uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
  en_rxaddr = en_rxaddr | (1 << 2);
  nrf24_WriteReg (EN_RXADDR, en_rxaddr);

  /* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
     The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
     Their 4 MSB Bytes will still be same as Data Pipe 1

     For Eg->
     Pipe 1 ADDR = 0xAABBCCDD11
     Pipe 2 ADDR = 0xAABBCCDD22
     Pipe 3 ADDR = 0xAABBCCDD33

  */
  nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
  nrf24_WriteReg(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

  nrf24_WriteReg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2


  // power up the device in Rx mode
  uint8_t config_reg = nrf24_ReadReg(CONFIG);
  config_reg = config_reg | (1 << 1) | (1 << 0);
  nrf24_WriteReg (CONFIG, config_reg);

  // Enable the chip after configuring the device
  CE_Enable();
}


uint8_t isDataAvailable (int pipenum)
{
  uint8_t status_reg = nrf24_ReadReg(STATUS);

  if ((status_reg & (1 << 6)) && (status_reg & (pipenum << 1)))
  {

    nrf24_WriteReg(STATUS, (1 << 6));

    return 1;
  }

  return 0;
}


void NRF24_Receive (uint8_t *data)
{
  uint8_t cmdtosend = 0;
  CS_Select();

  // payload command
  cmdtosend = R_RX_PAYLOAD;
  SPI.transfer(&cmdtosend, 1);
  SPI.transfer(data, 32);

  // Unselect the device
  CS_UnSelect();

  delay(100);

  cmdtosend = FLUSH_RX;
  nrfsendCmd(cmdtosend);
}



// Read all the Register data
void NRF24_ReadAll (uint8_t *data)
{
  for (int i = 0; i < 10; i++)
  {
    *(data + i) = nrf24_ReadReg(i);
  }

  nrf24_ReadReg_Multi(RX_ADDR_P0, (data + 10), 5);

  nrf24_ReadReg_Multi(RX_ADDR_P1, (data + 15), 5);

  *(data + 20) = nrf24_ReadReg(RX_ADDR_P2);
  *(data + 21) = nrf24_ReadReg(RX_ADDR_P3);
  *(data + 22) = nrf24_ReadReg(RX_ADDR_P4);
  *(data + 23) = nrf24_ReadReg(RX_ADDR_P5);

  nrf24_ReadReg_Multi(RX_ADDR_P0, (data + 24), 5);

  for (int i = 29; i < 38; i++)
  {
    *(data + i) = nrf24_ReadReg(i - 12);
  }

}

void setup(){
Serial.begin(9600);
SPI.begin();
uint8_t data[50];
NRF24_Init();
uint8_t RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
NRF24_RxMode(RxAddress, 10);
pinMode(8,OUTPUT);
pinMode(7,OUTPUT);
}
void loop(){
    Serial.print((isDataAvailable(2)));
    //Serial.print("data is being received");
    
   if((isDataAvailable(2))==1)
    {
    Serial.print("data is being received\n");
         
    }   
    else
    {
      Serial.print("data is not being received\n");
    }

    
    }
     
        
  /* USER CODE END 3 */
