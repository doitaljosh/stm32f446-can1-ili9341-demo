#define DEBUG 0
#include <SPI.h>
#include <TFT_eSPI.h>
#include "/home/atomicpi/Arduino/libraries/TFT_eSPI-master/examples/Smooth Fonts/FLASH_Array/Font_Demo_1_Array/NotoSansBold15.h"
TFT_eSPI lcd = TFT_eSPI();

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;

typedef enum {
  UI_STATUS = 0x00c,
  UI_TRIPPLANNING = 0x082,
  UI_SOLARDATA = 0x2d3,
  UI_GPSVEHICLESPEED = 0x3d9,
  UI_ODO = 0x3f3,
  VCFRONT_LIGHTING = 0x3f5,
  UI_HVACREQUEST = 0x2f3,
  UI_VEHICLECONTROL = 0x3b3,
  VCRIGHT_SWITCHSTATUS = 0x3c3,
  VCRIGHT_LIGHTSTATUS = 0x3e3,
  CC_CHGSTATUS = 0x31c,
  UI_RANGESOC = 0x33a,
  UI_POWER = 0x3bb,
  SYSTEMTIMEUTC = 0x318,
  UNIXTIME = 0x528,
  GEARLEVER = 0x229,
  SCCMLEFTSTALK = 0x249,
  UI_CHASSISCONTROL = 0x293,
  GPSLATLONG = 0x04f,
  TOTALCHARGEDISCHARGE = 0x3d2,
  STEERINGANGLE = 0x129,
  CHARGELINESTATUS = 0x264,
  DRIVESYSTEMSTATUS = 0x118,
  BMS_ENERGYSTATUS = 0x352,
  BMS_POWERAVAILABLE = 0x252,
  BMS_THERMAL = 0x312,
  BMS_SOC = 0x292,
  UISPEED = 0x257,
  VIN = 0x405,
  ODOMETER = 0x3b6,
  CARCONFIG = 0x7ff,
  BATTCELLVOLTAGES = 0x401
} m3_pids_t;

typedef struct
{
  uint32_t id;        /* 29 bit identifier                               */
  uint8_t  data[8];   /* Data field                                      */
  uint8_t  len;       /* Length of data field in bytes                   */
  uint8_t  ch;        /* Object channel(Not use)                         */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct
{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;

CAN_bit_timing_config_t can_configs[6] = {{2, 12, 60}, {2, 12, 30}, {2, 12, 24}, {2, 12, 12}, {2, 12, 6}, {1, 7, 5}};

/**
 * Print registers.
*/ 
void printRegister(char * buf, uint32_t reg) {
  if (DEBUG == 0) return;
  Serial.print(buf);
  Serial.print(reg, HEX);
  Serial.println();
}


/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
void CANSetGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t speed = 3) {
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7) {
      _index4 = (index - 8) * 4;
      ofs = 1;
    }

    uint32_t mask;
    printRegister("GPIO_AFR(b)=", addr->AFR[1]);
    mask = 0xF << _index4;
    addr->AFR[ofs]  &= ~mask;         // Reset alternate function
    setting = 0x9;                    // AF9
    mask = setting << _index4;
    addr->AFR[ofs]  |= mask;          // Set alternate function
    printRegister("GPIO_AFR(a)=", addr->AFR[1]);

    printRegister("GPIO_MODER(b)=", addr->MODER);
    mask = 0x3 << _index2;
    addr->MODER   &= ~mask;           // Reset mode
    setting = 0x2;                    // Alternate function mode
    mask = setting << _index2;
    addr->MODER   |= mask;            // Set mode
    printRegister("GPIO_MODER(a)=", addr->MODER);

    printRegister("GPIO_OSPEEDR(b)=", addr->OSPEEDR);
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;           // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;            // Set speed
    printRegister("GPIO_OSPEEDR(a)=", addr->OSPEEDR);

    printRegister("GPIO_OTYPER(b)=", addr->OTYPER);
    mask = 0x1 << index;
    addr->OTYPER  &= ~mask;           // Reset Output push-pull
    printRegister("GPIO_OTYPER(a)=", addr->OTYPER);

    printRegister("GPIO_PUPDR(b)=", addr->PUPDR);
    mask = 0x3 << _index2;
    addr->PUPDR   &= ~mask;           // Reset port pull-up/pull-down
    printRegister("GPIO_PUPDR(a)=", addr->PUPDR);
}

/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 27) return;

  CAN1->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN1->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN1->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
    if (mode == 0) {
    CAN1->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN1->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN1->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN1->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN1->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN1->FA1R |= (0x1UL<<index);                // Activate filter

}

    
/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 * @params: remap   - Select CAN port. 
 *                    =0:CAN1_RX mapped to PA11, CAN1_TX mapped to PA12
 *                       CAN2_RX mapped to PB5 , CAN2_TX mapped to PB6
 *                    =1:Not used
 *                    =2:CAN1_RX mapped to PB8,  CAN1_TX mapped to PB9 (not available on 36-pin package)
 *                       CAN2_RX mapped to PB12, CAN2_TX mapped to PB13
 *                    =3:CAN1_RX mapped to PD0,  CAN1_TX mapped to PD1 (available on 100-pin and 144-pin package)
 *                       CAN2_RX mapped to PB12, CAN2_TX mapped to PB13
 *
 */
bool CANInit(BITRATE bitrate, int remap)
{
  // Reference manual
  // https://www.st.com/content/ccc/resource/technical/document/reference_manual/4d/ed/bc/89/b5/70/40/dc/DM00135183.pdf/files/DM00135183.pdf/jcr:content/translations/en.DM00135183.pdf
  
  RCC->APB1ENR |= 0x2000000UL;           // Enable CAN1 clock 
  //RCC->APB1ENR |= 0x4000000UL;           // Enable CAN2 clock 
  
  if (remap == 0) {
    // CAN1
    RCC->AHB1ENR |= 0x1;                 // Enable GPIOA clock 
    CANSetGpio(GPIOA, 11);               // Set PA11
    CANSetGpio(GPIOA, 12);               // Set PA12
  }

  if (remap == 2) {
    // CAN1
    RCC->AHB1ENR |= 0x2;                 // Enable GPIOB clock 
    CANSetGpio(GPIOB, 8);                // Set PB8
    CANSetGpio(GPIOB, 9);                // Set PB9
  }
    
  if (remap == 3) {
    // CAN1
    RCC->AHB1ENR |= 0x8;                 // Enable GPIOD clock 
    CANSetGpio(GPIOD, 0);                // Set PD0
    CANSetGpio(GPIOD, 1);                // Set PD1
  }

  CAN1->MCR |= 0x1UL;                    // Require CAN1 to Initialization mode 
  while (!(CAN1->MSR & 0x1UL));          // Wait for Initialization mode
  printRegister("CAN1->MCR=", CAN1->MCR);
  
  //CAN1->MCR = 0x51UL;                  // Hardware initialization(No automatic retransmission)
  CAN1->MCR = 0x41UL;                    // Hardware initialization(With automatic retransmission)
  
  // Set bit rates 
  CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
  CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
  printRegister("CAN1->BTR=", CAN1->BTR);

  // Configure Filters to default values
  CAN1->FMR  |=   0x1UL;                 // Set to filter initialization mode
  CAN1->FMR  &= 0xFFFFC0FF;              // Clear CAN2 start bank
  printRegister("CAN1->FMR=", CAN1->FMR);

  // bxCAN has 28 filters.
  // These filters are used for both CAN1 and CAN2.
  // STM32F405 has CAN1 and CAN2, so CAN2 filters are offset by 14
  CAN1->FMR  |= 0xE00;                   // Start bank for the CAN2 interface

  // Set fileter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL); 

  // Set fileter 14
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  CANSetFilter(14, 1, 0, 0, 0x0UL, 0x0UL); 

  CAN1->FMR   &= ~(0x1UL);               // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;

  bool can1 = false;
  CAN1->MCR   &= ~(0x1UL);               // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN1->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //Serial.print("can1=");
  //Serial.println(can1);
  if (can1) {
    Serial.println("CAN1 initialize ok");
  } else {
    Serial.println("CAN1 initialize fail!!");
    return false;
  }
  return true;
}


#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU
 
/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
void CANReceive(uint8_t ch, CAN_msg_t* CAN_rx_msg)
{
  if(ch == 1) {
    uint32_t id = CAN1->sFIFOMailBox[0].RIR;
    if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
        CAN_rx_msg->format = STANDARD_FORMAT;;
        CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
    } 
    else {                               // Extended frame format
        CAN_rx_msg->format = EXTENDED_FORMAT;;
        CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    if ((id & STM32_CAN_RIR_RTR) == 0) {  // Data frame
        CAN_rx_msg->type = DATA_FRAME;
    }
    else {                                // Remote frame
        CAN_rx_msg->type = REMOTE_FRAME;
    }

    
    CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
    
    CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
    CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
    CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
    CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
    CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
    CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
    CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
    CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);
    
    CAN1->RF0R |= 0x20UL;
  } // end CAN1
}
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @params CAN_tx_msg - CAN message structure for transmission
 * 
 */
void CANSend(uint8_t ch, CAN_msg_t* CAN_tx_msg)
{
  volatile int count = 0;

  uint32_t out = 0;
  if (CAN_tx_msg->format == EXTENDED_FORMAT) { // Extended frame format
      out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  }
  else {                                       // Standard frame format
      out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
  }

  // Remote frame
  if (CAN_tx_msg->type == REMOTE_FRAME) {
      out |= STM32_CAN_TIR_RTR;
  }

  if (ch == 1) {
    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[0]      ));
    CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));

    // Send Go
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    // Wait until the mailbox is empty
    while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);

    // The mailbox don't becomes empty while loop
    if (CAN1->sTxMailBox[0].TIR & 0x1UL) {
      Serial.println("Send Fail");
      Serial.println(CAN1->ESR);
      Serial.println(CAN1->MSR);
      Serial.println(CAN1->TSR);
    }
  } // end CAN1
}

 /**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
uint8_t CANMsgAvail(uint8_t ch)
{
  if (ch == 1) {
    // Check for pending FIFO 0 messages
    return CAN1->RF0R & 0x3UL;
  } // end CAN1
}


uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)

void initLcd(void) {
  lcd.init();
  lcd.fillScreen(TFT_DARKGREY);
  lcd.setRotation(3);
  lcd.loadFont(NotoSansBold15);
  lcd.setCursor(0, 0, 4);
  lcd.setTextColor(TFT_WHITE, TFT_DARKGREY);
}

void clearLcd(void) {
  lcd.fillScreen(TFT_DARKGREY);
  lcd.setCursor(0, 0, 4);
}

uint8_t* getMcuTemperature(uint8_t data[8]) {
  uint8_t outputBuf[2];
  uint64_t value;
  memcpy(&value, data, 8);
  value >>= 48;
  outputBuf[0] = (uint8_t)(value & 0xFF);
  value >>= 8;
  outputBuf[1] = (uint8_t)(value & 0xFF);
  return outputBuf;
}

void setup() {
  initLcd();
  lcd.drawString("Initializing CAN bus...", 80, 110);
  bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  //bool ret = CANInit(CAN_1000KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  delay(2000);
  if (!ret) while(true);
  clearLcd();
   lcd.drawString("Ch: ", 0, 0);
   lcd.drawString("PID:", 0, 30);
   lcd.drawString("0x", 50, 30);
   lcd.drawString("DLC: ", 0, 60);
   lcd.drawString("Data: ", 0, 90);
   lcd.drawString("MCU Temp: CPU:", 0, 140);
   lcd.drawString("PCB:", 180, 140);
}

void loop() {
  //clearLcd();
  CAN_msg_t CAN_TX_msg;
  CAN_msg_t CAN_RX_msg;
   
  CAN_TX_msg.data[0] = 0x4E;
  CAN_TX_msg.data[1] = 0x09;
  CAN_TX_msg.data[2] = 0x85;
  CAN_TX_msg.data[3] = 0x18;
  CAN_TX_msg.data[4] = 0x0F;
  CAN_TX_msg.data[5] = 0x80;
  CAN_TX_msg.data[6] = 0x00;
  CAN_TX_msg.data[7] = 0x00;
  CAN_TX_msg.len = frameLength;

  uint8_t send_ch = 1;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( ( counter % 2) == 0) {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.id = 0x32F446;
    } else {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = STANDARD_FORMAT;
      CAN_TX_msg.id = 0xDDD;
    }
    CANSend(send_ch, &CAN_TX_msg);
    frameLength++;
    if (frameLength == 9) frameLength = 0;
    counter++;
  }

  uint8_t recv_ch = 1;
  int skipChars = 0;
  if(CANMsgAvail(recv_ch)) {
    digitalWrite(PA5, HIGH);
    CANReceive(recv_ch, &CAN_RX_msg);
    //lcd.print("Ch: ");
    //lcd.print(recv_ch);
    lcd.drawString(String(recv_ch), 50, 0);
    //lcd.println("\n");
      if (CAN_RX_msg.format != EXTENDED_FORMAT) {
        if (CAN_RX_msg.id < 0x100)
          lcd.drawString("0", 68, 30);
          skipChars = 10;
        if (CAN_RX_msg.id < 0x10)
          lcd.drawString("00", 68, 30);
          skipChars = 0;
        lcd.drawString(String(CAN_RX_msg.id, HEX), 68+skipChars, 30);
        //if (CAN_RX_msg.id < 0x1000) lcd.print("    ");
        //lcd.println("\n");
      }
  
      lcd.drawString(String(CAN_RX_msg.len), 50, 60);
      //lcd.println("\n");
      if (CAN_RX_msg.type == DATA_FRAME) {
        int xpos = 0;
        for(int i=0; i<CAN_RX_msg.len; i++) {
          //if (i < 0x10) lcd.drawString("0", xpos, 6); 
          lcd.drawString(String(CAN_RX_msg.data[i], HEX), xpos, 120); 
          if (i != (CAN_RX_msg.len-1)) {
            lcd.drawString(" ", xpos+30, 150);
            xpos += 30;
          } else {
            xpos = 0;
          }
        }
        //lcd.println("\n");
      } else {
        //Serial.println(" Data: REMOTE REQUEST FRAME");
      }
      uint8_t* temps = getMcuTemperature(CAN_RX_msg.data);
      lcd.drawString(String(temps[0]), 128, 140);
      lcd.drawString(String(temps[1]), 222, 140);
      digitalWrite(PA5, LOW);
    }
}
