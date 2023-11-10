#include <can.h>
#include <cm4.h>
#include <sys.h>
#include <kstdio.h>

CAN_msg CAN_TxMsg; // CAN messge for sending
CAN_msg CAN_RxMsg; // CAN message for receiving

unsigned int CAN_TxRdy = 0; // CAN HW ready to transmit a message
unsigned int CAN_RxRdy = 0; // CAN HW received a message

/* CAN identifier type */
#define CAN_ID_STD ((uint32_t)0x00000000) /* Standard Id          */
#define CAN_ID_EXT ((uint32_t)0x00000004) /* Extended Id          */

/* CAN remote transmission request */
#define CAN_RTR_DATA ((uint32_t)0x00000000)   /* Data frame           */
#define CAN_RTR_REMOTE ((uint32_t)0x00000002) /* Remote frame         */

/*----------------------------------------------------------------------------
  setup CAN interface
 *----------------------------------------------------------------------------*/
void CAN_setup(void)
{

    /* Enable clock for CAN2 and GPIOB */
    RCC->APB1ENR |= (1 << 25) | (1 << 26);
    RCC->AHB1ENR |= (1 << 1);

    /* CAN2, we use PB5, PB6  -- Datasheet - p. 58*/
    GPIOB->MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)));
    GPIOB->MODER |= ((2 << (5 * 2)) | (2 << (6 * 2)));
    GPIOB->OTYPER &= ~((1 << 5) | (1 << 6));
    GPIOB->OSPEEDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)));
    GPIOB->PUPDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)));

    GPIOB->AFRL &= ~((15 << (5 * 4)) | (15 << (6 * 4))); // Clear alternate function for PB5, PB6
    GPIOB->AFRL |= ((9 << (5 * 4)) | (9 << (6 * 4)));    // Set alternate function to AF9 for PB5, PB6

    /*
    ENABLE CAN2 RX0, TX INTERRUPT // RM - P. 1047
    */

    CAN2->IER |= CAN_IER_FMPIE0; // Enable FIFO0 message pending interrupt
    CAN2->IER |= CAN_IER_TMEIE;  // Enable Transmit mailbox empty interrupt

    NVIC_EnableIRQ(CAN2_RX0_IRQn); // Enable CAN2 RX0 interrupt
    NVIC_EnableIRQ(CAN2_TX_IRQn);  // Enable CAN2 TX interrupt

    /*
    SET CAN2 TO INITIALIZATION MODE
    */

    CAN2->MCR = (CAN_MCR_NART | CAN_MCR_INRQ);
    while (!(CAN2->MSR & CAN_MCR_INRQ))
        ; // Wait until CAN2 is in initialization mode

    /* Note: this calculations fit for CAN (APB1) clock = 42MHz */
    uint32_t brp = (42000000 / 7) / 500000; /* baudrate is set to 500k bit/s    */

    /* set BTR register so that sample point is at about 71% bit time from bit start */
    /* TSEG1 = 4, TSEG2 = 2, SJW = 3 => 1 CAN bit = 7 TQ, sample at 71%      */
    CAN2->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF));
    CAN2->BTR |= ((((3 - 1) & 0x03) << 24) | (((2 - 1) & 0x07) << 20) | (((4 - 1) & 0x0F) << 16) | ((brp - 1) & 0x3FF));

    /*
    SET CLOCK PRESCALER

    CAN2CLK = 45 MHz
    CAN2 Baudrate = 500 kbit/s
    CAN2 Baudrate Prescaler = 45 MHz / (500 kbit/s * 9) = 10
    */

    // CAN2->BTR |= (10 << 0); // Set bits 0 to 9 to 10
}

/*----------------------------------------------------------------------------
  setup acceptance filter
 *----------------------------------------------------------------------------*/
void CAN_wrFilter(unsigned int id, unsigned char format)
{
    static unsigned short CAN_filterIdx = 0;
    unsigned int CAN_msgId = 0;

    if (CAN_filterIdx > 13)
    { // check if Filter Memory is full
        return;
    }
    // Setup identifier information
    if (format == STANDARD_FORMAT)
    {
        CAN_msgId |= (unsigned int)(id << 21); //  32-bit identifier
    }
    else
    {
        CAN_msgId |= (unsigned int)(id << 3); //  32-bit identifier
        CAN_msgId |= CAN_ID_EXT;              // set Extended Identifier
    }

    CAN2->FMR |= CAN_FMR_FINIT;                        // set Initialisation mode for filter banks
    CAN2->FA1R &= ~(unsigned int)(1 << CAN_filterIdx); // deactivate filter

    // initialize filter
    CAN2->FS1R |= (unsigned int)(1 << CAN_filterIdx); // set 32-bit scale configuration
    CAN2->FM1R |= (unsigned int)(1 << CAN_filterIdx); // set 2 32-bit identifier list mode

    CAN2->sFilterRegister[CAN_filterIdx].FR1 = CAN_msgId; //  32-bit identifier
    CAN2->sFilterRegister[CAN_filterIdx].FR2 = CAN_msgId; //  32-bit identifier

    CAN2->FFA1R &= ~(unsigned int)(1 << CAN_filterIdx); // assign filter to FIFO 0
    CAN2->FA1R |= (unsigned int)(1 << CAN_filterIdx);   // activate filter

    CAN2->FMR &= ~CAN_FMR_FINIT; // reset Initialisation mode for filter banks

    CAN_filterIdx += 1; // increase filter index
}

/*----------------------------------------------------------------------------
  leave initialisation mode
 *----------------------------------------------------------------------------*/
void CAN_start(void)
{

    /*
    CAN_MCR
    Bit 0 INRQ: Initialization request
    The software clears this bit to switch the hardware into normal mode. Once 11 consecutive
    recessive bits have been monitored on the Rx signal the CAN hardware is synchronized and
    ready for transmission and reception. Hardware signals this event by clearing the INAK bit in
    the CAN_MSR register.
    Software sets this bit to request the CAN hardware to enter initialization mode. Once
    software has set the INRQ bit, the CAN hardware waits until the current CAN activity
    (transmission or reception) is completed before entering the initialization mode. Hardware
    signals this event by setting the INAK bit in the CAN_MSR register.
    */
    CAN2->MCR &= ~CAN_MCR_INRQ; // normal operating mode, reset INRQ


    /*
    CAN_MSR
    Bit 0 INAK: Initialization acknowledge
    This bit is set by hardware and indicates to the software that the CAN hardware is now in
    initialization mode. This bit acknowledges the initialization request from the software (set
    INRQ bit in CAN_MCR register).
    This bit is cleared by hardware when the CAN hardware has left the initialization mode (to
    be synchronized on the CAN bus). To be synchronized the hardware has to monitor a
    sequence of 11 consecutive recessive bits on the CAN RX signal.
    */
    // while (CAN2->MSR & CAN_MCR_INRQ){
    while (CAN2->MSR & CAN_MCR_INRQ){
        kprintf("%o\n", CAN2->MSR);
    }
        ; // wait till ready
    kprintf("Here 2.\n");
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
void CAN_waitReady(void)
{

    while ((CAN2->TSR & CAN_TSR_TME0) == 0)
        ; // Transmit mailbox 0 is empty
    CAN_TxRdy = 1;
}

/*----------------------------------------------------------------------------
  wite a message to CAN peripheral and transmit it
 *----------------------------------------------------------------------------*/
void CAN_wrMsg(CAN_msg *msg)
{

    CAN2->sTxMailBox[0].TIR = (uint32_t)0; /* reset TXRQ bit */
                                           /* Setup identifier information */
    if (msg->format == STANDARD_FORMAT)
    { /*    Standard ID                   */
        CAN2->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21) | CAN_ID_STD;
    }
    else
    { /* Extended ID                      */
        CAN2->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 3) | CAN_ID_EXT;
    }

    /* Setup type information           */
    if (msg->type == DATA_FRAME)
    { /* DATA FRAME                       */
        CAN2->sTxMailBox[0].TIR |= CAN_RTR_DATA;
    }
    else
    { /* REMOTE FRAME                     */
        CAN2->sTxMailBox[0].TIR |= CAN_RTR_REMOTE;
    }
    /* Setup data bytes                 */
    CAN2->sTxMailBox[0].TDLR = (((uint32_t)msg->data[3] << 24) |
                                ((uint32_t)msg->data[2] << 16) |
                                ((uint32_t)msg->data[1] << 8) |
                                ((uint32_t)msg->data[0]));
    CAN2->sTxMailBox[0].TDHR = (((uint32_t)msg->data[7] << 24) |
                                ((uint32_t)msg->data[6] << 16) |
                                ((uint32_t)msg->data[5] << 8) |
                                ((uint32_t)msg->data[4]));
    /* Setup length                     */
    CAN2->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN2->sTxMailBox[0].TDTR |= (msg->len & CAN_TDT0R_DLC);

    CAN2->IER |= CAN_IER_TMEIE;               /* enable  TME interrupt        */
    CAN2->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; /* transmit message             */
}

/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 *----------------------------------------------------------------------------*/
void CAN_rdMsg(CAN_msg *msg)
{

    /* Read identifier information  */
    if ((CAN2->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0)
    {
        msg->format = STANDARD_FORMAT;
        msg->id = 0x000007FF & (CAN2->sFIFOMailBox[0].RIR >> 21);
    }
    else
    {
        msg->format = EXTENDED_FORMAT;
        msg->id = 0x1FFFFFFF & (CAN2->sFIFOMailBox[0].RIR >> 3);
    }
    /* Read type information        */
    if ((CAN2->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0)
    {
        msg->type = DATA_FRAME;
    }
    else
    {
        msg->type = REMOTE_FRAME;
    }
    /* Read number of rec. bytes    */
    msg->len = (CAN2->sFIFOMailBox[0].RDTR) & 0x0F;
    /* Read data bytes              */
    msg->data[0] = (CAN2->sFIFOMailBox[0].RDLR) & 0xFF;
    msg->data[1] = (CAN2->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
    msg->data[2] = (CAN2->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
    msg->data[3] = (CAN2->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

    msg->data[4] = (CAN2->sFIFOMailBox[0].RDHR) & 0xFF;
    msg->data[5] = (CAN2->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
    msg->data[6] = (CAN2->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
    msg->data[7] = (CAN2->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

    CAN2->RF0R |= CAN_RF0R_RFOM0; /* Release FIFO 0 output mailbox */
}

/*----------------------------------------------------------------------------
  Interrupt Service Routine for CAN interrupt
 *----------------------------------------------------------------------------*/

void CAN2_TX_Handler(void)
{
    kprintf("CAN2 TX Interrupt\n");
    if (CAN2->TSR & CAN_TSR_RQCP0)
    {                                /* request completed mbx 0        */
        CAN2->TSR |= CAN_TSR_RQCP0;  /* reset request complete mbx 0   */
        CAN2->IER &= ~CAN_IER_TMEIE; /* disable  TME interrupt         */
        CAN_TxRdy = 1;
    }
}

void CAN2_RX0_Handler(void)
{
    kprintf("CAN2 RX0 Interrupt\n");
    if (CAN2->RF0R & CAN_RF0R_FMP0)
    {                          // message pending ?
        CAN_rdMsg(&CAN_RxMsg); // read the message
        CAN_RxRdy = 1;         // set receive flag
    }
}