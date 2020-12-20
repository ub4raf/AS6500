#include    "driverlib.h"
#include    "StopWatchMode.h"
#include    "hal_LCD.h"
#include    "stdio.h"
#include    "stdlib.h"
#include    "HAL_UART.h"
#include    "delay.h"
#include    "stddef.h"

#define GPIO_PORT_SSN   GPIO_PORT_P1
#define GPIO_PIN_SSN    GPIO_PIN5
#define GPIO_PORT_SCK   GPIO_PORT_P1
#define GPIO_PIN_SCK    GPIO_PIN4
#define GPIO_PORT_MOSI  GPIO_PORT_P1
#define GPIO_PIN_MOSI   GPIO_PIN6
#define GPIO_PORT_MISO  GPIO_PORT_P1
#define GPIO_PIN_MISO   GPIO_PIN7
#define GPIO_PORT_INTERRUPT GPIO_PORT_P4
#define GPIO_PIN_INTERRUPT  GPIO_PIN7

volatile unsigned char mode = 0;
volatile unsigned char S1buttonDebounce = 0;
volatile unsigned char S2buttonDebounce = 0;
volatile unsigned int holdCount = 0;
volatile unsigned int counter = 0;
volatile int centisecond = 0;
volatile bool flag_rx_spi=0;
Calendar currentTime;

char RXData=0;
char TXData=0;

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};


   // Virtual functions:
   // send_byte_to_SPI( Var1 ); : send Var1 (8 Bits) through the SPI
   // read_byte_from_SPI( Var1 ); : read 1 Byte data from SPI and write it
void    send_byte_to_SPI(uint8_t TXData)
    {
        //char message[7];
        //sprintf(message,"T %02X\n",TXData);
        //UART_puts(EUSCI_A1_BASE,message);
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, TXData);
        __bis_SR_register(LPM0_bits);
    }
uint8_t read_byte_from_SPI(void)
    {
        //UART_puts(EUSCI_A1_BASE,"T FF\n");
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, 0xFF);
        __bis_SR_register(LPM0_bits);
        return  RXData;
    }

/// CS and interrupt pin macro
#define GPIO_SSN_1                        (GPIO_setOutputHighOnPin (GPIO_PORT_SSN, GPIO_PIN_SSN))
#define GPIO_SSN_0                        (GPIO_setOutputLowOnPin  (GPIO_PORT_SSN, GPIO_PIN_SSN))
#define GPIO_INTERRUPT                    (GPIO_getInputPinValue   (GPIO_PORT_INTERRUPT,GPIO_PIN_INTERRUPT))

/// CFG0    register
#define PIN_ENA_STOP1       0x01
#define PIN_ENA_STOP2       0x02
#define PIN_ENA_STOP3       0x04
#define PIN_ENA_STOP4       0x08
#define PIN_ENA_REFCLK      0x10
#define CFG0_FIXED_VALUE    0x00    /// FIXED:  BIT_5 <- 0
#define PIN_ENA_DISABLE     0x40
#define PIN_ENA_RSTIDX      0x80
/// CFG1    register
#define HIT_ENA_STOP1   0x01
#define HIT_ENA_STOP2   0x02
#define HIT_ENA_STOP3   0x04
#define HIT_ENA_STOP4   0x08
#define CHANNEL_COMBINE_NORMAL      0x00
#define CHANNEL_COMBINE_PULSE_DISTANCE  0x10
#define CHANNEL_COMBINE_PULSE_WIDTH     0x20
#define HIGH_RESOLUTION_OFF 0x00
#define HIGH_RESOLUTION_X2  0x40
#define HIGH_RESOLUTION_X4  0x80
/// CFG2    register
#define CFG2_FIXED_VALUE    0x00    /// FIXED [5:0]<--0b00000
#define COMMON_FIFO_READ    0x40
#define BLOCKWISE_FIFO_READ 0x80
/// CFG3, CFG4, CFG5 Registers:
/// CFG3 REFCLK_DIVISIONS lower  8 bits
/// CFG4 REFCLK_DIVISIONS middle 8 bits
/// CFG5 REFCLK_DIVISIONS high   4 bits
/// CFG6    register
#define CFG6_FIXED_VALUE    0xC0
/// CFG7    register
#define CFG7_FIXED_VALUE    0x23
#define REFCLK_BY_XOSC      0x80
/// CFG8    register
#define CFG8_FIXED_VALUE    0xA1
/// CFG9    register
#define CFG9_FIXED_VALUE    0x13
/// CFG10    register
#define CFG10_FIXED_VALUE   0x00
/// CFG11    register
#define CFG11_FIXED_VALUE   0x0A
/// CFG12    register
#define CFG12_FIXED_VALUE   0xCC
/// CFG13    register
#define CFG13_FIXED_VALUE   0x05
/// CFG14    register
#define CFG14_FIXED_VALUE   0xF1
/// CFG15    register
#define CFG15_FIXED_VALUE   0x7D
/// CFG16    register
#define CFG16_FIXED_VALUE   0x04

   // to Var1
   // Virtual pin variables:
   // GPIO_SSN : Variable (1 Bit) to control the output pin which is
   // supposed to be connected the SSN pin of the AS6500
   // GPIO_INTERRUPT: Variable (1 Bit) to monitor the input pin which is
   // supposed to be connected INTERRUPT pin of the AS6500
   // --------------------------------------------------------------------------
   // *** Configuration Registers ***
   // --------------------------------------------------------------------------
   const char config_register[17] = {/*0x03, 0x03, 0xDF, 0x40,
                                   0x0D, 0x03, 0xC0, 0xD3,
                                   0xA1, 0x13, 0x00, 0x0A,
                                   0xCC, 0xCC, 0xF1, 0x7D,
                                   0x04   init from datasheet */

                                  PIN_ENA_STOP1|PIN_ENA_STOP2|PIN_ENA_STOP3|PIN_ENA_STOP4|CFG0_FIXED_VALUE,
                                  HIT_ENA_STOP1|HIT_ENA_STOP2|HIT_ENA_STOP3|HIT_ENA_STOP4|CHANNEL_COMBINE_NORMAL|HIGH_RESOLUTION_OFF,
                                  CFG2_FIXED_VALUE,
                                  0xF1, 0xFA, 0x02, /// REFCLK_DIVISIONS is 0x2FAF1 in case of 5.120MHz quartz crystal and 1ps resolution
                                  CFG6_FIXED_VALUE, /// FIXED value
                                  CFG7_FIXED_VALUE|REFCLK_BY_XOSC,
                                  //    FIXED register values:
                                  CFG8_FIXED_VALUE, CFG9_FIXED_VALUE, CFG10_FIXED_VALUE, CFG11_FIXED_VALUE,
                                  CFG12_FIXED_VALUE, CFG13_FIXED_VALUE, CFG14_FIXED_VALUE, CFG15_FIXED_VALUE,
                                  CFG16_FIXED_VALUE
                                   };
  // --------------------------------------------------------------------------
  // *** SPI Opcodes ***
  // --------------------------------------------------------------------------
   const char spiopc_power = 0x30; // opcode for "Power on Reset"
   const char spiopc_init = 0x18; // opcode for "Initialize Chip and Start
   const char spiopc_write_config = 0x80; // opcode for "Write Configuration"
   const char spiopc_read_config = 0x40; // opcode for "Read Configuration"
   const char spiopc_read_results = 0x60; // opcode for "Read Measure Results"
   // --------------------------------------------------------------------------
   // *** SPI Addresses ***
//   REFID is the reference index of the preceding reference clock edge.
//   TSTOP is the ratio of the internal measured times of tSTOP over tREF scaled by the configured
//   REFCLK_DIVISONS. For details see section Time Measurements and Results.
   // --------------------------------------------------------------------------
   const char reference_index_ch1_byte3 = 8;
   const char reference_index_ch1_byte2 = 9;
   const char reference_index_ch1_byte1 = 10;
   const char stopresult_ch1_byte3 = 11;
   const char stopresult_ch1_byte2 = 12;
   const char stopresult_ch1_byte1 = 13;
   const char reference_index_ch2_byte3 = 14;
   const char reference_index_ch2_byte2 = 15;
   const char reference_index_ch2_byte1 = 16;
   const char stopresult_ch2_byte3 = 17;
   const char stopresult_ch2_byte2 = 18;
   const char stopresult_ch2_byte1 = 19;
   const char reference_index_ch3_byte3 = 20;
   const char reference_index_ch3_byte2 = 21;
   const char reference_index_ch3_byte1 = 22;
   const char stopresult_ch3_byte3 = 23;
   const char stopresult_ch3_byte2 = 24;
   const char stopresult_ch3_byte1 = 25;
   const char reference_index_ch4_byte3 = 26;
   const char reference_index_ch4_byte2 = 27;
   const char reference_index_ch4_byte1 = 28;
   const char stopresult_ch4_byte3 = 29;
   const char stopresult_ch4_byte2 = 30;
   const char stopresult_ch4_byte1 = 31;



   // Initialization calls
   void Init_GPIO(void);
   void Init_Clock(void);

   uint8_t uart_state_get(uint16_t state)   //  from another project.
   {
       switch(state)
       {
       case EUSCI_A1_BASE:
           return 1;//settings.UART_1_flag;
       case EUSCI_A0_BASE:
           return 0;//settings.UART_0_flag;
       default: return 0x00;
       }
   }
   void set_UART_state_get(void)            //  from another project
   {
       UART_state=uart_state_get;
   }

int main(void) {

    // *** Other Variables ***
    // --------------------------------------------------------------------------
    uint8_t Buffer = 0; // buffer variable used to copy the SPI data
    uint16_t i = 0; // counter for for-loops
    uint32_t reference_index[4] = {0}; // reference index data array {Ch1, Ch2, Ch3, Ch4}
    uint32_t stopresult[4] = {0}; // stop result data array {Ch1, Ch2, Ch3, Ch4}
    bool config_error = false; // flag that indicates if the config registers are not written correctly

    uint8_t buffer_spi_in[24];  /// total receive buffer 4ch*6B
    char    buffer_str[17];     /// sprintf() string
    char    error_no=0;         /// error message

    // Stop watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PMM_unlockLPM5();

    //enable interrupts
    __enable_interrupt();

    // Initializations
    Init_GPIO();
    Init_Clock();
    Init_LCD(LCD_MEMORY_MAIN);
    //error_no=UART_init(EUSCI_A1_BASE,9600, CS_getACLK());
    error_no=UART_init(EUSCI_A1_BASE,115200, CS_getSMCLK());

    if(error_no)
        {
            displayScrollText("ERROR_UART1_INIT:", LCD_MEMORY_MAIN);
            showString("error", LCD_MEMORY_MAIN );
            showChar(error_no+'0', 5, LCD_MEMORY_MAIN );
            delay_ms(1000);
        }

    set_UART_state_get();
    __enable_interrupt();
    UART_puts(EUSCI_A1_BASE,"hello TDC\n\r");

    //Initialize Master
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = //EUSCI_B_SPI_CLOCKSOURCE_ACLK;
                                EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = //CS_getACLK();
                                CS_getSMCLK();
    param.desiredSpiClock = //CS_getACLK()/4;
                            //1000;
                            500000; /// depended of wires length
                            //CS_getSMCLK();    /// max frequency
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT; //  0
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;             //  0
    param.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &param);

    //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

    EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE,
            EUSCI_B_SPI_RECEIVE_INTERRUPT);
    // Enable USCI_B0 RX interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_SPI_RECEIVE_INTERRUPT);

    //Wait for slave to initialize
    __delay_cycles(100);


    displayScrollText("HELLO TDC",LCD_MEMORY_MAIN);

    while(1){

        GPIO_SSN_1;
        delay_us(1);
        GPIO_SSN_0;
        delay_us(1);

        send_byte_to_SPI( spiopc_power ); // Opcode for "Power On Reset" is sent
        delay_us(100);

        GPIO_SSN_1;
        delay_us(1);
        GPIO_SSN_0;
        delay_us(1);

        config_error = false;
        send_byte_to_SPI( spiopc_write_config + 00 );
        // Opcode for "Write Configuration” and config address (00) are sent over SPI
        UART_puts(EUSCI_A1_BASE,"\nTx config:\n");
        for ( i = 0; i < 17; i++) // Send all 17 config registers via SPI
            {
                if(!(i%4))
                    UART_puts(EUSCI_A1_BASE,"\n");
                send_byte_to_SPI( config_register[i] );
                sprintf(buffer_str,"0x%02X\t",config_register[i]);
                UART_puts(EUSCI_A1_BASE,buffer_str);
            }

        GPIO_SSN_1;
        delay_us(1);
        GPIO_SSN_0;
        delay_us(1);

        send_byte_to_SPI( spiopc_read_config + 00 );
        // Opcode for "Read Configuration" and config address (00) are sent over SPI
        for ( i = 0; i < 17; i++)
            {
                Buffer=read_byte_from_SPI(); // read byte from SPI to Buffer variable
                if ( config_register[i] != Buffer )
                    config_error = true;
        // if there was a failure in writing the config registers, then the
        // config_error flag is raised.
            }

        GPIO_SSN_1;
        delay_us(1);
        GPIO_SSN_0;
        delay_us(1);

        send_byte_to_SPI( spiopc_read_config + 00 );
        UART_puts(EUSCI_A1_BASE,"\nrx_data_config:");
        for ( i = 0; i < 17+32; i++)            ////    Debug total registers read
            {
                if(!(i%4))
                    UART_puts(EUSCI_A1_BASE,"\n");
                sprintf(buffer_str,"0x%02X\t",read_byte_from_SPI());
                UART_puts(EUSCI_A1_BASE,buffer_str);
            }
        UART_puts(EUSCI_A1_BASE,"\n");

        if(config_error)
            {
                UART_puts(EUSCI_A1_BASE,"Config ERROR\n\r");
                displayScrollText("Config ERROR",LCD_MEMORY_MAIN);
            }
        else
            {
                UART_puts(EUSCI_A1_BASE,"Config OK\n\r");
                displayScrollText("Config OK",LCD_MEMORY_MAIN);
            }

        //if (config_error == false )
        //while(1)
          //  {
                UART_puts(EUSCI_A1_BASE,"start measurement\n\r");
                displayScrollText("start measurement",LCD_MEMORY_MAIN);

                GPIO_SSN_1;
                delay_us(1);
                GPIO_SSN_0;
                delay_us(1);

                send_byte_to_SPI( spiopc_init );

        // Opcode for "Initialize" is sent over SPI. This is required to start
        // measuring process
        // ------------------------------------------------------------------------
        // End of the configuration settings. After now the time measurement will
        // start. This code is designed to use SPI to read the measurement data from
        // AS6500.
            do
                {

                i=1000;
                while( GPIO_INTERRUPT != 0 )
                    {
                        if(GPIO_INTERRUPT)
                            P9OUT |= BIT7;
                        else
                            P9OUT &=~BIT7;
                        P1OUT ^= BIT0;
                        delay_ms(10);
                        if(!(--i))
                            {
                                UART_puts(EUSCI_A1_BASE,"\ntime overflow\n");
                                displayScrollText("time overflow",LCD_MEMORY_MAIN);
                                break;
                            }
                    } // wait till the Interrupt pin is low

                if(i)
                    {
                        UART_puts(EUSCI_A1_BASE,"\nevent captured\n");
                        displayScrollText("event captured",LCD_MEMORY_MAIN);
                    }



                GPIO_SSN_1;
                delay_us(1);
                GPIO_SSN_0;
                delay_us(1);

                UART_puts(EUSCI_A1_BASE,"\nrx_data_result:");
                send_byte_to_SPI( spiopc_read_results + reference_index_ch1_byte3 );
                for ( i = 0; i < 24; i++)
                    {
                        if(!(i%6))
                            UART_puts(EUSCI_A1_BASE,"\n");
                        else
                            if(!(i%3))
                                UART_puts(EUSCI_A1_BASE,"\t");
                        buffer_spi_in[i]=read_byte_from_SPI();
                        sprintf(buffer_str,"0x%02X\t",buffer_spi_in[i]);
                        UART_puts(EUSCI_A1_BASE,buffer_str);
                    }
                UART_puts(EUSCI_A1_BASE,"\n");

                // Opcode for "Read Result" and data address are sent
                for ( i = 0; i < 4; i++)
                    {
                        reference_index[i]  =0;
                        stopresult[i]       =0;

                        Buffer=buffer_spi_in[i*6+0];
                        reference_index[i] = /*reference_index[i] +*/ ( (uint32_t)Buffer << 16 );
                        // Data is shifted 16 Bits to the left and added to the reference_index
                        //Buffer=read_byte_from_SPI(  ); // read one byte from SPI to Buffer
                        Buffer=buffer_spi_in[i*6+1];
                        reference_index[i] = reference_index[i] + ( (uint16_t)Buffer << 8 );
                        // Data is shifted 8 Bits to the left and added to the reference_index
                        Buffer=buffer_spi_in[i*6+2];
                        reference_index[i] = reference_index[i] + Buffer;
                        // Data is directly added to reference_index

                        // The complete reference index (3 Bytes) has been received.
                        Buffer=buffer_spi_in[i*6+3];
                        stopresult[i] = /*stopresult[i]+ */( (uint32_t)Buffer << 16 ); // is repeated for stop results
                        Buffer=buffer_spi_in[i*6+4];
                        stopresult[i] = stopresult[i] + ( (uint16_t)Buffer << 8 );
                        Buffer=buffer_spi_in[i*6+5];
                        stopresult[i] = stopresult[i] + Buffer;
                        // The complete stop result (3 Bytes) has been received

                        sprintf(buffer_str,"CH%d:\t",i+1);
                        UART_puts(EUSCI_A1_BASE,buffer_str);
                        sprintf(buffer_str,"0x%02X%04X\t",(uint8_t)(reference_index[i]>>16),
                                (uint16_t)reference_index[i]);//&0xFFFF);
                        UART_puts(EUSCI_A1_BASE,buffer_str);
                        sprintf(buffer_str,"%ld\t",reference_index[i]);
                        UART_puts(EUSCI_A1_BASE,buffer_str);
                        sprintf(buffer_str,"0x%02X%04X\t\t",(uint8_t)(stopresult[i]>>16),
                                (uint16_t)stopresult[i]);//&0xFFFF);
                        UART_puts(EUSCI_A1_BASE,buffer_str);
                        sprintf(buffer_str,"%ld ps\t",stopresult[i]);
                        UART_puts(EUSCI_A1_BASE,buffer_str);

                        if(i)
                            {
                                sprintf(buffer_str,"%d-%d %ld ps",i,i-1,
                                                                stopresult[i]-stopresult[i-1]);
                                UART_puts(EUSCI_A1_BASE,buffer_str);
                                sprintf(buffer_str,"%d-%d %ld ps",i,i-1,
                                                                stopresult[i]-stopresult[i-1]);
                                //displayScrollText(buffer_str,LCD_MEMORY_MAIN);
                            }
                        UART_puts(EUSCI_A1_BASE,"\n");



                    }

                }
                    while( GPIO_INTERRUPT == 0 );

                UART_puts(EUSCI_A1_BASE,"all events received\n");
                 // In this point the software has obtained
                // the reference_index and stopresult data for all channels,
                // the rest of the codes should be designed depending on the user’s
                // application.
                // . . .
                delay_ms(5000);
            }

    return (0);
}


/*
 * GPIO Initialization
 */
void Init_GPIO()
{

    /*
     *
     * NOT used ////Anal_in  8.4 left
     *
     * Interrupt4.7 right
     *
     * UCB0CLK  1.4 left
     * UCB0MISO 1.7 right
     * UCB0SIMO 1.6 right
     * UCB0STE  1.5 right
     *
     * UCA1TXD  3.4
     * UCA1RXD  3.5
     *
     * REDLED   1.0
     * GRNLED   9.7
     *
     * BTN1     1.1
     * BTN2     1.2
     */
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);


    // Configure button S1 (P1.1) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_INTERRUPT, GPIO_PIN_INTERRUPT);

    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_MISO,
            GPIO_PIN_MISO,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_MOSI,
            GPIO_PIN_MOSI,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_SCK,
            GPIO_PIN_SCK,
            GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsOutputPin(       GPIO_PORT_SSN,GPIO_PIN_SSN);
    GPIO_setOutputHighOnPin (GPIO_PORT_SSN, GPIO_PIN_SSN);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to default 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);

    // Configure MCLK and SMCLK to default 2MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);

    PMM_unlockLPM5();
    CS_setExternalClockSource(32768, 0);
    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif
void USCI_B0_ISR (void)
{
    //char message[7];
    switch (__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_SPI_UCRXIFG:      // UCRXIFG
            while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                    EUSCI_B_SPI_TRANSMIT_INTERRUPT));
            RXData = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
            //sprintf(message,"R %02X\n",RXData);
            //UART_puts(EUSCI_A1_BASE,message);
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        default:
            break;
    }
}

/*
 * RTC Interrupt Service Routine
 * Wakes up every ~10 milliseconds to update stowatch
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, 16))
    {
    case RTCIV_NONE: break;      //No interrupts
    case RTCIV_RTCOFIFG: break;      //RTCOFIFG
    case RTCIV_RTCRDYIFG:             //RTCRDYIFG
        counter = RTCPS;
        centisecond = 0;
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case RTCIV_RTCTEVIFG:             //RTCEVIFG
        //Interrupts every minute
        __no_operation();
        break;
    case RTCIV_RTCAIFG:             //RTCAIFG
        __no_operation();
        break;
    case RTCIV_RT0PSIFG:
        centisecond = RTCPS - counter;
        __bic_SR_register_on_exit(LPM3_bits);
        break;     //RT0PSIFG
    case RTCIV_RT1PSIFG:
        __bic_SR_register_on_exit(LPM3_bits);
        break;     //RT1PSIFG

    default: break;
    }
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 :    // Button S1 pressed
            P1OUT |= BIT0;    // Turn LED1 On
            if ((S1buttonDebounce) == 0)
            {
                // Set debounce flag on first high to low transition
                S1buttonDebounce = 1;
                holdCount = 0;

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG2 :    // Button S2 pressed
            P9OUT |= BIT7;    // Turn LED2 On
            if ((S2buttonDebounce) == 0)
            {
                // Set debounce flag on first high to low transition
                S2buttonDebounce = 1;
                holdCount = 0;

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 : break;
        case P1IV_P1IFG7 : break;
    }
}

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Both button S1 & S2 held down
    if (!(P1IN & BIT1) && !(P1IN & BIT2))
    {
        holdCount++;
        if (holdCount == 40)
        {
            P9OUT &= ~BIT7;
            P1OUT &= ~BIT0;
            S1buttonDebounce = 0;
            S2buttonDebounce = 0;
            // Stop Timer A0
            Timer_A_stop(TIMER_A0_BASE);

            /// PLACE DOUBLE BUTTON CALLBACK HERE
            //EUSCI_B_SPI_transmitData(EUSCI_B0_BASE,TXData);
            //GPIO_setOutputLowOnPin  (GPIO_PORT_SSN, GPIO_PIN_SSN);

            __bic_SR_register_on_exit(LPM0_bits);      // CPU off, enable interrupts
        }
    }
    else
    {

    // Button S1 n released
    if (P1IN & BIT1 && S1buttonDebounce)
    {
        S1buttonDebounce = 0;                                   // Clear button debounce
        P1OUT &= ~BIT0;
        Timer_A_stop(TIMER_A0_BASE);


        /// PLACE S1 BUTTON CALLBACK HERE
        //TXData++;
        //EUSCI_B_SPI_transmitData(EUSCI_B0_BASE,TXData);
        //GPIO_setOutputLowOnPin  (GPIO_PORT_SSN, GPIO_PIN_SSN);

        __bic_SR_register_on_exit(LPM0_bits);      // CPU off, enable interrupts
    }

    // Button S2 n released
    if (P1IN & BIT2 && S2buttonDebounce)
    {
        S2buttonDebounce = 0;                                   // Clear button debounce
        P9OUT &= ~BIT7;
        Timer_A_stop(TIMER_A0_BASE);

        /// PLACE S2 BUTTON CALLBACK HERE
        //TXData--;
        //EUSCI_B_SPI_transmitData(EUSCI_B0_BASE,TXData);
        //GPIO_setOutputLowOnPin  (GPIO_PORT_SSN, GPIO_PIN_SSN);

        __bic_SR_register_on_exit(LPM0_bits);      // CPU off, enable interrupts
    }
    }
    // Both button S1 & S2 released
    if ((P1IN & BIT1) && (P1IN & BIT2))
    {
        // Stop timer A0
        Timer_A_stop(TIMER_A0_BASE);
    }

}
