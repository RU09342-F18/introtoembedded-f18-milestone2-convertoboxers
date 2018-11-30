/* Rowan University
 * File: main.c
 * Course: Intro to Embedded Systems
 * Section: 1
 * Creation Data: 11/28/18
 * Milestone Project 2
 * Application - Closed Loop Control System for MSP430
 * Board - MSP430F5529
 */

#include <msp430.h> 
#include <math.h>

// Data Transmission
volatile unsigned char byte = 0;
volatile unsigned char data[8];

//Temperature Variables
unsigned int ADC_Result;
float adcMax = 4095.0; // MAX ADC Conversion
volatile float K = 2.0; // Constant
float error; // Calculate the difference between extractTempC - currentTempC

// Control Fan
volatile float pwmspeed = 10; // Control Speed
volatile int desiredTemp = 38; // Desired Temp Value

// The voltage divider consists of thermistor Rt and series resistor R0.
unsigned int R0 = 10000; // R0 = 10k

////////////////////////////////////////////////////////////


/* Initialize UART */
void setUart()
{
        P4SEL |= BIT5 | BIT4;   // UART TX, UART RX
        UCA1CTL1 |=  UCSWRST;   // Resets state machine
        UCA1CTL1 |=  UCSSEL_1;  // ACLK
        UCA1BR0   =  3;         // 9600 Baud Rate
        UCA1BR1   =  0;         // 9600 Baud Rate
        UCA1MCTL |= UCBRS_3 | UCBRF_0;
        UCA1CTL1 &= ~UCSWRST;   // Initializes the state machine
        UCA1IE |= UCTXIE;       // Enables USCI_A1 TX Interrupt
        UCA1IE   |=  UCRXIE;    // Enables USCI_A1 RX Interrupt

}

/* Set the PWM */
void setPWM()
{
    P1DIR |= BIT2;     // P1.2 output
    P1SEL |= BIT2;     // P1.2 to TA0.1
    P1OUT &= ~BIT2;    // P1.2 is off
    TA0CTL = TASSEL_2 + MC_1 + TACLR; // Configure TA0: Upmode using 1MHz clock / 4 = 250k
    TA0CCR0 = 255; // 250k / 255 = ~1kHz, set compare to 255

    TA0CCR1  = pwmspeed; // Fan PWM at P1.2
    TA0CCTL1 = OUTMOD_7; // Output PWM to pin 1.2
}

/* Set the ADC */
void setADC()
{
    // ADC Setup
    // Configure ADC A0 (pin 6.0)
    P6DIR &= ~BIT0;
    P6SEL |= BIT0;

    ADC12CTL2 = ADC12RES_2;     // 12-bit conversion
    ADC12CTL1 = ADC12SHP;       // Use sampling timer, set mode

    ADC12CTL0 = ADC12SHT1_15 | ADC12SHT0_15 | ADC12MSC | ADC12ON | ADC12TOVIE | ADC12ENC | ADC12SC;
    // Turn on ADC12, set sampling time
    // set multiple sample conversion

    ADC12IE = ADC12IE0;                         // Enable interrupt
    ADC12IFG &= ~ADC12IFG0;                     // Clear flag

}

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    setPWM();
    LED();
    setUart();
    setADC();

    __bis_SR_register(GIE);                     // LPM0, ADC_ISR will force exit
    while (1);
}

/* UART Interrupt*/
#pragma vector=USCI_A1_VECTOR
__interrupt void UartStart(void)
{
        if (UCA1IFG & UCTXIFG) // Check USCI A1 Interrupt Flags and USCI Transmit Interrupt Flag
    {
        UCA1IFG &= ~UCTXIFG; // Clear the TX flag
        if (byte > 0)
        {
            byte--;
            UCA1TXBUF = data[byte]; // Transmit message
        }
        else
        {
            __delay_cycles(5000); // Delay message
            ADC12CTL0 |= ADC12SC; // Start conversion
        }
    }

    if (UCA1IFG & UCRXIFG)
    {
        unsigned char setData = UCA1RXBUF; // Read data and clear flag
        desiredTemp = setData; // Set the new temperature to target
    }

}

// Interrupt to convert the ADC value into the MUcontroller
#pragma vector = ADC12_VECTOR
__interrupt void newADC(void)
{
    switch(ADC12IV)
    {
      case  6: {
          /* Return the Temperature in Celsius*/
              // Read the A0 of the ADC
              unsigned int adcVal = ADC12MEM0;

              // Convert the value to Ohms
              float Rt = R0 *((adcMax/adcVal)-1);

              // Convert the value to Kelvin
              const float invT0 = 1.00 / 298.15;   // room temp in Kelvin
              const float invBeta = 1.00 / 3977.00;   // replace "Beta" with beta of thermistor

              float tempK = 1.00 / (invT0 + invBeta*(log ( adcMax / adcVal - 1.00))); // SteinHart Equation

              // Convert to Celcius
              float tempC = (tempK - 273.0);

              error = tempC - desiredTemp; // Calculate the difference between extractTempC - currentTempC

              // Calculate the Control Variable

              pwmspeed = pwmspeed + (error * K);

              if (pwmspeed > 255)  // Adjust to MAX speed if the Control Variable is high
              {
                  pwmspeed = 255;
              }

              if (pwmspeed < 0) // Adjust to MIN speed if the Control Variable is low
              {
                  pwmspeed = 0;
              }

              if (TA0CCR1 == 0) // If the fan PWM is 0,
              {
                  if(pwmspeed > 0) // and if adjust Control Variable value is > 0,
                  {
                      TA0CCR1 = 30; // then set fan PWM at 30 %.
                  }
              }
              else
              {
                  if (pwmspeed < 15) // if adjust Control Variable is < 15
                  {
                      TA0CCR1 = 0; // then set fan PWM at 0%.
                  }
                  else if (pwmspeed > 255) // if adjust Control Variable is > 255
                  {
                      TA0CCR1 = 255;  // then set fan PWM at MAX.
                  }
                  else
                  {
                      TA0CCR1 = pwmspeed; // else set fan PWM at Control Variable.
                  }
              }

          // Send to ASICC characters
          byte = 7;
          unsigned int SendTemp = (tempC * 100); // Move two decimal places
          int i;
          for (i = 3; i < 8; i++)
          {
              if (i == 5)
              {
                  data[5] = '.'; //
              }
              else
              {
                  data[i] = (SendTemp % 10) + '0';
                  SendTemp = SendTemp / 10;
              }
          }
          data[2] = ' ';
          data[1] = 'C'; // 'C'
          data[0] = 32; // SPACE
          UCA1TXBUF = data[7]; // Ready to transmit temp in Celsuis
          break;
    }
      default:
    }
}
