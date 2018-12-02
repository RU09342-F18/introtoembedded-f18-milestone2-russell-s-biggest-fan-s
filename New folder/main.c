#include <msp430.h>
#include <math.h>

// P1.2 is the PWM pin for the fan.
// P6.0 is the analog input pin for the thermistor
// P3.4 is the UART0 receive pin
// P3.3 is the UART0 transmit pin
// P4.5 is the UART1 receive pin
// P4.4 is the UART1 transmit pin

//Constants to determine temperature for the Steinhart and Hart Equation
#define A1 3.354016e-3
#define B1 2.56985e-4
#define C1 2.620131e-6
#define D1 6.383091e-8

// Variables utilized to calculate the temperature of the thermistor
float voltage = 0;
float resistance = 0;
int temperature = 0;

int setTemp = 100;                              // Desired temperature

int fanValue = 0;                               // Fan value utilized in open loop control

// Variables utilized to determine PWM value for closed loop PID control
float kp = 1.25;
int ki = 0;
int  kd =  1;
float errorValue = 0.0;
float preError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output = 0.0;

// Configures ADC12 to be utilized for the thermistor
void configureADC12() {
    ADC12CTL0 = ADC12SHT02 + ADC12ON;           // Turns on ADC12
    ADC12CTL1 = ADC12SHP;                       // Sources the sampcon signal from the sampling timer
    ADC12IE = BIT0;                             // Enables ADC12 interrupts
    ADC12CTL0 |= ADC12ENC;                      // Enales ADC12 conversion
    P6SEL |= BIT0;                              // P6.0 is configures as the ADC12 input
    P1DIR |= BIT0;                              // P1.0 is set to the output direction
}

// Configures PWM to control fan speed
void configurePWM() {
    P1OUT &= ~BIT2;                             // Initially sets the output to 0
    P1DIR |= BIT2;                              // Sets P1.2 to the output direction
    P1SEL |= BIT2;                              // Connects P1.2 to TA CCR0 Capture
    TA0CTL = TASSEL_2 + ID_0 + MC_1 + TACLR;    // Configures TA0 to utilize SMCLK, an internal divider
                                                // of 1, sets the clock to up mode, and initially clears
                                                // the clock
    TA0CCR0 = 255;                              // Sets the period of the PWM cycle
    TA0CCTL1 = OUTMOD_7;                        // Sets TA0 to set/reset
}

void configureUART0() {
    P1DIR |= BIT0;                              // Sets P1.0 LED to the output direction
    P1OUT &= ~BIT0;                             // Initially clears P1.0 LED
    P3SEL |= BIT3 | BIT4;                       // P3.4 is connected to RX and P3.3 is connected to TX
    UCA0CTL1 = UCSWRST;                         // Enables software reset
    UCA0CTL1 = UCSSEL_1;                        // Sets ACLK as the clock source
    UCA0BR0 = 3;                                // Sets the baud rate to 9600
    UCA0BR1 = 0;                                // Sets the baud rate to 9600
    UCA0MCTL |= UCBRS_3 | UCBRF_0;              //
    UCA0CTL1 &= ~UCSWRST;                       // Disables software reset
    UCA0IE |= UCRXIE;                           // Enables RX based interrupts
    UCA0IE |= UCTXIE;                           // Enables TX based interrupts;
}

void configureUART1() {
    P1DIR |= BIT0;                              // Sets P1.0 LED to the output direction
    P1OUT &= ~BIT0;                             // Initially clears P1.0 LED
    P4SEL |= BIT4 | BIT5;                       // P4.5 is connected to RX and P4.4 is connected to TX
    UCA1CTL1 = UCSWRST;                         // Enables software reset
    UCA1CTL1 = UCSSEL_1;                        // Sets ACLK as the clock source
    UCA1BR0 = 3;                                // Sets the baud rate to 9600
    UCA1BR1 = 0;                                // Sets the baud rate to 9600
    UCA1MCTL |= UCBRS_3 | UCBRF_0;              //
    UCA1CTL1 &= ~UCSWRST;                       // Disables software reset
    UCA1IE |= UCRXIE;                           // Enables RX based interrupts
    UCA1IE |= UCTXIE;                           // Enables TX based interrupts
}

// Determines the temperature of the thermistor
void determineTemperature() {
    voltage = (ADC12MEM0 / 4096.0) * 3.3;       // Calculates the voltage
    resistance = (voltage * 10000.0) / (3.3 - voltage);
                                                // Calculates the resistance of the thermistor
    float temp = log(resistance / 10000);
    temperature = (1.0 / (A1 + (B1 * temp) + (C1 * temp * temp) + (D1 * temp * temp * temp))) - 270;
                                                // Calculates the temperature using the Steinhart and Hart
                                                // equation
    UCA0TXBUF = temperature;                    // Transmits the temperature of the thermistor through UART0
}

// Determines the PWM duty cycle based on an open loop equation
void openLoop(){
    if (setTemp <= 35) {                        // If the desired temperature is below 35 degrees celsius
        fanValue = 255;                         // the fan is set to 100% duty cycle
    } else if (setTemp <= 55) {                 // If the  desired temperature is below 55 degrees celsius but
                                                // above 35 degrees celsius, it uses the following equation
                                                // is utilized to determine the PWM duty cycle
        fanValue = (0.9259 * setTemp * setTemp)  - (91.922 * setTemp) + 2333.6;
    } else {                                    // If the desired temperature is above 55 degrees celsius, the
        fanValue = 0;                           // fan is shut off
    }
    TA0CCR1 = fanValue;
}

// Determines the PWM duty cycle based on a closed loop equation
void PID(){
    errorValue = setTemp - temperature;
    integral = integral + errorValue;
    derivative = errorValue - preError;
    output = kp * errorValue + ki * integral + kd * derivative;
    TA0CCR1 = output;
    preError = errorValue;
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                     // Disables watchdog timer

  //openLoop();                                 // Uncomment to enable open loop control

  configureADC12();                             // Configures ADC12
  configurePWM();                               // Configures PWM
  configureUART0();                             // Configures UART0
  configureUART1();                             // Configures UART1



  while (1)
  {
    ADC12CTL0 |= ADC12SC;                       // Start sampling and conversion of ADC12
    __bis_SR_register(LPM0_bits + GIE);         // LPM0, ADC12_ISR will force exit
    __no_operation();                           // For debugger
  }
}

// ADC12 interrupt vevtor
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void){
  switch(ADC12IV) {
  case  6:                                       // Vector 6:  ADC12IFG0
      determineTemperature();                    // Fuction to determine temperature from ADC12
      PID();                                     // Function to determine PWM duty cycle utilized closed
                                                 // loop feedback
      __bic_SR_register_on_exit(LPM0_bits);      // Exit active CPU

  default: break;
  }
}

// UART0 interrupt vector
// Utilized to transmit temperature to Arduino Uno
#pragma vector=USCI_A0_VECTOR
__interrupt void UART0(void) {
    P1OUT |= BIT0;                              // Turns on the P1.0 LED
    if (UCA0IFG & UCTXIFG) {                    // If the TX interrupt flag is triggered
        UCA0IFG &= ~UCTXIFG;                    // Clears the TX interrupt flag
        ADC12CTL0 |= ADC12SC;                   // Start sampling and conversion of ADC12
    } if (UCA0IFG & UCRXIFG){                   // If the RX interrupt flag is triggered
    }
    P1OUT &= ~BIT0;                             // Turns off the P1.0 LED
}

// UART1 interrupt vector
// Utilized to receive desired temperature from computer
#pragma vector=USCI_A1_VECTOR
__interrupt void UART1(void) {
    P1OUT |= BIT0;                              // Turns on the P1.0 LED
    if (UCA1IFG & UCTXIFG) {                    // If the TX interrupt flag is triggered
        UCA1IFG &= ~UCTXIFG;                    // Clears the TX interrupt flag
        ADC12CTL0 |= ADC12SC;                   // Start sampling and conversion of ADC12
    } if (UCA1IFG & UCRXIFG){                   // If the RX interrupt flag is triggered
        setTemp = UCA1RXBUF;                    // Desired temperature is set to the recieved message
    }
    P1OUT &= ~BIT0;                             // Turns off the P1.0 LED
}

