# Milestone 2
In this milestone, the temperature of a 5 volt regulator controlled utilizing a fan and open loop and closed loop control.

### ADC DAC:

##### Voltage Regulator:
In order to heat up the 5 volt regulator, a load of 10 120 ohm 1/4 watt resistors in parallel (equivalent resistance of 12 ohms) was utilized, as shown below. This drew 0.417 amps and a load of 2.08 watts. Because of this high load, the voltage regulator was able to achieve a temperature of over 75 degrees Celsius.
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/5V%20Regulator%20Circuit.PNG)

##### Thermistor:
In order to measure the temperature of the voltage regulator, a 10k ohm thermistor was utilized. A thermistor varies in voltage based on the temperature of the thermistor. In order to derive the temperature of the thermistor, a voltage divider was utilized, as shown below.
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/Thermistor%20Circuit.PNG)
The output of the voltage regulator was then connected to P6.0 of the MSP430F5529 to be utilized as an Anaglog to Digital Converter (ADC) input. The ADC recieved the voltage in as a 12 bit number. In order to convert this to a voltage, the following equation was utilized: Voltage = (ADC_Input / 4096) * 3.3. In order to convert the voltage to the resistance of the thermistor, the following equation was utilized: Resistance = . In order to determine the temperature of the thermistor based on its resistance, the Steinhart and Hart equation was utilized based off of A1, B1, C1, and D1 values found in the datasheet of the thermistor. From there, the temperature was converted from Kelvin to Celsius.

##### UART:
In order to display the temperature, the temperature was transmitted over UART to an Arduino Uno, where the temperature was plotted in real time through a serial display on a computer. This allowed for a quick and easy to understand plot of the temperature. In order to recieve the desired temperature, the desired temperature was transmitted over UART from a computer, and then set a global variable to the message recieved.

##### Fan PWM:
In order to control the 12 volt fan, a low side switch was utilized to take the hardware PWM signal from P1.2 of the MSP430F5529 and control the fan. An N-Mos was utilized to connect and disconnect the fan to and from ground based on the recieved signal, this controlling the speed of the fan. A circuit of the low side switch can be seen below.
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/Low%20Side%20Fan%20Circuit.PNG)

### Open Loop:
Open Loop control determines the duty cycle of the fan in order to reach a desired temperature. In order to achieve this, the following equation was developed:
Duty Cycle = [(0.9259 * setTemp^2)  - (91.922 * setTemp) + 2333.6] / 255
This was done by plotting a series of temperatures of varying PWM duty cycles, plotting the measurements, and calculating a line of best fit. A plot of the measurements can be seen below.
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/Open%20Loop%20Control%20Plot.PNG)
From this, it was able to be determined that in order to achieve a temperature of below 35 degrees Celsius, the fan would have to be set to 100% duty cucle, and in order to achieve a tempaerature of above 55 degrees Celsius, the fan would have to be shut off. A digram of how this open loop control works can be seen below: 
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/Open%20Loop%20Block%20Diagram.png)

### Closed Loop:
A closed loop system utilizes both the desired temperature and current temperature to determine the fan PWM duty cycle. In order to do so, a Proportional, Integral, Derivative control, or PID control equation was utilized. 
The proportiaonal portion of the controller compares the desired temperature wuth the actual temperature. The resulting error is then multiplied with a constannt (KP) to get a portion of the output. 
The integral portion of the controller integrates the error until the error value reaches zero. The integral control decreases its output when negative error takes place and limits the speed of response. The intragral is mutilplied with a constant (KI) to get another portion of the output.
The derivative portion of the controller anticipates the future error of the value. The derivative is mutiplied with a constant (KD) to get the last potion of the output.
The constants, KP, KI, and KD can be adjusted in order to better tune the closed loop control and minimize oscillation of the temperature.
A diagram of how this closed clop control works can be seen below:
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/PID%20Block%20Diagram.png)
