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
In order to display the temperature, the temperature was transmitted over UART to an Arduino Uno, where the temperature was plotted in real time through a serial display on a computer. This allowed for a quick and easy to understand plot of the temperature

##### Fan PWM:
In order to control the 12 volt fan, a low side switch was utilized to take the hardware PWM signal from P1.2 of the MSP430F5529 and control the fan. An N-Mos was utilized to connect and disconnect the fan to and from ground based on the recieved signal, this controlling the speed of the fan. A circuit of the low side switch can be seen below.
![alt text](https://github.com/RU09342-F18/introtoembedded-f18-milestone2-russell-s-biggest-fan-s/blob/master/Figures/Low%20Side%20Fan%20Circuit.PNG)

### Open Loop:

### Closed Loop:
