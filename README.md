# Objective:
#### Implement a system on the STM32F103 microcontroller using HAL libraries to generate a PWM signal, measure its voltage and duty cycle, control LEDs based on these measurements, and display the results on an LCD.
# Task Breakdown:

## 1.Generate PWM:

#### Use HAL libraries to configure a timer (e.g., TIM2) for PWM generation.

#### Implement functionality to adjust the duty cycle dynamically (e.g., using apotentiometer connected to an ADC channel or through software control).

## 2.Measure PWM Voltage using ADC:

#### Configur e the ADC using HAL to read the analog voltage of the PWM signal.

#### Ensure the ADC is set up for single channel conversion with the appropriate samplingtime.

#### Convert the ADC reading to a voltage value and implement any necessary scaling.

## 3.Measure Duty Cycle using ICU:

#### Configure another timer (e.g., TIM3) in Input Capture mode using HAL to capture the PWM signal.

#### Set up the timer to capture on both rising and falling edges of the signal.

 #### Use the captured values to calculate the duty cycle of the PWM signal.

#### Im plement the calculation using HAL functions to retrieve timer values and compute the duty cycle as a percentage.

## 4.Control LEDs Based on Duty Cycle/Voltage:

#### Map the measured duty cycle to LED control logic. For example, if the duty cycle is 80%, turn on 4 out of 5 LEDs.

#### Use GPIO functions from HAL to control the state of the LEDs.

#### Update the LEDs based on the measured duty cycle or voltage values.

## 5.Display Duty Cycle and Voltage on LCD:

#### Write functions to display the current duty cycle (as a percentage) and th e measured voltage value on the LCD.

#### Ensure the display is updated periodically to show real time values.
