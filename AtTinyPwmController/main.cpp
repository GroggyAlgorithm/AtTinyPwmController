/**
 * \file AtTinyPwmController - main.cpp
 * \brief Program to control pwm frequency and duty cycle using tiny resources and the controllers ADC
 * \author Tim Robbins
 */

//Headers, Macros, and Definitions-----------------------------------------------

#include <avr/io.h>
#include <math.h>

///Mode selection for if we're using a pot to control the PWM's prescaler
#define ACTIVE_PRESCALER_CHANGE	1

///If we're detecting the prescaler POT value during startup before entering the main program
#define PRESCALER_ON_STARTUP	1

///If the frequency value can never reach 0 or not
#define FREQUENCY_NEVER_0		1

///The output register that the adc pins are on
#define ADC_PIN_OUTPUT_REGISTER PORTB

///The direction register the adc pins are on
#define ADC_PIN_DIR_REGISTER	DDRB

///The prescaler adc
#define ADC_PRESCALER_READ		PB4
#define ADC_PRESCALER_CHANNEL	2

///The duty cycle adc
#define ADC_DUTY_CYCLE_READ		PB3
#define ADC_DUTY_CYCLE_CHANNEL	3

///The frequency adc
#define ADC_FREQUNCY_READ		PB2
#define ADC_FREQUENCY_CHANNEL	1

///Pin mask for the ADC pins
#define ADC_PIN_MASK			(1 << ADC_FREQUNCY_READ | 1 << ADC_DUTY_CYCLE_READ | 1 << ADC_PRESCALER_READ)

///The amount of times to sample the ADC readings
#define ADC_SAMPLES				4

///Pwm out pin
#define PWM_OUT_A				PB1

///Automatic Anti phase Pwm pin
#define PWM_OUT_ANTI_A			PB0

///Clears the active prescaler and ensures pwm has no output by disabling pwm functionalities
#define PWM_CLEAR_PRESCALER()	GTCCR &= ~(1 << PSR1); TCCR1 &= ~(1 << COM1A0 | 1 << PWM1A)

///Sets the prescaler and related PWM bits using the timer 1 prescaler variable
#define PWM_SET_PRESCALER()		TCCR1 = (((TCCR1 | (1 << COM1A0 | 1 << PWM1A)) & 0xF0) | (timer1Prescaler & 0x0F) )

//-------------------------------------------------------------------------------


//Data types---------------------------------------------------------------------

//-------------------------------------------------------------------------------


//Variables----------------------------------------------------------------------

///The current prescaler for Timer 1
static uint8_t timer1Prescaler = (1 << CS10);

//-------------------------------------------------------------------------------


//Functions----------------------------------------------------------------------

void SystemInit();
void AdcSetup(uint8_t pinMask);
void PwmSetup();
uint8_t GetPrescaler();
uint8_t GetFrequency();
uint8_t GetDutyCycle(uint8_t frequencyValue);
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount);

//-------------------------------------------------------------------------------

/**
* \brief Entry point to program
*
*/
int main(void)
{
	//Variables
	uint8_t frequencyValue = 0; //Our current frequency register value
	uint8_t dutyCycle = 0; //Our current duty cycle register value
	
	//Initialize the system
	SystemInit();
    
	//Program loop
    while (1) 
    {
		
		//Get our frequency first, this way we can have an appropriate duty cycle
		frequencyValue = GetFrequency();
		
		//Get our duty cycle, set based on the frequency.
		dutyCycle = GetDutyCycle(frequencyValue);
		
		//If our duty cycle value is either off or our frequency value is off...
		if(frequencyValue <= 0 || dutyCycle <= 0) 
		{
			//Clear the timers prescaler to stop the clock
			PWM_CLEAR_PRESCALER();
			
			//Turn off pwm generation and turn off the pin
			PORTB &= ~(1 << PWM_OUT_A);
			PORTB |= (1 << PWM_OUT_ANTI_A);
			
		}
		//else if our duty cycle is at 100%...
		else if(dutyCycle >= 255)
		{
			//Clear the timers prescaler to stop the clock
			PWM_CLEAR_PRESCALER();
			
			//Turn off pwm generation, BUT turn ON the pin
			PORTB |= (1 << PWM_OUT_A);
			PORTB &= ~(1 << PWM_OUT_ANTI_A);
		}
		//else...
		else
		{
			#if defined(ACTIVE_PRESCALER_CHANGE) && ACTIVE_PRESCALER_CHANGE > 0
			//Read our prescaler value
			timer1Prescaler = GetPrescaler();
			#endif
			
			//Set our prescaler to make sure PWM is on
			PWM_SET_PRESCALER();
		}
		
		//Set our frequency control register
		OCR1C = frequencyValue;
		
		//Set our duty cycle control register
		OCR1A = dutyCycle;
		
    }
	
}



/**
* \brief Initializes all controller systems and peripherals
*
*/
void SystemInit()
{
	//Initialize pins and directions
	PORTB = 0;
	DDRB = 0xff;
	
	//Initialize ADC
	AdcSetup(ADC_PIN_MASK);
	
	//Initialize PWM
	PwmSetup();
	
	#if defined(PRESCALER_ON_STARTUP) && PRESCALER_ON_STARTUP > 0
	//Get our prescaler value
	timer1Prescaler = GetPrescaler();
	#endif
}



/**
* \brief Initializes ADC
*/
void AdcSetup(uint8_t pinMask)
{
	//Set as input low, internal pull up disabled
	ADC_PIN_OUTPUT_REGISTER &= ~(pinMask);
	ADC_PIN_DIR_REGISTER &= ~(pinMask);
	
	//Disable digital input
	DIDR0 |= (pinMask);
	
	//Set admux mode to REF mode 1 with right justified results
	ADMUX = 0;
	
	//Set for free running mode
	ADCSRB &= ~(1 << ADTS0 | 1 << ADTS1 | 1 << ADTS2);
	
	//Turn on ADC
	ADCSRA |= (1 << ADEN);
}



/**
* \brief Initializes the PWM
*
*/
void PwmSetup()
{
	//Make sure timer is cleared
	TCCR1 = 0;
	
	//Make sure pwm value registers are cleared
	OCR1A = 0;
	OCR1B = 0;
	
	//Make sure no interrupts are enabled
	TIMSK = 0;
	
	//Make sure frequency register is cleared
	OCR1C = 0;
	
	//Set the output mode to toggle our oc1a pin and that pwm A is enabled, 
	TCCR1 |= (1 << COM1A0 | 1 << PWM1A);
	
	//Reset prescaler to its default
	timer1Prescaler = (1 << CS10);
	
}



/**
* \brief Gets the value for the pwm prescaler
* \return the prescaler value
*/
uint8_t GetPrescaler()
{
	//Variables
	uint16_t adcValue = 0; //The adc reading for the prescaler
	uint8_t prescalerValue = 0; //The value for the prescaler
	
	//Calculate our prescaler...
	
	//Read ADC
	adcValue = SampleAdc(ADC_PRESCALER_CHANNEL, ADC_SAMPLES + ADC_SAMPLES);
	
	//Convert that to a percentage between min(0) and max, multiplied by our max value to get the current value
	prescalerValue = (uint8_t)(((float)adcValue/1023.0f) * 15.0f);
	
	//Range check our prescaler
	if(prescalerValue < 1)
	{
		prescalerValue = 1;
	}
	
	//Return our prescaler value
	return (prescalerValue & 0x0F);
}



/**
* \brief Handles pwm frequency control
* \return The value for the frequency register
*/
uint8_t GetFrequency()
{
	//Variables
	uint16_t adcValue = 0; //The adc reading for the frequency
	uint8_t frequencyValue = 0; //The value for the frequency register
	
	//Get our ADC reading
	adcValue = SampleAdc(ADC_FREQUENCY_CHANNEL, ADC_SAMPLES);
	
	//Convert that to a percentage between min and max, multiplied by our max value to get the current value
	frequencyValue = 255-(uint8_t)(((float)adcValue/1023.0f) * 255.0f);
	
	#if defined(FREQUENCY_NEVER_0) && FREQUENCY_NEVER_0 > 0
	//Range check our frequency value
	if(frequencyValue <= 0)
	{
		frequencyValue = 1;
	}
	#endif
	
	//Return our frequency
	return frequencyValue;
}



/**
* \brief Handles pwm duty cycle control
* \return The duty cycle register value
*/
uint8_t GetDutyCycle(uint8_t frequencyValue)
{
	//Variables
	uint16_t adcValue = 0; //The adc reading for the duty cycle
	uint8_t dutyCycle = 0; //The value for the duty cycle register
	
	//Get our ADC reading
	adcValue = SampleAdc(ADC_DUTY_CYCLE_CHANNEL, ADC_SAMPLES);
	
	//Convert that to a percentage between min and max, multiplied by our max value to get the current value
	dutyCycle = (uint8_t)(((float)adcValue/1023.0f) * (float)frequencyValue);
	
	//Return our value
	return dutyCycle;
}



/**
* \brief Samples ADC for sampleCount Counts on channel adcChannel, returning the average
*
*/
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount)
{
	//Variables
	uint16_t adcResult = 0; //Return value from the ADC
	
	//If the sample count is greater than 0, avoid divide by 0 errors,...
	if(sampleCount > 0)
	{
		//Create a variable for the loop
		uint8_t i = 0;

		//Select the passed channel
		ADMUX |= adcChannel;

		//While the index is less than the passed sample count...
		while(i < sampleCount)
		{
			i++;

			//Start our conversion
			ADCSRA |= (1 << ADSC);
			
			//Wait until the conversion is finished
			while(((ADCSRA >> ADSC) & 0x01));
			
			//Add our result
			adcResult += ADC;
		}
		
		//Divide our result by the passed sample count
		adcResult /= sampleCount;

		//Make sure the clear the selected channel on the way out
		ADMUX &= ~adcChannel;

	}
	
	//Return our result
	return adcResult;
}
