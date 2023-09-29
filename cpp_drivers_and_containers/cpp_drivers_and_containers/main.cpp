/********************************************************************************
 * @brief Demonstration of GPIO device drivers in C++.
 ********************************************************************************/
#include <drivers.hpp> 
#include <lin_reg.hpp>

using namespace yrgo::driver;
using namespace yrgo::container;

/********************************************************************************
 * @brief Devices and models used in the embedded system.
 *
 * @param model
 *        Linear regression model for predicting the temperature.
 * @param button1
 *        Button used to toggle the LED between blinking and being disabled.
 * @param timer0
 *        Timer used to reduced the effect of contact bounces when pressing
 *        the button.
 * @param timer1
 *        Timer used to predict temp every 60 seconds.
 ********************************************************************************/
static yrgo::LinReg model{};
static GPIO tmp1{2, GPIO::Direction::kInput};
static GPIO button1{13, GPIO::Direction::kInputPullup};
static Timer timer0{Timer::Circuit::k0, 300};
static Timer timer1{Timer::Circuit::k1, 60000};

namespace {

/*********************************************************************************
 * @brief Predicts and prints temperature based on input voltage.
 *
 * This function reads an analog voltage signal from channel 2 using the adc::Read function,
 * scales it to a voltage value between 0 and 5, and then uses a predictive model (presumably stored
 * in the 'model' variable) to estimate the temperature corresponding to the input voltage.
 * The predicted temperature is then rounded using the utils::Round function and printed to the serial output.
 **********************************************************************************/
void PredictTemp(void) {
	const double uin = adc::Read(2) / (double)adc::kMaxVal * 5.0;
	const auto temp = model.Predict(uin);
	serial::Printf("Temp: %d\n", utils::Round(temp));
}

/********************************************************************************
 * @brief Callback routine called when button1 is pressed or released.
 *        Every time button1 is pressed, timer1 is toggled, which indirectly
 *        toggles the LED (since timer1 is responsible for toggling led1).
 *        Pin change interrupts are disabled for 300 ms on the button's I/O port
 *        to reduce the effects of contact bounces.
 ********************************************************************************/
void ButtonCallback(void) {
    button1.DisableInterruptsOnIoPort();
    timer0.Start();
	if (button1.Read()) {
		PredictTemp();
		timer1.Restart();
	}
}

/********************************************************************************
 * @brief Enabled pin change interrupts on the button's I/O port 300 ms after
 *        press or release to reduce the effects of contact bounces.
 ********************************************************************************/
void Timer0Callback(void) {
    if (timer0.Elapsed()) {
        timer0.Stop();
	    button1.EnableInterruptsOnIoPort();
	}
}

/********************************************************************************
 * @brief Toggles PreditctTemp when timer1 elapsed, which is every 100 ms 
 *        when enabled.
 ********************************************************************************/
void Timer1Callback(void) {
    if (timer1.Elapsed()) {
        PredictTemp();   
    }
}

/********************************************************************************
 * @brief Sets callback routines, enabled pin change interrupt on button1 and
 *        enables the watchdog timer in system reset mode.
 ********************************************************************************/
inline void Setup(void) {
	const Vector<double> inputs{{0.0, 1.0, 2.0, 3.0, 4.0}};
	const Vector<double> outputs{{-50.0, 50.0, 150.0, 250.0, 350.0}};
	model.LoadTrainingData(inputs, outputs);
	model.Train(1000);
	
	serial::Init();
	PredictTemp();
	timer1.Start();
	
	button1.SetCallbackRoutine(ButtonCallback);
	timer0.SetCallback(Timer0Callback);
    timer1.SetCallback(Timer1Callback);

	button1.EnableInterrupt();
	watchdog::Init(watchdog::Timeout::k1024ms);
	watchdog::EnableSystemReset();
}

} /* namespace */

/********************************************************************************
 * @brief Perform a setup of the system, then running the program as long as
 *        voltage is supplied. The hardware is interrupt controlled, hence the
 *        while loop is almost empty. If the program gets stuck anywhere, the
 *        watchdog timer won't be reset in time and the program will then restart.
 ********************************************************************************/
int main(void)
{
    Setup();

    while (1) 
    {
	    watchdog::Reset();
    }
	return 0;
}