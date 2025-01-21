#include <Arduino.h>
#include <ens220.h>
#include "i2c_interface.h"

using namespace ScioSense;

/////////////////////////////////////////////////////////////////
//                                                             //
//    ENS220: High-accuracy pressure and temperature  sensor   //
//    ------------------------------------------------         //
//                                                             //
//    Application example 1: Indoor (pressure) event detection //
//                                                             //
/////////////////////////////////////////////////////////////////


//This algorithm enables the detection of window and door opening events in a building using the high-precision ENS220 pressure and temperature sensor

// The ENS220 offers precise air pressure measurements with high resolution and sampling rate at low power consumption. Events such as the opening and 
// closing of doors and windows lead to small and rapid changes in air pressure. These changes can be clearly detected with the ENS220 due to its
// high resolution and sampling rate.

// PLEASE READ OUR APPLICATION NOTE TO UNDERSTAND THIS CODE EXAMPLE AND ITS USE CASE: https://www.sciosense.com/products/pressure-sensor/ens220/


// Define the I2C address of the ENS220
#define I2C_ADDRESS 0x20

// Define the speed of the Serial communication with the computer
#define SERIAL_BAUDRATE 57600

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220

// You can (optionally) connect an LED to visually indicate pressure events
#define OPTIONAL_LED_PIN 19
bool ledState = LOW;

// Create an ENS220 sensor object
ENS220 ens220;

// Create an I2C interface
I2cInterface i2c_1;

// Declare/define variables for data readout
float relative_time_ms;
int last_readout_time_ms = 0;
float absolute_pressure_Pa;
float first_pressure_Pa = 0.0;
float relative_pressure_Pa; // Absolute Pressure (t) - Absolute Pressure (t_0) 
int target_sampling_interval_ms = 64; //NOTE: The algorithm parameters are optimized for this interval. Do not change it.

// Define event detection algorithm  parameters
float event_threshold_multiplier = 6.0; // Event threshold multiplier, event is detected if filtered pressure derivative is m times larger than average variation
float minimum_event_duration_s = 1.5; // Minimum event duration in seconds, once the filtered pressure derivative is passed the threshold the event continues for at least d seconds. This is used to prevent multiple event detection from one event signal
float average_calculation_decay_rate = 0.9999;  // Decay rate for calculating mean absolute deviation (mad) using exponential moving average. Example: average_calculation_decay_rate=1: average with no decaye, average_calculation_decay_rate =0.9999 : average approximately 10000 samples
// Structure declaration for holding the states     // These are variables that needs to be presisted if we want to run the algo on per sample basis
typedef  struct  {   
  int event_detected;            // Event detected
  int first_event_lower_than_threshold; // Indicates first sample in the event where the signal (filtered pressure derivative) becomes smaller than threshold
  int first_event_detected;      // Indicates the first sample that the event is detected
  float norm;                    //normalizing constant for average calculation, initital value =1
  float average;                 // Average value of filtered derivative of pressure, used for calculating mean absolute deviation
  float mean_absolute_deviation; // Mean absolute deviation of filtered derivative of pressure, initial value = 2.0
  float p_1;    // Variable for keeping one time lagged sample of pressure p_1 = p[n-1]= p[t- Ts] where t is time and Ts is sample time
  float t_1;    // Variable for keeping one time lagged value of time t_1 = t- Ts. Thus t-t_1= Ts
  float dp;     // Derivative of pressure: dp[n]
  float dpf;    // Derivative of pressure after filtering: dpf[n-1]
  float dp_1;   // Variable for keeping one time lagged sample of derivative of pressure: dp[n-1]
  float dp_2;   // Variable for keeping two time lagged sample of derivative of pressure: dp[n-2]
  float dpf_1;  // dpf[n-1]
  float dpf_2;  // dpf[n-2] 
  float detection_threshold;     // Threshold for event detection
  float t_first_event_lower_than_threshold; // Time that signal is first time smaller than threshold
} State;
State state;

/////////// ALGORITHM ///////////////
void detect_events(float p, float t, float m, float d, float alpha) {
  
  // Filter coefficients for filtering derivate of pressure. Second order Butterworth filter
  // dpf[n] = -a[1]* dpf[n-1] -a[2] * dpf[n-2] + b[0] dp[n]+ b[1] dp[n-1]+ b[2] dp[n-2] 
  float b[] = {0.06745527, 0.13491055, 0.06745527}; 
  float a[] = {1.0, -1.1429805, 0.4128016};
  float event_weight_in_average = 1/8.0;
 
  state.dp_2 = state.dp_1; // Time lag for filtering
  state.dp_1 = state.dp;   // Time lag for filtering
  if ((t - state.t_1) == 0) {
    state.dp = 0;
  } else {
    state.dp = (p - state.p_1) / (t - state.t_1); // Derivative
  }
  state.p_1 = p; //Time lag for calculation of derivative
  state.t_1 = t;

  state.dpf_2 = state.dpf_1;   // Time lag for filtering
  state.dpf_1 = state.dpf;    // Time lag for filtering

  state.dpf = -a[1] * state.dpf_1 - a[2] * state.dpf_2 + b[0] * state.dp + b[1] * state.dp_1 + b[2] * state.dp_2;  // IIR filter

  state.detection_threshold = state.mean_absolute_deviation * m; // Threshold calculation

  //Update mean absolute variation
  if (fabs(state.dpf) < state.detection_threshold) { // update when not in event
      state.norm = state.norm * alpha + 1.0;
      state.average = state.average + (state.dpf - state.average) / state.norm;
      state.mean_absolute_deviation = state.mean_absolute_deviation + (fabs(state.dpf - state.average) - state.mean_absolute_deviation) / state.norm;
  } else { // Update slower during event. To make update during event slower change 8.0 to a larger number.
      state.average = state.average + (state.dpf - state.average) * event_weight_in_average / state.norm;
      state.mean_absolute_deviation = state.mean_absolute_deviation + (fabs(state.dpf - state.average) - state.mean_absolute_deviation) * event_weight_in_average / state.norm;
  }

  // Event detection 
  if (fabs(state.dpf) > state.detection_threshold) { // larger than threshold
      state.event_detected = 1;
      state.first_event_detected = 1;
      state.first_event_lower_than_threshold = 1;
      if (state.first_event_detected) {
          state.first_event_detected = 0;
      }
  } else { //smaller than threshold
      if (state.event_detected) {
          if (state.first_event_lower_than_threshold) {
              state.t_first_event_lower_than_threshold = t;
              state.first_event_lower_than_threshold = 0;
          }
          // Event duration d is used to indicate the end-time for an event after signal goes lower than threshold. 
          //The signal might go momentarily lower than threshold and go higher again. The time threshold is used so that we don't detect a single event as multiple ones.
          if ((t - state.t_first_event_lower_than_threshold) > d) {  
              state.event_detected = 0;
              state.first_event_detected = 1;
          }
      }
  }
}

void setup()
{
    pinMode(OPTIONAL_LED_PIN, OUTPUT);
    Serial.begin(SERIAL_BAUDRATE);

    Wire.begin();
    
    #ifdef DEBUG_ENS220
        ens220.enableDebugging(Serial);
    #endif

    ens220_setup();

    // Initialize algorithm
    relative_time_ms = millis();
    auto result = ens220.update();
    while(result != ENS220::Result::Ok) {
      relative_time_ms = millis();
      result = ens220.update();
    }
    first_pressure_Pa = ens220.getPressurePascal();
    state = {.event_detected=0, .first_event_lower_than_threshold=0, .first_event_detected=1, .norm=1, .average = 0, .mean_absolute_deviation=2.0, .p_1=first_pressure_Pa, .t_1=relative_time_ms/1000.};

}

void ens220_setup()
{
    Serial.println("Starting ENS220 example 01_Basic_I2C_SingleShot");
    
    // Start the communication, confirm the device PART_ID, and read the device UID
    i2c_1.begin(Wire, I2C_ADDRESS);
    
    while(ens220.begin(&i2c_1) != true)
      {
        Serial.println("Waiting for I2C to start");
        delay(1000);
      }
    
    Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);

    // Choose the desired configuration of the sensor. In this example we will use the settings described in the application note
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_8_2);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP) 
    ens220.setOversamplingOfPressure(ENS220::Oversampling::N_32);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_4);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_4);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220::StandbyTime::ContinousOperation);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();

    // Start continous
    ens220.startContinuousMeasure(ENS220::Sensor::Pressure);
}

void loop()
{ 

  while ((((int)millis())-last_readout_time_ms) >= (target_sampling_interval_ms)) {

    // Check the DATA_STAT from the sensor. Read data, if available
    relative_time_ms = millis();
    auto result = ens220.update();
    
    if(result == ENS220::Result::Ok)
    {

      // Get the values that were collected during the ens220.update()
      absolute_pressure_Pa = ens220.getPressurePascal();
      relative_pressure_Pa = absolute_pressure_Pa - first_pressure_Pa;

      // Feed the algorithm with the new values
      detect_events(absolute_pressure_Pa, relative_time_ms/1000., event_threshold_multiplier, minimum_event_duration_s, average_calculation_decay_rate);

      // Print all the current values (they can be plotted with the Serial Plotter)
      Serial.print("Abs_P[hPa]:");
      Serial.print(absolute_pressure_Pa);

      Serial.print("\tRel_P[Pa]:");
      Serial.print(relative_pressure_Pa);

      Serial.print("\tEvent_detected:");
      Serial.println(state.event_detected);

      // Light up LED if event was detected (if connected)
      toggle_led();

      last_readout_time_ms = relative_time_ms;
      
    } else {
      Serial.println("Read-out skipped. No data available");
    }
  }
}

void toggle_led() {
  if (state.event_detected == 1) {
    turn_led_on();
  } else {
    turn_led_off();
  }
}

void turn_led_on() {
  if (ledState == LOW) {
    digitalWrite(OPTIONAL_LED_PIN, HIGH);
    ledState = HIGH;
  }
}
void turn_led_off() {
  if (ledState == HIGH) {
    digitalWrite(OPTIONAL_LED_PIN, LOW);
    ledState = LOW;
  }
}