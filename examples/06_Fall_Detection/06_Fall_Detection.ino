#include <Arduino.h>
#include <ens220.h>
#include "i2c_interface.h"
#include "DropAlgorithm.h"

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220
// If the following line is un-commented, all the collected pressure values and algorithm output will be printed to the Serial port
//#define DEBUG_MEASUREMENTS

// Interface hardware configuration
#define INTN_1 23               // Pin connected to the ENS220 interrupt output
#define SERIAL_BAUDRATE 921600  // Speed of the Serial communication with the computer

// Algortihm parameters
#define ENS220_BUFFERSIZE       32  // Amount of samples to read from the ENS220 at a time
#define ALGORITHM_POINTS_WINDOW 32  // Number of samples to evaluate if a drop and impact are present
#define MIN_HEIGHT_DIFF_M       0.6 // Minimum difference height to register a fall
#define MIN_IMPACT_HEIGHT_M     1.0 // Minimum pressure spike to register a fall expressed in meters
#define BASELINE_MAX_DIFF_TH_M  0.5 // Maximum difference in height in window to register values as a valid baseline expressed in meters
#define T_SUBSAMPLE_MS          500 // Period in milliseconds to add a new pressure value as a baseline if the value is constant in the window
#define AMOUNT_BASELINES        4   // Amount of previous pressure baseline values to calculate the relative pressure increase
#define MAXIMUM_FALL_HEIGHT_M   4   // Maximum height of detected fall for the algorithm to output

// ENS220 definitions
#define ENS220_I2C_ADDRESS 0x20
#define CONVERSION_PA_TO_RAW  64 // Conversion factor from Pa to raw ENS220 measurement

// Physics definitions
#define R_SPECIFIC_DRY_AIR_J_KGK  287.05                // Specific gas constant for dry air in J / (K * kg)
#define GRAVITATIONAL_ACCELERATION_SURFACE_M_S2 9.81    // Acceleration of gravity at the surface of the Earth in m/s2


using namespace ScioSense;

// Object creation
I2cInterface i2c_1;                     // I2C communication object
ENS220 ens220;                          // ENS220 object
DropDetectionState* algorithm_state;     // Algorithm state struct
AlgorithmConstants algorithm_constants; // Algorithm constants struct

// Global variables
float latestPressure = 0;
float latestTemp = 0;
uint32_t maximum_fall_pressure_peak_raw = 0;
float conversion_pa_to_m = 0;
unsigned long current_ms = 0;

// Variables containing the measurements from the last points
uint32_t meas_buffer[ENS220_BUFFERSIZE];
unsigned long time_buffer[ENS220_BUFFERSIZE];
uint8_t algorithm_state_buffer[ENS220_BUFFERSIZE];
uint16_t algorithm_window_idx_buffer[ENS220_BUFFERSIZE];
uint8_t algorithm_drop_event_buffer[ENS220_BUFFERSIZE];

int t_individual_meas_ms = 2;   // Time it takes to take a measurement, depends on configuration

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Bungee demo starting");    

    #ifdef DEBUG_ENS220
        ens220.enableDebugging(Serial);
    #endif         

    Wire.begin();
    StartEns2210I2cCommunication();

    float temperature_k = 0;
    float pressure_pa = 0;
    while( pressure_pa == 0)
    {
        ConfigureENS220SingleShotMode();
        SingleShot_Measure(&temperature_k, &pressure_pa);
        delay(300);
    }

    conversion_pa_to_m = GetConversionFactorDryAirPaToM(temperature_k, pressure_pa);
    maximum_fall_pressure_peak_raw = MAXIMUM_FALL_HEIGHT_M * CONVERSION_PA_TO_RAW / conversion_pa_to_m;

    ConfigureAlgorithmConstants(MIN_HEIGHT_DIFF_M, MIN_IMPACT_HEIGHT_M, BASELINE_MAX_DIFF_TH_M, ALGORITHM_POINTS_WINDOW, T_SUBSAMPLE_MS);

    InitializeAlgorithmState(pressure_pa * CONVERSION_PA_TO_RAW);

    Serial.println("Algorithm ready");
    
    ConfigureENS220ContinuousMode();
    Serial.println("ENS220 configured");

    // Measure the initial temperature, so that all the rest of the measurements are pressure
    ContinuousModeWithFIFO_loop(meas_buffer);
    delay(100);

    Serial.println("Ready to detect drop events");

    current_ms = millis();
}

void loop()
{        
    ContinuousModeWithFIFO_loop(meas_buffer);

    uint32_t measured_drop_height_raw = 0;
    unsigned long data_collection_finished = millis();
    unsigned long curr_time = data_collection_finished - (ENS220_BUFFERSIZE-1) * t_individual_meas_ms;

    for(int measurement=0; measurement<ENS220_BUFFERSIZE; measurement++) 
    {      
        detect_drop_impact_state(algorithm_state, meas_buffer[measurement], curr_time, algorithm_constants);
        algorithm_state_buffer[measurement] = algorithm_state->state;
        algorithm_window_idx_buffer[measurement] = algorithm_state->index_window;
        algorithm_drop_event_buffer[measurement] = algorithm_state->drop_event;

        if(algorithm_state->drop_event)
        {
            // A drop was detected and the measurement completed
            measured_drop_height_raw = algorithm_state->spike_height;  
            
            // Coerce drop height to the harcoded limits
            if( measured_drop_height_raw > maximum_fall_pressure_peak_raw )
            {
                measured_drop_height_raw = maximum_fall_pressure_peak_raw;
            }
            if(measured_drop_height_raw < 0)
            {
                measured_drop_height_raw = 0;
            }
        }  
        time_buffer[measurement] = curr_time;
        curr_time += t_individual_meas_ms;        
    }
    
    #ifdef DEBUG_MEASUREMENTS
        for(int measurement=0; measurement<ENS220_BUFFERSIZE; measurement++) 
        {
            Serial.print(time_buffer[measurement]);
            Serial.print(",");
            Serial.print(meas_buffer[measurement]);
            Serial.print(",");
            Serial.print(algorithm_state_buffer[measurement]);
            Serial.print(",");
            Serial.print(algorithm_window_idx_buffer[measurement]);
            Serial.print(",");
            Serial.println(algorithm_drop_event_buffer[measurement]);
        }
    #endif

    if(measured_drop_height_raw > 0)
    {
        Serial.print("Drop ");
        // We transform the raw impact height into pressure in Pascals and then into height in meters
        float drop_height = ((float)measured_drop_height_raw) * conversion_pa_to_m / CONVERSION_PA_TO_RAW; 
        Serial.print(drop_height);
        Serial.println("m");             
        measured_drop_height_raw = 0;
    }     
}

void StartEns2210I2cCommunication()
{    
    // Start the communication, confirm the device PART_ID, and read the device UID
    i2c_1.begin(Wire, ENS220_I2C_ADDRESS);    
    
    while(ens220.begin(&i2c_1) != true)
    {
        Serial.println("Waiting for I2C to start");
        delay(1000);
    }
    Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);
}

void ConfigureENS220ContinuousMode()
{
    Serial.println("Starting ENS220 acquisition");

    // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_8_2);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP) 
    ens220.setOversamplingOfPressure(ENS220::Oversampling::N_1);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_1);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_32);        
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220::PressureDataPath::Fifo);
    // Enable interrupts (INTF_CFG register, field INT_EN)
    ens220.setInterfaceConfiguration(ENS220::InterfaceConfiguration::InterruptEnable);
    // Choose events that generate interrupts (INT_CFG register)
    ens220.setInterruptConfiguration(ENS220::InterruptConfiguration::TemperatureDataReady | ENS220::InterruptConfiguration::PressureFifoFull); 
    
    // Set which pin to use to poll for interrupts
    ens220.setInterruptPin(INTN_1);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();

    // Start continuous measurements
    ens220.startContinuousMeasure(ENS220::Sensor::Pressure);
}

void ContinuousModeWithFIFO_loop(uint32_t *buffer)
{    
    // Poll the interrupt pin until a new value is available
    ens220.waitInterrupt();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result= ens220.update();
    if(result == ENS220::Result::Ok)
    {      
        if(hasFlag(ens220.getInterruptStatus(), ENS220::InterruptStatus::FifoFull))
        {
            for(int i=0; i<ENS220_BUFFERSIZE; i++)
            {
                buffer[i] = ens220.getPressureRaw(i);              
            }            
        }

        if(hasFlag(ens220.getInterruptStatus(), ENS220::InterruptStatus::Temperature))
        {
            // Send the temperature value that was collected during the ens220.update()
            Serial.print("T[C]:");
            Serial.println(ens220.getTempCelsius());          
        }
    }
}

void ConfigureENS220SingleShotMode()
{    
    // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP) 
    ens220.setOversamplingOfPressure(ENS220::Oversampling::N_128);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_128);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);
    // Write the desired configuration into the sensor
    ens220.writeConfiguration();
}

void SingleShot_Measure(float* temperature_k, float* pressure_pa)
{
    // Start single shot measurement
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);

    // Wait until the measurement is ready
    ens220.waitSingleShot();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result = ens220.update();   
    if(result == ENS220::Result::Ok)
    {
        if(hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady))
        {
            *pressure_pa = ens220.getPressurePascal();
            *temperature_k = ens220.getTempKelvin();
        }
    }
}

void ConfigureAlgorithmConstants(double min_height_diff_m, double min_impact_height_m, double max_height_diff_m, int32_t window_size, 
    double baseline_update_period)
{
    algorithm_constants.min_diff_th = (int)(min_height_diff_m * CONVERSION_PA_TO_RAW / conversion_pa_to_m);
	algorithm_constants.min_spike_th = (int)(min_impact_height_m * CONVERSION_PA_TO_RAW / conversion_pa_to_m); 
	algorithm_constants.baseline_max_diff_th = (int)(max_height_diff_m * CONVERSION_PA_TO_RAW / conversion_pa_to_m);
    algorithm_constants.baseline_update_period = baseline_update_period;
    algorithm_constants.window_size = window_size;
}

void InitializeAlgorithmState(int initial_pressure)
{
    algorithm_state = CreateAlgorithmStruct(AMOUNT_BASELINES);
    InitializeAlgorithmStruct(algorithm_state, initial_pressure);
}

float GetConversionFactorDryAirPaToM(float temperature_k, float pressure_pa)
{
    // Calculates the conversion factor from a pressure difference in Pascal to a height in meters
    // Assumes dry air properties  

    // Calculate density of dry air in kg / m3
    float air_density_dry_air_kg_m3 = pressure_pa / (R_SPECIFIC_DRY_AIR_J_KGK * temperature_k);
    // The pressure difference between two heights is the weight of the fluid divided by unit area
    return ( 1.0 / (air_density_dry_air_kg_m3 * GRAVITATIONAL_ACCELERATION_SURFACE_M_S2) );
}
