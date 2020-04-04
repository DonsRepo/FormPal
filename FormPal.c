
//*****************************************************************************
//
// FormPal - records the path of an exercise, then offers feedback to the user
// about future iterations of the same exercise being different. Uses TM4C123GXL
// and Invensense 9150 IMU on Senshub Boosterpack
//
// by Don Reynolds for Engineering 478 - Design with Microprocessors
// San Francisco State University, School of Engineering
//*****************************************************************************



#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "drivers/buttons.c"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include <TM4C123GH6PM.h>

//global acceleration threhold
#define GESTURE_EMIT_THRESHOLD_ACCEL    \
                               0.1f
//used to determine if user at rest
#define GESTURE_EMIT_THRESHOLD_EULERS_REST				\
															 0.02f
//used to determine if user is moving. set higher to remove jitter
#define GESTURE_EMIT_THRESHOLD_EULERS_MOTION				\
															 1.7f
//delta for accelerometer deviation from trained model
#define GESTURE_EMIT_MOTION_DELTA				\
															 1.7f
															 
//delta for euler angle deviation from trained model
#define GESTURE_EMIT_EULERS_DELTA				\
															 10.0f														 

//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        5

uint32_t g_ui32PrintSkipCounter;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif





//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}


static void
//*****************************************************************************
//
//Multiply a vector by a matrix
//
//*****************************************************************************

MatrixVectorMul(float pfVectorOut[3], float ppfMatrixIn[3][3],
                float pfVectorIn[3])
{
    uint32_t ui32X, ui32Y;

    //
    // Loop through the rows of the matrix.
    //
    for(ui32Y = 0; ui32Y < 3; ui32Y++)
    {
        //
        // Initialize the value to zero
        //
        pfVectorOut[ui32Y] = 0;

        //
        // Loop through the columns of the matrix.
        //
        for(ui32X = 0; ui32X < 3; ui32X++)
        {
            //
            // The answer to this vector's row's value is the sum of each
            // column value multiplied by each vector's row value.
            //
            pfVectorOut[ui32Y] += (ppfMatrixIn[ui32Y][ui32X] *
                                   pfVectorIn[ui32X]);
        }
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Set RGB Color to RED
    //
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
    RGBColorSet(g_pui32Colors);

    //
    // Increase blink rate to get attention
    //
    RGBBlinkRateSet(10.0f);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Calculate change in acceleration and Euler angles from sensor data, return it back in an 
// array. This is called wherever sensor data is updated within the main function.
//
//*****************************************************************************
		

float* SensorUpdateMath(float *pfAccelNet, float *pfEulers)
{	    static float pfSensorCalc[5];
			static float fAccelMagnitude, fJerk, g_fAccelMagnitudePrevious;
			static float Roll, Pitch, Yaw, PreviousRoll, PreviousPitch, PreviousYaw,
			JerkRoll, JerkPitch, JerkYaw;
			
			fAccelMagnitude = (pfAccelNet[0] * pfAccelNet[0]) +  // get local acceleration magnitude
												(pfAccelNet[1] * pfAccelNet[1]) +
												(pfAccelNet[2] * pfAccelNet[2]);

			// Subtract from previous accel magnitude, measuring delta acceleration

			fJerk = fAccelMagnitude - g_fAccelMagnitudePrevious;
			g_fAccelMagnitudePrevious = fAccelMagnitude;

			//get Euler angles, convert to degrees

			pfEulers[0] *= 57.295779513082320876798154814105f;
			pfEulers[1] *= 57.295779513082320876798154814105f;
			pfEulers[2] *= 57.295779513082320876798154814105f;

			if(pfEulers[2] < 0)
			{
					pfEulers[2] += 360.0f;
			}
			Roll = pfEulers[0];
			Pitch = pfEulers[1];
			Yaw = pfEulers[2];

			//calculate changes in Eulers angles
			JerkRoll = Roll-PreviousRoll;
			JerkPitch=Pitch-PreviousPitch;
			JerkYaw = Yaw-PreviousYaw;
			JerkRoll = fabs(JerkRoll);
			JerkPitch= fabs(JerkPitch);
			JerkYaw=fabs(JerkYaw);
			PreviousRoll = Roll;
			PreviousPitch = Pitch;
			PreviousYaw = Yaw;
		  pfSensorCalc[0] = JerkRoll;
			pfSensorCalc[1] = JerkPitch;
			pfSensorCalc[2] = JerkYaw;
			pfSensorCalc[3] = fJerk;
			pfSensorCalc[4] = fAccelMagnitude;
			
			return pfSensorCalc;

}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int main(void)
{
    int i = 0;
    int_fast32_t i32IPart[16], i32FPart[16];
    uint_fast32_t ui32Idx, ui32CompDCMStarted;
    float pfData[16];
    float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
    volatile bool Trainingmodebeginflag;
	
	    //create variables to store useful data from accelerometer and eulers
    //"jerk" variables serve as
    volatile float fAccelMagnitude, fJerk;
    float pfAccelNet[3];
    float ppfDCM[3][3];
    volatile float g_fAccelMagnitudePrevious;
    volatile float Roll, Pitch, Yaw, PreviousRoll, PreviousPitch, PreviousYaw,
             JerkRoll, JerkPitch, JerkYaw;

//		float pfSensorCalc[5]; //holds results of sensor calculations
		
    //training timers
    unsigned int TrainingStartTimer; //Training gesture ready wait time
    unsigned int TrainingRunTimer; //timers for learning gesture in two parts separated by apex
    unsigned int WaitForMotionTimer = 0;
		volatile bool TrainingModeStartFlag=0; //1=training mode active
		volatile bool TrainingEndFlag = 0;
    volatile bool TrainingReadyFlag = 0;
    volatile bool ExerciseMode = 0;

    //timers for inactivity at training mode start
    TrainingStartTimer = 0;
    TrainingRunTimer = 0;

    //Timer that helps reset exercise features to 0 when rest exceeds time limit
    unsigned int ExerciseModeRestTimer = 0;

    //training variables
    volatile float minAccelTraining[3], maxAccelTraining[3], minEulersTraining[3], maxEulersTraining[3], TrialExerciseTimeTraining,
             TrialAccelMagnitudeTraining, TrialSpeedTraining;

    //exercise trial variables
    volatile float minAccel[3], maxAccel[3], minEulers[3], maxEulers[3], TrialExerciseTime,
             TrialAccelMagnitude, TrialSpeed;
    //
    // Initialize convenience pointers that clean up and clarify the code
    // meaning. We want all the data in a single contiguous array so that
    // we can make our pretty printing easier later.
    //
    pfAccel = pfData;
    pfGyro = pfData + 3;
    pfMag = pfData + 6;
    pfEulers = pfData + 9;
    pfQuaternion = pfData + 12;

    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable port B used for motion interrupt.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JMPU9150 Raw Example\n");

    //
    // Set the color to a White, shows that it's yet to be trained
    //
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[BLUE] = 0x8000;
    g_pui32Colors[GREEN] = 0x8000;

    //
    // Initialize RGB and buttons driver.
    //
    ButtonsInit();
    volatile uint8_t ui8ButtonsChanged, ui8Buttons;

    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(0.5f);
    RGBEnable();

    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOB);

    //
    // Keep only some parts of the systems running while in sleep mode.
    // GPIOB is for the MPU9150 interrupt pin.
    // UART0 is the virtual serial port
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    // I2C3 is the I2C interface to the ISL29023
    //
    ROM_SysCtlPeripheralClockGating(true);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                 MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                 MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

    ui32CompDCMStarted = 0;




    while(1)
    {

        //
        //Pressing left button enters training mode, the white LED color changes to slowly flash yellow.
        // A period of roughly 3 seconds of inactivity is required for training mode to begin learning the ideal
        // exercise path. This is referred to as the "training gesture ready wait time" because the device is waiting
        // for the user to get ready to perform the exercise
        //

        ButtonsPoll(&ui8ButtonsChanged, &ui8Buttons); //check for button press

        if (ui8ButtonsChanged && !TrainingModeStartFlag)
        {
            if(ui8Buttons && !TrainingModeStartFlag) //enter training mode if left switch is pressed
            {
                RGBBlinkRateSet(1.0f);
                UARTprintf("\n \n Button pressed registered"); //for debugging

                g_pui32Colors[RED] = 0x8000; //set LED to flash yellow
                g_pui32Colors[BLUE] = 0x0000;
                g_pui32Colors[GREEN] = 0x8000;
                RGBColorSet(g_pui32Colors);

                TrainingModeStartFlag = 1; //set training mode flag

            }
        }

        if(TrainingModeStartFlag)
        {
            UARTprintf("\n \n Training mode start flag up");

            ///////////////////
            // movement below threshold, advance timer to end "gesture ready wait time"
            //
            while(TrainingStartTimer<30000 && (fJerk < GESTURE_EMIT_THRESHOLD_ACCEL) |
                    (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL)&& (JerkPitch < GESTURE_EMIT_THRESHOLD_EULERS_REST) &&
                    (JerkRoll < GESTURE_EMIT_THRESHOLD_EULERS_REST))
            {
                TrainingStartTimer = TrainingStartTimer+1; 	//advance gesture ready start timer, no movement
                // if no movement
                UARTprintf("\n \n Training mode standby, movement below threshold");

                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1], //get local update of sensor readings
                                   pfAccel[2]);
                //get Euler angles, convert to degrees
                CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                     pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);							//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
							
								float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[5];

            }

            ///////////////////
            // movement above threshold, reset timer for "gesture ready wait time"
            //

            while(TrainingStartTimer<30000 && (fJerk > GESTURE_EMIT_THRESHOLD_ACCEL | (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL)
                                               | (JerkPitch > GESTURE_EMIT_THRESHOLD_EULERS_REST) | (JerkRoll > GESTURE_EMIT_THRESHOLD_EULERS_REST)
                                               | (JerkYaw > GESTURE_EMIT_THRESHOLD_EULERS_REST) ))
            {
                WaitForMotionTimer++;//advance timer to require slight movement before ending pre-training phase
								UARTprintf("\033[19;75H\n \n Training mode standby, movement detected");
                TrainingStartTimer = 0;											//if movement goes above thresholds, reset timer

							
                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);
				        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                  pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
								
							  float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[4];


            }

            ///////////////////
            // Timer elapsed, user is ready to train system for movement
            // Note: very small "Wait for motion timer" exists to keep 
						// system from exiting pre training too early
						
            if(TrainingStartTimer==30000 && WaitForMotionTimer>5)
            {
                TrainingReadyFlag=1; //this flag enters the "training ready" block below this one
                RGBBlinkRateSet(5.0f);
                TrainingModeStartFlag=0;
                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);


                //get Euler angles, convert to degrees
                CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                     pfEulers + 2);

                continue;
            }



            ////////////////////////////////////////
            //!Ready for training motion.
            //!Learn the user's path through the exercise, including acceleration and yaw, pitch, roll.
            //!Data is captured while in motion above threshold, not captured while motion is below threshold.
            //!Trainins is completed by user pressing SW1/left button.

        }
        if(TrainingReadyFlag)
        {
            //////////////////////////////////////
            //!
            //! no movement, do not record sensor output
            //!
            while(!TrainingEndFlag && TrainingReadyFlag && ((fJerk < GESTURE_EMIT_THRESHOLD_ACCEL) |
                    (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL)&& (JerkPitch < GESTURE_EMIT_THRESHOLD_EULERS_MOTION) &&
                    (JerkRoll < GESTURE_EMIT_THRESHOLD_EULERS_MOTION) && (JerkPitch < GESTURE_EMIT_THRESHOLD_EULERS_MOTION)))
            {
                g_pui32Colors[RED] = 0x8000; //set LED to flash green
                g_pui32Colors[BLUE] = 0;
                g_pui32Colors[GREEN] = 0x8000;
                RGBColorSet(g_pui32Colors);

                // Get floating point version of the Accel Data in m/s^2.
                //
                MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
                                         pfAccel + 2);

                //
                // Get floating point version of angular velocities in rad/sec
                //
                MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
                                        pfGyro + 2);

                //
                // Get floating point version of magnetic fields strength in tesla
                //
                MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
                                           pfMag + 2);

                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);
				        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                  pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
								
							  float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[4];

                //
                //left button press (SW2) ends training mode here
                //
                ButtonsPoll(&ui8ButtonsChanged, &ui8Buttons); //check for button press
                if (ui8ButtonsChanged)
                {
                    if(ui8Buttons)
                    {
                        UARTprintf("\n \n Button pressed registered, training mode ended"); //for debugging

                        g_pui32Colors[RED] = 0x8000; //set LED to flash yellow
                        g_pui32Colors[BLUE] = 0x0000;
                        g_pui32Colors[GREEN] = 0x8000;
                        RGBColorSet(g_pui32Colors);
                        SysCtlDelay(1000000);
                        TrainingEndFlag = 1; //set training mode end flag
                        CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                           pfAccel[2]);

                        //get Euler angles, convert to degrees
                        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                             pfEulers + 2);
                        break;
                    }
                }
            }
            //////////////////////////////////////
            //!
            //!movement detected, record data
            //!
            while(!TrainingEndFlag && TrainingReadyFlag && (fJerk > GESTURE_EMIT_THRESHOLD_ACCEL
                    | (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL) | (JerkPitch > GESTURE_EMIT_THRESHOLD_EULERS_MOTION)
                    | (JerkRoll > GESTURE_EMIT_THRESHOLD_EULERS_MOTION) | (JerkYaw > GESTURE_EMIT_THRESHOLD_EULERS_MOTION)))
            {
                g_pui32Colors[RED] = 0x8000; //set LED to flash white
                g_pui32Colors[BLUE] = 0x8000;
                g_pui32Colors[GREEN] = 0x8000;
                RGBColorSet(g_pui32Colors);

                TrainingRunTimer++; //advance timer for movement

                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);
				        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                  pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
								
							  float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[5];

                //
                //feature extraction
                //

                //Euler's Angles
                //store min and max of roll
                if (minEulersTraining[0] > pfEulers[0])
                {
                    minEulersTraining[0] = pfEulers[0];
                }
                if (maxEulersTraining[0] < pfEulers[0])
                {
                    maxEulersTraining[0] = pfEulers[0];
                }
                //store min and max of pitch
                if (minEulersTraining[1] > pfEulers[1])
                {
                    minEulersTraining[1] = pfEulers[1];
                }
                if (maxEulersTraining[0] < pfEulers[0])
                {
                    maxEulersTraining[1] = pfEulers[1];
                }
                //store min and max of yaw
                if (minEulersTraining[2] > pfEulers[2])
                {
                    minEulersTraining[2] = pfEulers[2];
                }
                if (maxEulersTraining[2] < pfEulers[2])
                {
                    maxEulersTraining[2] = pfEulers[2];
                }


                //Acceleration
                //store min and max of x-axis acceleration
                if (minAccelTraining[0] > pfAccelNet[0])
                {
                    minAccelTraining[0] = pfAccelNet[0];
                }
                if (maxAccelTraining[0] < pfAccelNet[0])
                {
                    maxAccelTraining[0] = pfAccelNet[0];
                }
                //store min and max of y-axis acceleration
                if (minAccelTraining[1] > pfAccelNet[1])
                {
                    minAccelTraining[1] = pfAccelNet[1];
                }
                if (maxAccelTraining[1] < pfAccelNet[1])
                {
                    maxAccelTraining[1] = pfAccelNet[1];
                }
                //store min and max of z-axis acceleration
                if (minAccelTraining[2] > pfAccelNet[2])
                {
                    minAccelTraining[2] = pfAccelNet[2];
                }
                if (maxAccelTraining[2] < pfAccelNet[2])
                {
                    maxAccelTraining[2] = pfAccelNet[2];
                }

                //update net acceleration magnitude
                if (TrialAccelMagnitudeTraining<fAccelMagnitude)
                {
                    TrialAccelMagnitudeTraining = fAccelMagnitude;
                }

            }

            //////////////////////////////////////
            //!
            //!  end of training gestures
            //!
            if(TrainingEndFlag && TrainingReadyFlag)
            {
                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);

                //get Euler angles, convert to degrees
                CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                     pfEulers + 2);

                UARTprintf("\n \n Training Complete");
                TrainingReadyFlag = 0; //clear training ready flag
                RGBBlinkRateSet(99.99F);
                g_pui32Colors[RED] = 0x8000; //set LED to stay red
                g_pui32Colors[BLUE] = 0x0000;
                g_pui32Colors[GREEN] = 0x0000;
                SysCtlDelay(2000);
                RGBColorSet(g_pui32Colors);
                ExerciseMode = 1;
								
								minAccelTraining[0]=minAccelTraining[0]-GESTURE_EMIT_MOTION_DELTA;
								minAccelTraining[1]=minAccelTraining[1]-GESTURE_EMIT_MOTION_DELTA;
								minAccelTraining[2]=minAccelTraining[2]-GESTURE_EMIT_MOTION_DELTA;
								
								maxAccelTraining[0]=maxAccelTraining[0]+GESTURE_EMIT_MOTION_DELTA;
								maxAccelTraining[1]=maxAccelTraining[1]+GESTURE_EMIT_MOTION_DELTA;
								maxAccelTraining[2]=maxAccelTraining[2]+GESTURE_EMIT_MOTION_DELTA;
		
								minEulersTraining[0]=minEulersTraining[0]-GESTURE_EMIT_EULERS_DELTA;
								minEulersTraining[1]=minEulersTraining[1]-GESTURE_EMIT_EULERS_DELTA;
								minEulersTraining[2]=minEulersTraining[2]-GESTURE_EMIT_EULERS_DELTA;
								
								maxEulersTraining[0]=maxEulersTraining[0]+GESTURE_EMIT_EULERS_DELTA;
								maxEulersTraining[1]=maxEulersTraining[1]+GESTURE_EMIT_EULERS_DELTA;
								maxEulersTraining[2]=maxEulersTraining[2]+GESTURE_EMIT_EULERS_DELTA;
                continue; //exit all of these conditions, move to exercise mode
            }
        }

        //!
        //!Exercise mode compares the features of any IMU motion to the trained motion's features.
        //!White LED: means no movement detected, system not extracting features
        //!Green LED: movement detected, features extracted have not deviated significantly from ideal
        //!Red LED: movement detected, the motion's data does not match the training data.

        if(ExerciseMode)
        {

            //////////////////////////////////////
            //!
            //! no movement, do not extract feaures from sensor output.
            //! in this stage, a timer advances that will trigger a reset of learned exercise features
            while(((fJerk < GESTURE_EMIT_THRESHOLD_ACCEL) |
                    (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL)&& (JerkPitch < GESTURE_EMIT_THRESHOLD_EULERS_MOTION) &&
                    (JerkRoll < GESTURE_EMIT_THRESHOLD_EULERS_MOTION) && (JerkPitch < GESTURE_EMIT_THRESHOLD_EULERS_MOTION)))
            {

                ExerciseModeRestTimer++;

                // Get floating point version of the Accel Data in m/s^2.
                //
                MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
                                         pfAccel + 2);

                //
                // Get floating point version of angular velocities in rad/sec
                //
                MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
                                        pfGyro + 2);

                //
                // Get floating point version of magnetic fields strength in tesla
                //
                MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
                                           pfMag + 2);

                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);
				        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                  pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
								
							  float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[4];
								
                if(ExerciseModeRestTimer>1100) //reset all exercise features if rest timeout
                {

                    minAccel[0] = 0;
                    minAccel[1] = 0;
                    minAccel[2] = 0;
                    maxAccel[0] = 0;
                    maxAccel[1] = 0;
                    maxAccel[2] = 0;
                    minEulers[0] = 0;
                    minEulers[1] = 0;
                    minEulers[2] = 0;
                    maxEulers[0] = 0;
                    maxEulers[1] = 0;
                    maxEulers[2] = 0;
                    TrialExerciseTime = 0;
                    TrialAccelMagnitude = 0;
                    TrialSpeed = 0;

                    ExerciseModeRestTimer = 0;

                    CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                       pfAccel[2]);

                    CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                    MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);


                    //get Euler angles, convert to degrees
                    CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                         pfEulers + 2);


                }


            }
            //////////////////////////////////////
            //!
            //!movement detected, extract features, compare them to training data
            //!
            while((fJerk > GESTURE_EMIT_THRESHOLD_ACCEL
                    | (fJerk < -GESTURE_EMIT_THRESHOLD_ACCEL) | (JerkPitch > GESTURE_EMIT_THRESHOLD_EULERS_MOTION)
                    | (JerkRoll > GESTURE_EMIT_THRESHOLD_EULERS_MOTION) | (JerkYaw > GESTURE_EMIT_THRESHOLD_EULERS_MOTION)))
            {

                TrainingRunTimer++; //advance timer for movement
                ExerciseModeRestTimer = 0;
							
                CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],    //get local acceleration values
                                   pfAccel[2]);
				        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                  pfEulers + 2);

                CompDCMMatrixGet(&g_sCompDCMInst, ppfDCM);			//normalize to geospatial refernce
                MatrixVectorMul(pfAccelNet, ppfDCM, pfAccel);
								
							  float* pfSensorCalc = SensorUpdateMath(pfAccelNet, pfEulers);
								JerkRoll = pfSensorCalc[0];
								JerkPitch = pfSensorCalc[1];
								JerkYaw = pfSensorCalc[2];
								fJerk = pfSensorCalc[3];
								fAccelMagnitude = pfSensorCalc[4];

                ///////////////////
                //feature extraction
                ////////////////////

                //Euler's Angles
                //store min and max of roll
                if (minEulers[0] > pfEulers[0])
                {
                    minEulers[0] = pfEulers[0];
                }
                if (maxEulers[0] < pfEulers[0])
                {
                    maxEulers[0] = pfEulers[0];
                }
                //store min and max of pitch
                if (minEulers[1] > pfEulers[1])
                {
                    minEulers[1] = pfEulers[1];
                }
                if (maxEulers[1] < pfEulers[1])
                {
                    maxEulers[1] = pfEulers[1];
                }
                //store min and max of yaw
                if (minEulers[2] > pfEulers[2])
                {
                    minEulers[2] = pfEulers[2];
                }
                if (maxEulers[2] < pfEulers[2])
                {
                    maxEulers[2] = pfEulers[2];
                }


                //Acceleration
                //store min and max of x-axis acceleration
                if (minAccel[0] > pfAccelNet[0])
                {
                    minAccel[0] = pfAccelNet[0];
                }
                if (maxAccel[0] < pfAccelNet[0])
                {
                    maxAccel[0] = pfAccelNet[0];
                }
                //store min and max of y-axis acceleration
                if (minAccel[1] > pfAccelNet[1])
                {
                    minAccel[1] = pfAccelNet[1];
                }
                if (maxAccel[1] < pfAccelNet[1])
                {
                    maxAccel[1] = pfAccelNet[1];
                }
                //store min and max of z-axis acceleration
                if (minAccel[2] > pfAccelNet[2])
                {
                    minAccel[2] = pfAccelNet[2];
                }
                if (maxAccel[2] < pfAccelNet[2])
                {
                    maxAccel[2] = pfAccelNet[2];
                }

                //!
                //! Compare exercise features with training features
                //!

                //update net acceleration magnitude
                if (TrialAccelMagnitude<fAccelMagnitude)
                {
                    TrialAccelMagnitude = fAccelMagnitude;
                }

//minAccel[0]>=minAccelTraining[0] && minAccel[1]>=minAccelTraining[1]
//                        && minAccel[2]>=minAccelTraining[2] && maxAccel[0]<=maxAccelTraining[0] && maxAccel[1]<=maxAccelTraining[1]
//                        && maxAccel[2]<=maxAccelTraining[2] &&								
								
                if(minEulers[0]>=minEulersTraining[0] && minEulers[1]>=minEulersTraining[1]
                        && minEulers[2]>=minEulersTraining[2] && maxEulers[0]<=maxEulersTraining[0] && maxEulers[1]<=maxEulersTraining[1]
                        && maxEulers[2]<=maxEulersTraining[2])
                {
                    g_pui32Colors[RED] = 0x0000; //set LED to green to indicate good path
                    g_pui32Colors[BLUE] = 0x0000;
                    g_pui32Colors[GREEN] = 0x8000;
                    RGBColorSet(g_pui32Colors);
                }
//minAccel[0]<=minAccelTraining[0] || minAccel[1]<=minAccelTraining[1]
//                        || minAccel[2]<=minAccelTraining[2] || maxAccel[0]>=maxAccelTraining[0] || maxAccel[1]>=maxAccelTraining[1]
//                        || maxAccel[2]>=maxAccelTraining[2] ||
								
                else if(minEulers[0]<=minEulersTraining[0] || minEulers[1]<=minEulersTraining[1]
                        || minEulers[2]<=minEulersTraining[2] || maxEulers[0]>=maxEulersTraining[0] || maxEulers[1]>=maxEulersTraining[1]
                        || maxEulers[2]>=maxEulersTraining[2])
                {
                    g_pui32Colors[RED] = 0x8000; //set LED to red to indicate path deviation
                    g_pui32Colors[BLUE] = 0x0000;
                    g_pui32Colors[GREEN] = 0x0000;
                    RGBColorSet(g_pui32Colors);
                }
            }

        }

        //
        // Go to sleep mode while waiting for data ready.
        //

        while(!g_vui8I2CDoneFlag)
        {
            ROM_SysCtlSleep();
        }

        //
        // Clear the flag
        //
        g_vui8I2CDoneFlag = 0;

        //
        // Get floating point version of the Accel Data in m/s^2.
        //
        MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
                                 pfAccel + 2);

        //
        // Get floating point version of angular velocities in rad/sec
        //
        MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
                                pfGyro + 2);

        //
        // Get floating point version of magnetic fields strength in tesla
        //
        MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
                                   pfMag + 2);

        //
        // Check if this is our first data ever.
        //
        if(ui32CompDCMStarted == 0)
        {
            //
            // Set flag indicating that DCM is started.
            // Perform the seeding of the DCM with the first data set.
            //
            ui32CompDCMStarted = 1;
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
                                 pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
                              pfGyro[2]);
            CompDCMStart(&g_sCompDCMInst);
        }
        else
        {
            //
            // DCM Is already started.  Perform the incremental update.
            //
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
                                 pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
                              -pfGyro[2]);
            CompDCMUpdate(&g_sCompDCMInst);
        }

        //
        // Get Euler data. (Roll Pitch Yaw)
        //
        CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                             pfEulers + 2);
        //
        // Get Quaternions.
        //
        CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

        //
        // convert mag data to micro-tesla for better human interpretation.
        //


    }
}
