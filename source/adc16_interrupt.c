#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_tpm.h"
#include "pid.h"
#include <stdio.h>

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 12U /*PTE20, ADC0_SE0 */

#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

/* The Flextimer instance/channel used for board */
#define BOARD_TPM_BASEADDR TPM1
#define BOARD_FIRST_TPM_CHANNEL 0U
#define BOARD_SECOND_TPM_CHANNEL 1U
#define ServoX_Dir   1   //0 is normal	1 is opposite
#define ServoY_Dir   1

/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_PllFllSelClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Init_Read_X(void);
void Init_Read_Y(void);
void delay(uint32_t Delay_Count);
void Long_delay(uint32_t Delay_Count);
void Get_XY(void);
void Set_PWM_Duty(uint16_t X_Pos, uint16_t RevX, uint16_t Y_Pos, uint16_t RevY); //Revn = 0 is normal =1 is rev

/*******************************************************************************
 * Variables and Structure for ADC
 ******************************************************************************/
adc16_channel_config_t adc16ChannelConfigStruct;
signed int Xval;
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
volatile uint32_t g_Adc16InterruptCounter;
volatile uint32_t g_ADC_Channel_No_Var = 13U;
volatile uint32_t g_ADC_Info;
//Variables used in the PID controllers
volatile float g_XDInput, g_YDInput, g_XITerm, g_YITerm;
volatile uint32_t g_SysTick [2], g_XPresentTime, g_YPresentTime,  g_XDeltaTime, g_YDeltaTime;
volatile int32_t g_XPrevious_Time, g_Xerror, g_Xsetpoint, g_XlastInput;
volatile int32_t g_YPrevious_Time, g_Yerror, g_Ysetpoint, g_YlastInput;
volatile uint16_t g_XoutMax, g_XoutMin, g_XoutLevel, g_XOutput, kk;
volatile uint16_t g_YoutMax, g_YoutMin, g_YoutLevel, g_YOutput;
volatile float g_Xki, g_Xkd, g_Xkp, Xtest;
volatile float g_Yki, g_Ykd, g_Ykp, Ytest;

volatile uint8_t getCharValue = 0U;
volatile uint8_t updatedDutycycle1 = 85U, updatedDutycycle2 = 85U;
volatile float  g_A_Cal, g_B_Cal, g_C_Cal, g_D_Cal, g_E_Cal, g_F_Cal;
volatile uint32_t g_X_Raw, g_Y_Raw, g_X_Cal, g_Y_Cal;
volatile float g_XOut, g_YOut;
volatile int32_t g_Xoff, g_Yoff;
volatile uint32_t ValSave[100];
uint32_t A[10];


/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)
{


g_SysTick [0] += 1u;
g_SysTick [1] += 1u;
}


void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    g_Adc16InterruptCounter++;
}

/*!
 * @brief Main function ***************************************
 */
int main(void)
{
	uint8_t Ti, Tk;
    g_XoutMin = 110;
 	g_XoutMax = 163;
 	g_XoutLevel = 133;

	g_YoutMin = 122;
	g_YoutMax = 180;
	g_YoutLevel = 150;

	g_Xki = 0.00015;
	g_Xkd = 0.015;
	g_Xkp = 0.04;

	g_Yki = 0.00015;
	g_Ykd = 0.015;
	g_Ykp = 0.04;

	g_Xoff = 95;
	g_Yoff = 120;




    // Configure sys tick for 0.1 mSec per tick
	SysTick_Config(CLOCK_GetFreq(kCLOCK_CoreSysClk)/(4370UL));
    g_SysTick [0] = 0;
    g_SysTick [1] = 0;
 	g_X_Raw=0; g_Y_Raw=0;
	g_A_Cal=  0.5895 ; g_B_Cal= 0.0054; g_C_Cal= -138.84;
	g_D_Cal= -0.0004 ; g_E_Cal= 0.6222; g_F_Cal= -222.49;
	g_X_Cal=0; g_Y_Cal=0;
	uint32_t Option_value, i;
    adc16_config_t adc16ConfigStruct;

    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL;
    tpmParam[0].level = kTPM_HighTrue;
    tpmParam[0].dutyCyclePercent = updatedDutycycle1;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL;
    tpmParam[1].level = kTPM_HighTrue;
    tpmParam[1].dutyCyclePercent = updatedDutycycle2;


    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    EnableIRQ(DEMO_ADC16_IRQn);

    PRINTF("\r\nTilt Table Control Code: \r\n");

    /* Select the clock source for the TPM counter as kCLOCK_PllFllSelClk */
    CLOCK_SetTpmClock(1U);

    // Set up the PID Structures, Pointers and Controller Constants

    PID* pXPID;
    PID* pYPID;
    pXPID = InitPID();
    pYPID = InitPID();

    SetPIDGain(pXPID, g_Xkp, g_Xki, g_Xkd);
    SetPIDLimits(pXPID, g_XoutMin - g_XoutLevel, g_XoutMax - g_XoutLevel);  //Works out to roughly -/+ 30
    SetPIDSetpoint(pXPID, 1000.0);

    SetPIDGain(pYPID, g_Ykp, g_Yki, g_Ykd);
    SetPIDLimits(pYPID, g_YoutMin - g_YoutLevel, g_YoutMax - g_YoutLevel);  //Works out to roughly -/+ 30
    SetPIDSetpoint(pYPID, 1000.0);


     TPM_GetDefaultConfig(&tpmInfo);
     /* Initialize TPM module */
     TPM_Init(BOARD_TPM_BASEADDR, &tpmInfo);

     TPM_SetupPwm(BOARD_TPM_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 100U, TPM_SOURCE_CLOCK);
     TPM_StartTimer(BOARD_TPM_BASEADDR, kTPM_SystemClock);
     g_SysTick [1] = 0u;  //Zero g_SysTick[1] so you can somewhat sync with the servo, every 100 ticks =10ms


    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif    
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    ADC16_SetHardwareAverage(DEMO_ADC16_BASE, kADC16_HardwareAverageCount32);
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    g_Adc16InterruptCounter = 0U;
    //Center Tables
    Set_PWM_Duty(g_XoutLevel, ServoX_Dir, g_YoutLevel, ServoY_Dir);  //X servo should be centered and Y low

    //Set uo the pin E5 to toggle on the systic


    while(1)
    {
    do{

    PRINTF("\r\n1= Read and Display X and Y raw values, Will stop after 20 samples");
    PRINTF("\r\n2= Test Servo Direction:  X servo Low then Y Low  then X Hi then Y Hi then both level");
    PRINTF("\r\n3= Display calibrated X & Y values");
    PRINTF("\r\n4= Run your Code");
    PRINTF("\r\n5= Center X and Y Servo");
    PRINTF("\r\n6= Print and Move Servo Direction X and Y, Min X,Y  and Max X,Y Servo and Level X and Y");
    PRINTF("\r\n7= Set test error and watch response.  No D or I");
    PRINTF("\r\n8= Print out ValSave 0-9");
    PRINTF("\r\n9= Move Servo smoothly from down to up and then Mid Servo Pos\n\r");
    Option_value = GETCHAR()- 0x30U;
    PRINTF("\r\nYou Entered: %d\r\n", Option_value);
    } while (Option_value > 10U);

    switch (Option_value)
		{
    case 1:
    	    PRINTF("XRaw,YRaw\r\n" );
    		for (i=0; i<20; i++)
    		{
    			Get_XY();
    			PRINTF("%4.d %4.d\r\n", g_X_Raw, g_Y_Raw  );
    		}
    		break;
    case 2:  /*  Tests the servo motions if wrong Change ServoX_Dir and/or ServoY_Dir  */

    		Set_PWM_Duty(100, 0,100, 0);
    		Long_delay(70LU);

    		Set_PWM_Duty(200, 0, 200, 0);
    		Long_delay(70LU);

    		break;
    case 3:

		    for (i=0; i<20; i++)
		       {
			   Get_XY();
	    	   g_X_Cal = g_A_Cal * g_X_Raw + g_B_Cal * g_Y_Raw + g_C_Cal ;
	    	   g_Y_Cal = g_D_Cal * g_X_Raw + g_E_Cal * g_Y_Raw + g_F_Cal ;

			   PRINTF("%4.d %4.d %4.d %4.d %6.d\r\n", g_X_Raw, g_Y_Raw, g_X_Cal, g_Y_Cal, g_SysTick[0]);
		       }
    		break;

    case 4:  /* Runs Code */
    		while(1){
    			while (!(TPM_GetStatusFlags(BOARD_TPM_BASEADDR ) & 0x100U ))
    				continue;

    			GPIOE->PTOR = (1<<5);

				Get_XY();

				if ((g_X_Raw<350)||(g_Y_Raw <100)){
				ResetPID(pXPID);
				ResetPID(pYPID);
				Set_PWM_Duty(g_XoutLevel, ServoX_Dir, g_YoutLevel, ServoY_Dir);  //X servo should be centered and Y low
				continue;
				}

// Calibrate the raw X and Y to Cal X and Y values
		    g_X_Cal = g_A_Cal * g_X_Raw + g_B_Cal * g_Y_Raw + g_C_Cal;
		    g_Y_Cal = g_D_Cal * g_X_Raw + g_E_Cal * g_Y_Raw + g_F_Cal;

		    g_XOut = GetPIDOutput(pXPID, g_X_Cal, 0.01);
		    g_YOut = GetPIDOutput(pYPID, g_Y_Cal, 0.01);

		    Set_PWM_Duty(g_XoutLevel + g_XOut , ServoX_Dir,g_YoutLevel+ g_YOut, ServoY_Dir);

			// Print statements for debugging
		    // PRINTF("Pos %4ul %4ul Servos %c%d %c%d\r\n",g_X_Cal, g_Y_Cal,
		    // g_XOut < 0 ? '-' : ' ', (uint32_t) g_XOut,
		    // g_YOut < 0 ? '-' : ' ', (uint32_t) g_YOut );

    		}
    		break;

    case 5:  //Center X Servo
			Set_PWM_Duty(g_XoutLevel, ServoX_Dir, g_YoutLevel, ServoY_Dir);  //X servo should be centered and Y low
			break;

    case 6:  //Center Y Servo
			Set_PWM_Duty(g_XoutMin, ServoX_Dir, g_YoutMin, ServoY_Dir);
			Long_delay(50LU);
			Set_PWM_Duty(g_XoutMax, ServoX_Dir, g_YoutMax, ServoY_Dir);
			Long_delay(50LU);
    	    PRINTF("%4.d %4.d\r\n", g_XoutLevel, g_YoutLevel);
			Set_PWM_Duty(g_XoutLevel, ServoX_Dir, g_YoutLevel, ServoY_Dir);
			Long_delay(50LU);
			break;

    case 7:  /* Used for testing the code  by setting an arbitrary error */
            for(kk=70; kk<180; kk++){

            	TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL, kTPM_EdgeAlignedPwm, kk);
                TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL, kTPM_EdgeAlignedPwm,kk);
                PRINTF("%4.d \r\n",kk  );
                for (i=0; i<30000; i++)continue;
                for (i=0; i<30000; i++)continue;
                for (i=0; i<30000; i++)continue;
                for (i=0; i<30000; i++)continue;
            }
        		break;
    case 8:  /* Print out saved data for debug and tuning controller*/

            	break;
    case 9:	 /* This Checks if servo moves smoothly or not and sets to servo mid position */
    	        for(Ti=100;Ti<201;Ti++){
    	    		TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL, kTPM_EdgeAlignedPwm,Ti);
    	    	    TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL, kTPM_EdgeAlignedPwm,Ti);
    	    	    //GETCHAR();
    	    	    Long_delay(10U);
    	        }
    	        PRINTF("%4.d %4.d\r\n", g_XoutLevel, g_YoutLevel);
    	        Set_PWM_Duty(g_XoutLevel, ServoX_Dir, g_YoutLevel, ServoY_Dir);

    	        break;
    default:
    		PRINTF("\r\nYou Entered bad value=: %d\r\n", Option_value);
    		break;
		}



    }  //End of principle inf while loop

  }


/*******************************************************************************
 * User Function  Set_PWM_Duty
 ******************************************************************************/

void Set_PWM_Duty(uint16_t X_Pos, uint16_t RevX, uint16_t Y_Pos, uint16_t RevY)
{


	if(X_Pos > 200U) X_Pos = 200U;   //Upper limit
	if(X_Pos < 100U)  X_Pos = 100U;    //Lower limit
	if(Y_Pos > 200U) Y_Pos = 200U;   //Upper limit
	if(Y_Pos < 100U)  Y_Pos = 100U;    //Lower limit

	if(RevX) X_Pos = 300 - X_Pos ;
	if(RevY) Y_Pos = 300 - Y_Pos;

	TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL, kTPM_EdgeAlignedPwm,Y_Pos);
    TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL, kTPM_EdgeAlignedPwm,X_Pos);


    //base->CONTROLS[chnlNumber].CnV = cnv;
}

/*******************************************************************************
 * User Function  Get_XY
 * volatile uint32_t g_X_Raw, g_Y_Raw, g_A_Cal, g_B_Cal, g_C_Cal, g_D_Cal, g_E_Cal, g_F_Cal, g_X_Cal, g_Y_Cal;
 *
 *        0,2k__H__2k,2k
 *         |          |
 *         |          |    H = connector
 *         |0,0___2k,0|
 *
 *
 ******************************************************************************/

  void Get_XY(void) //Returns the RAW values as global floats, need to be calibrated
  {

	  /* Get X position */
	            g_Adc16ConversionDoneFlag = false;
	            Init_Read_X ();
	            adc16ChannelConfigStruct.channelNumber = 13U ;
	            delay(100U);  //to let the touch change over settle and charge sample cap
	            ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	            while (!g_Adc16ConversionDoneFlag)
	            {
	            }

	            g_X_Raw = g_Adc16ConversionValue;

	  /* Get Y  position */
	            g_Adc16ConversionDoneFlag = false;
	            Init_Read_Y ();
	            adc16ChannelConfigStruct.channelNumber = 12U ;
	            delay(100U);  //to let the touch change over settle and charge sample cap
	            ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	            while (!g_Adc16ConversionDoneFlag)
	            {
	            }

	            g_Y_Raw = g_Adc16ConversionValue;

  }

  /*******************************************************************************
   * User Function  delay
   ******************************************************************************/
  void delay(uint32_t Delay_Count)
  {
      volatile uint32_t i = 0;
      for (i = 0; i < Delay_Count; ++i)
      {
        	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
      }
  }


  /*******************************************************************************
   * User Function  Long_delay
   ******************************************************************************/
  void Long_delay(uint32_t Delay_Count)
  {
      volatile uint32_t i = 0, k = 0;
      for (i = 0; i < Delay_Count; ++i)
      {
          for(k=0; k<60000U; k++)
          {
        	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
          	  __asm("NOP"); /* delay */
          }
      }
  }

  /*******************************************************************************
   * User Function  Init_Read_X
   ******************************************************************************/

  void Init_Read_X (void)
  {
	     gpio_pin_config_t Back_config = {
	   	        .pinDirection = kGPIO_DigitalOutput,
	   	        .outputLogic = 0U
	     };
	     /* Initialize GPIO functionality on pin PTB10 (pin 49)  */
	     GPIO_PinInit(BOARD_INITPINS_Back_GPIO, BOARD_INITPINS_Back_PIN, &Back_config);

	     gpio_pin_config_t Front_config = {
	   	        .pinDirection = kGPIO_DigitalOutput,
	   	        .outputLogic = 0U
	     };

	     /* Initialize GPIO functionality on pin PTB11 (pin 50)  */
	   	 GPIO_PinInit(BOARD_INITPINS_Front_GPIO, BOARD_INITPINS_Front_PIN, &Front_config);

          // The ADC_Channel No Should be = 13U;
  	    gpio_pin_config_t Left_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 0U
  	    };
  	    /* Initialize GPIO functionality on pin PTB8 (pin 47)  */
  	    GPIO_PinInit(BOARD_INITPINS_Left_GPIO, BOARD_INITPINS_Left_PIN, &Left_config);

  	    gpio_pin_config_t Right_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 1U
  	    };
  	    /* Initialize GPIO functionality on pin PTB9 (pin 48)  */
  	    GPIO_PinInit(BOARD_INITPINS_Right_GPIO, BOARD_INITPINS_Right_PIN, &Right_config);


  	    Back_config.pinDirection = kGPIO_DigitalInput;
		Back_config.outputLogic = 0U;

  	    /* Initialize GPIO functionality on pin PTB10 (pin 49)  */
  	    GPIO_PinInit(BOARD_INITPINS_Back_GPIO, BOARD_INITPINS_Back_PIN, &Back_config);

  	    Front_config.pinDirection = kGPIO_DigitalInput;
		Front_config.outputLogic = 0U;

  	    /* Initialize GPIO functionality on pin PTB11 (pin 50)  */
  	    GPIO_PinInit(BOARD_INITPINS_Front_GPIO, BOARD_INITPINS_Front_PIN, &Front_config);

  	    //PRINTF("Init X \r\n" );

  }

  /*******************************************************************************
   * User Function  Init_Read_Y
   ******************************************************************************/
  void Init_Read_Y(void)
  {



	  // The ADC_Channel No Should be = 12U;
  	    gpio_pin_config_t Left_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 0U
  	    };
  	    /* Initialize GPIO functionality on pin PTB8 (pin 47)  */
  	    GPIO_PinInit(BOARD_INITPINS_Left_GPIO, BOARD_INITPINS_Left_PIN, &Left_config);

  	    gpio_pin_config_t Right_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 0U
  	    };
  	    /* Initialize GPIO functionality on pin PTB9 (pin 48)  */
  	    GPIO_PinInit(BOARD_INITPINS_Right_GPIO, BOARD_INITPINS_Right_PIN, &Right_config);

  	    gpio_pin_config_t Back_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 1U
  	    };
  	    /* Initialize GPIO functionality on pin PTB10 (pin 49)  */
  	    GPIO_PinInit(BOARD_INITPINS_Back_GPIO, BOARD_INITPINS_Back_PIN, &Back_config);

  	    gpio_pin_config_t Front_config = {
  	        .pinDirection = kGPIO_DigitalOutput,
  	        .outputLogic = 0U
  	    };
  	    /* Initialize GPIO functionality on pin PTB11 (pin 50)  */
  	    GPIO_PinInit(BOARD_INITPINS_Front_GPIO, BOARD_INITPINS_Front_PIN, &Front_config);
  	    // The ADC_Channel No Should be = 12U;
		 Left_config.pinDirection = kGPIO_DigitalInput;
		 Left_config.outputLogic = 0U;
		/* Initialize GPIO functional ity on pin PTB8 (pin 47)  */
		GPIO_PinInit(BOARD_INITPINS_Left_GPIO, BOARD_INITPINS_Left_PIN, &Left_config);

		Right_config.pinDirection = kGPIO_DigitalInput;
		Right_config.outputLogic = 0U;
		/* Initialize GPIO functionality on pin PTB9 (pin 48)  */
		GPIO_PinInit(BOARD_INITPINS_Right_GPIO, BOARD_INITPINS_Right_PIN, &Right_config);

  	    //PRINTF("Init Y \r\n" );

  }
