// Rushi Patel
// pater82
// 400316315

/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

// Initialized port M for active low push button (Stop)
void PortM_Init(void){
    //Use PortM pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                     // Activate clock for Port M
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};     // Allow time for clock to stabilize
    GPIO_PORTM_DIR_R |= 0x00    ;                                           // Making PM0 an input
  GPIO_PORTM_AFSEL_R &= ~0xFF;                                          // Disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;                                             // Enable digital I/O on PN0
  GPIO_PORTM_AMSEL_R &= ~0xFF;                                          // Disable analog functionality on PN0
    return;
}

void PortH_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; //allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111; // Make PM0:PM3 inputs, reading if the button is pressed or not
	GPIO_PORTH_DEN_R = 0b00001111; // Enable PM0:PM3
return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void spin(){
	int wait = 2;
	for(int i=0; i<64; i++){
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(wait);
	}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t id;
	uint8_t type;
	uint16_t both;

	//initialize the ports
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortJ_Init();
	PortM_Init();

/* Those basic I2C read functions can be used to check your own I2C functions 
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);
*/
	// Booting ToF chip
	// check if sensor is done before i2c starts to being acess
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

	// function will be called. This is to get the range
  status = VL53L1X_StartRanging(dev); 

	//status = VL53L1_RdByte(dev, 0x010F, &id); //for model ID (0xEA)
	//status = VL53L1_RdByte(dev, 0x0110, &type); //for module type (0xCC)
	//status = VL53L1_RdWord(dev, 0x010F, &both); //for both model ID and type
	//sprintf(printf_buffer,"%x, %x, %x\r\n",id, type, both);
	// keep track of how many scans have been done 
	int count = 0;
	// get 3 measurments
	while(count < 3)
	{
		// this is for the button press. Active LO is being used
		if(GPIO_PORTJ_DATA_R == 0b00000000)
		{
			count++;
			// get 8 measurments for 360 degrees we have 
			for(int i = 0; i < 8; i++) {
			
				//  used to detect stop button 
        if((GPIO_PORTM_DATA_R&0b00000001)==0){
          //  break out of code random place holder value
          count = 5;
          break;
        }else{
        }
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
				FlashLED1(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			
			//read the data values from ToF sensor
			//status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			//status = VL53L1X_GetSignalRate(dev, &SignalRate);
			//status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			
			spin();

			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			
			// print the resulted readings to UART
			sprintf(printf_buffer,"%u\n",Distance);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(50);
			}
		}
		else
		{
			SysTick_Wait10ms(50);
		}
		
	}
  
	VL53L1X_StopRanging(dev);
  while(1) {}

}

