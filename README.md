# Microprocessor-Lidar-System




<!-- GETTING STARTED -->
## Getting Started

Designed and constructed an embedded spatial measurement system using a time-of-flight sensor to gather information about the surrounding environment. The system includes a rotary mechanism that allows for a 360-degree measurement of distance within a single vertical geometric plane (e.g. y-z). The collected data is stored in onboard memory and later transmitted to a personal computer or web application for visualization and analysis.

### Description
 The microcontroller serves as a bridge between the time-of-flight sensor and the computer. The sensor is mounted on a stepper motor that rotates in both clockwise and counterclockwise directions to obtain spatial distance measurements in the y-z plane. It completes three full rotations, each representing a 360-degree surface that will later be visualized on a personal computer. The sensor uses a photon emitter to emit light and measures the time it takes for the light to bounce back after hitting a surface. The sensor includes all necessary analog-to-digital conversion such as transduction, signal conditioning and analog-to-digital conversion. The microcontroller communicates with the sensor using I2C protocol to get distance measurements and then communicates with the personal computer using UART. The data is then visualized using Python, specifically Pyserial to read data from UART and Open3D to create a 3D representation of the environment
This is an example of how to list things you need to use the software and how to install them.




### Built With



Python: OpenGL,Pyserial
C: Extracting Distance Measurements, I2C Communication, UART Communication

SimpleLink MSP432E401Y Microcontroller
Stepper Motor, Time of Flight Sensor

### Images

![alt text](https://github.com/rushi231/Microprocessor-Lidar-System/blob/main/Output.png)



<!-- USAGE EXAMPLES -->
Code

These are screenshots of some of the code used
	
	
	void PortM_Init(void){      //USED FOR OFF BUTTON(PM0)
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x00	;        								// making PM0 an input  
  	GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
        GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									
        GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}
	
	
	
	
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
	
	while (dataReady == 0){
		status = VL53L1X_CheckForDataReady(dev, &dataReady);
		FlashLED1(1);
		L53L1_WaitMs(dev, 5);
	}






