/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM� FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"

//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
  
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

BOOL CheckButtonPressed(void);

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else 
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
  
  //To fix this situation, we should always deliberately place a 
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie fast", since this is in a #pragma interrupt section 
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150(); 

  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
   //oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	// Soft Start the APP_VDD
   while(!AppPowerReady())
		;

    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
  //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  //This allows the device to power up at a lower initial operating frequency, which can be
  //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
  //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif

    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
    
    UserInit();

}//end InitializeSystem

static void InitializeADCON0(void)
{
    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ADCON0 = 0x11;                  // Default all pins to digital
    #endif
}

void StringCleaner(int num, char* s){
	if (num < 0){
		if(num > -1000){
			if(num > -100){
				if(num > -10){
					s[2] = ' ';
				}
				s[3] = ' ';
			}
			s[4] = ' ';
		}
		s[5] = '\0';		
	}
	else {
		if (num < 1000){
			if (num < 100){
				if (num < 10){
					s[1] = ' ';
				}
				s[2] = ' ';
			}
			s[3] = ' ';
			s[4] = '\0';
		}
	}
}

void readA2D(BYTE channel, char* s)
{
	int a2d;
	a2d = readTouch(channel);
	sprintf(s, "%d", a2d);			//convert a2d into decimal number and put it in "s"
	StringCleaner(a2d, s);
	return a2d;
}

int readTouch(BYTE channel){
	BYTE x;
	int a2d;
	x = 2;
	ADCON0 = channel;
	while (x){						//check the status of ADCON0 if get any input, else exit the loop
		x = x & ADCON0;
	}
	a2d = ADRESH;
	a2d = a2d << 8;
	a2d = a2d + ADRESL;	
	return a2d;
}

BOOL CheckButtonPressed()
{
    static char buttonPressed = FALSE;
    static unsigned long buttonPressCounter = 0;

    if(PORTBbits.RB0 == 0)
    {
        if(buttonPressCounter++ > 10)
        {
            buttonPressCounter = 0;
            buttonPressed = TRUE;
        }
    }
    else
    {
        if(buttonPressed == TRUE)
        {
            if(buttonPressCounter == 0)
            {
                buttonPressed = FALSE;
                return TRUE;
            }
            else
            {
                buttonPressCounter--;
            }
        }
    }
    return FALSE;
}

int readAcc(BYTE address){
	BYTE b;
	int result;
	b = BMA150_ReadByte(address);
	result = (int)b;
	result = result << 8;
	b = BMA150_ReadByte(address - 0x01);
	result += b;
	result = result >> 6;
	if (address == 0x07)
		{return result;}
	result = calcAcc(result);
	return result;
}

int calcAcc(int result){
	if(result > 511){
		result = result | 0xFC00;
		result = twosComp(result);
	}
	return result;
}

int twosComp(int value){
	value = ~value;
	value += 0x1;
	return -value;
}

void cleanBar (char* bar){
	int i;
	bar[0] = 0x23;
	for (i = 1; i < 20; i++){
		bar[i] = 0x24;
	}
	bar[i] = 0x25;
	bar[10] = 0x26;
}

//  ========================	Screens code		========================

void DrawScreen(int type)
{
	FillDisplay(0x00);
	char phrase[20];
	int i;
	//Type 0 - Main menu
	if(type == 0)
	{	
		sprintf(phrase,"%s","Hello");	
		oledPutString(phrase,1,10);
		return;	
	}
}

//	========================	Application Code	========================


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{
	static char potValues[5];
	static char left[2] = {' ', '\0'};
	static char right[2] = {' ', '\0'};
	static char scroll[2] = {' ', '\0'};
	static char arrows[7] = {'m', 'o', 'v', 'e', ':', ' ', '\0'};
	static char potBar[22] = {0x23, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x25, '\0'};
	static char xBar[22] = {0x23, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x26, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x25, '\0'};
	static char yBar[22] = {0x23, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x26, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x25, '\0'};
	static char yName[6] = {'y', ' ', 'b', 'a', 'r', '\0'};
	static char xName[6] = {'x', ' ', 'b', 'a', 'r', '\0'};
	static char potMeter[10] = {'p', 'o', 't', ' ', 'm', 'e', 't', 'e', 'r', '\0'};

	static char x[6];				//x,y,z init
	static char y[6];				//x,y,z init
	static char z[5];				//x,y,z init
	BYTE xG;						//x,y,z init
	BYTE yG;						//x,y,z init
	BYTE zG;						//x,y,z init
	signed int resultX, resultY, resultZ, maxResultX, maxResultY, minResultX, minResultY;		//x,y,z init

	int BP;						// button init
	BOOL flag;						// button init

	int L, R, U, D, scr;			// left right up down scroll

	int potValue;					//pot meter

	int barFill, forFill;			// bar content

	BYTE bmaTest;					//temprature init
	int bmaCelsius;					//temprature init
	static char bmaString[6];		//temprature init

	static char btn[2] = {0x7E, '\0'};				// button icon
	char words[2][10];

    InitializeADCON0();
	InitializeSystem();
	BP = 0;
	flag = TRUE;
	forFill = 0;
	minResultX = 0;
	maxResultX = 0;
	minResultY = 0;
	maxResultY = 0;
	//oledPutString (potMeter, 0, 6*6);
	//oledPutString (potBar, 1, 0);
	oledPutString (xBar, 3, 0);
	oledPutString (yBar, 5, 0);
	oledPutString (arrows, 7, 0);
	oledPutString (xName, 2, 8*6);
	oledPutString (yName, 4, 8*6);
	oledPutString (btn, 0, 18*6);

    while(1)							//Main is Usualy an Endless Loop
    {
		ADCON0 = 0x13;
		//checkPressed(flag);
		if(CheckButtonPressed()){
			if (flag){
				flag = FALSE;
				btn[0] = 0x7D;
				oledPutString(btn, 0, 18*6);
				BP = 1;
			}
			else if(!flag & !BP){
				flag = TRUE;
				btn[0] = 0x7E;
				oledPutString(btn, 0, 18*6);
			}
			BP = 0;
		}
		ADCON0 = 0x11;

		/*readA2D(0x13, potValues);				//read the potentiometer values
		potValue = readTouch(0x13);
		oledPutString(potValues, 0, 0*6);		//print "s" to the screen
		barFill = potValue / 50;
		while (forFill > 0){							// clean pot meter bar
			potBar[forFill--] = 0x24;
		}
		for (forFill = 0; barFill > 0; forFill++){		// filling pot meter bar
			potBar[forFill] = 0x26;
			barFill--;
		}
		oledPutString (potBar, 1, 0);					// print pot meter bar*/

		L = readTouch(0x0F);							//read if left is being touched
		if (L < 680){
			left[0] = 0x7b;
			oledPutString(left, 7, 9*6);				// print left arrow
		}
		else{
			left[0] = ' ';
			oledPutString(left, 7, 9*6);
		}
		R = readTouch(0x03);							//read if right is being touched
		if (R < 680){
			right[0] = 0x7c;
			oledPutString(right, 7, 11*6);				// print right arrow
		}
		else{
			right[0] = ' ';
			oledPutString(right, 7, 11*6);
		}
		U = readTouch(0x07);
		D = readTouch(0x0B);
		scr = U - D;
		if(scr > 0){									//scroll down
			while (D < 800){
				scroll[0] = 0x28;
				oledPutString(scroll, 7, 10*6);
				D = readTouch(0x0B);
			}
		}
		else if(scr < 0){								// scroll up
			while(U < 800){
				scroll[0] = 0x27;
				oledPutString(scroll, 7, 10*6);
				U = readTouch(0x07);
			}
		}
		else{											// clear the scroll
			scroll[0] = ' ';
			oledPutString(scroll, 7, 10*6);
		}

		bmaCelsius = (int)BMA150_ReadByte(0x08);			//read the temprature
		bmaCelsius -= 52;
		bmaCelsius *= 5;
		bmaCelsius /= 9;
		sprintf(&bmaString[2], "%d", bmaCelsius);
		bmaString[0] = 'T';
		bmaString[1] = ':';
		bmaString[4] = 0x60;
		bmaString[5] = '\0';
		oledPutString(bmaString, 7, 16*6);

		resultX = readAcc(0x03);						// read x
		if (resultX > maxResultX){								// read positive x bar
			sprintf(x, "%d", resultX);
			StringCleaner(resultX, x);
			oledPutString(x, 2, 18*6);
			maxResultX = resultX;
			barFill = maxResultX / 50;
			for (forFill = 11; barFill > 0; forFill++){			// fill positive x bar
				xBar[forFill] = 0x26;
				barFill--;
			}
			oledPutString (xBar, 3, 0);
		}
		else if (resultX < minResultX){							// read negative x bar
			sprintf(x, "%d", resultX);
			StringCleaner(resultX, x);
			oledPutString(x, 2, 0*6);
			minResultX = resultX;
			barFill = (-minResultX) / 50;
			for (forFill = 9; barFill > 0; forFill--){			// fill negative x bar
				xBar[forFill] = 0x26;
				barFill--;
			}
			oledPutString (xBar, 3, 0);
		}
		resultY = readAcc(0x05);						// read y
		if (resultY > maxResultY){								// read positive y bar
			sprintf(y, "%d", resultY);
			StringCleaner(resultY, y);
			oledPutString(y, 4, 18*6);
			maxResultY = resultY;
			barFill = maxResultY / 50;
			for (forFill = 11; barFill > 0; forFill++){			// fill positive y bar
				yBar[forFill] = 0x26;
				barFill--;
			}
			oledPutString (yBar, 5, 0);
		}
		else if (resultY < minResultY){							// read negative y bar
			sprintf(y, "%d", resultY);
			StringCleaner(resultY, y);
			oledPutString(y, 4, 0*6);
			minResultY = resultY;
			barFill = (-minResultY) / 50;
			for (forFill = 9; barFill > 0; forFill--){			// fill negative y bar
				yBar[forFill] = 0x26;
				barFill--;
			}
			oledPutString (yBar, 5, 0);		
		}
		resultZ = readAcc(0x07);						//read z
		if (resultZ == 960){
			while(resultZ != 64){						// if fliped clean x and y bars
				resultZ = readAcc(0x07);
				resultX = 0;
				resultY = 0;
				minResultX = 0;
				maxResultX = 0;
				minResultY = 0;
				maxResultY = 0;
				sprintf(x, "%d", resultX);
				sprintf(y, "%d", resultY);
				StringCleaner(resultX, x);
				StringCleaner(resultY, y);
				oledPutString(x, 2, 0*6);
				oledPutString(x, 2, 18*6);
				oledPutString(y, 4, 18*6);
				oledPutString(y, 4, 0*6);
				cleanBar(&xBar);
				cleanBar(&yBar);
				oledPutString(xBar, 3, 0*6);
				oledPutString(yBar, 5, 0*6);
				}
		}
		DrawScreen(0);
    }
}//end main


/** EOF main.c *************************************************/
//#endif
