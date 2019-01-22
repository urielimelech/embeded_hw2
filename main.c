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

//  ========================	Screens code		========================

/*void DrawScreen(int type)
{
	int i;
	char phrase[20];
	FillDisplay(0x00);
	//Type 0 - Main menu
	if(type == 0)
	{	
		phrase="Hello\0";
		oledPutString(phrase,1,10);
		return;	
	}
}*/

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

void checkedLine(char* unCheck, char* Check){
  unCheck[0] = ' ';
  Check[0] = '>';
}

BOOL flag = TRUE, BP = TRUE;
int pressed(int selection){
  if (flag){
    flag = FALSE;
    BP = TRUE;
  }
  else if(!flag & !BP){
    flag = TRUE;
  }
  BP = FALSE;
  return selection;
}

static char MainHead[10] = {'M', 'a', 'i', 'n', ' ', 'M', 'e', 'n', 'u', '\0'};
static char instruction_Touch[14] = {'U', 's', 'e', ' ', 'T', 'o', 'u', 'c', 'h', ' ', 'P', 'a', 'd', '\0'};
static char instruction_Tilt[9]= {'U', 's', 'e', ' ', 'T', 'i', 'l', 't', '\0'};
static char instruction_Pot[8]= {'U', 's', 'e', ' ', 'P', 'o', 't', '\0'};
static char subHead[9] = {'S', 'u', 'b', ' ', 'M', 'e', 'n', 'u', '\0'};
static char instruction_left_right[17] = {'U', 's', 'e', ' ', 'L', 'e', 'f', 't', ' ', '&', ' ', 'R', 'i', 'g', 'h', 't', '\0'};
static char st1[7] = {'>', 'F', 'i', 'r', 's', 't', '\0'};
static char nd2[8] = {' ', 'S', 'e', 'c', 'o', 'n', 'd', '\0'};
static char rd3[7] = {' ', 'T', 'h', 'i', 'r', 'd', '\0'};
static char th4[6] = {' ', 'F', 'o', 'u', 'r', '\0'};
static char long_ex[19] = {' ', 'E', 'x', 'e', 'c', 'u', 't', 'e', ' ', 'A', 'c', 't', 'i', 'o', 'n', ' ', '0', '1', '\0'};
static char ex[5] = {'E', 'x', 'e', 'c', '\0'};
static char exNum[2] = {'1','\0'};
static char instruction_shake[18] = {'S','h','a','k','e',' ','M','e', ' ', 'T', 'o' ,' ','R','e','s','e','t','\0'};
static char shake[10] = {'S','h','a','k','e',' ','N','o','w','\0'};
static char shakeHard[13] = {'S','h','a','k','e',' ','H','a','r','d','e','r', '\0'};
static char sub_sub_menu[13] = {'S','u','b',' ','S', 'u', 'b', ' ', 'M', 'e', 'n', 'u', '\0'};
static char instruction_sub_sub[22] = {'J','u','s','t',' ','P','r','e','s','s',' ','T','h','e',' ','B','u','t','t','o','n','\0'};

void mainMenu(void){                      //touch pad menu
  oledPutString(MainHead, 0, 4*6);
  oledPutString(instruction_Touch, 1, 3*6);
  oledPutString(st1, 3, 0*6);
  oledPutString(nd2, 3, 10*6);
  oledPutString(rd3, 5, 0*6);
  oledPutString(th4, 5, 10*6);
}

void tilt_Menu(void){                    //pot menu
  oledPutString(subHead, 0,7*6);
  oledPutString(instruction_Tilt, 1, 5*6);
  oledPutString(st1, 2,0*6);
  oledPutString(nd2, 3,0*6);
  oledPutString(rd3, 4,0*6);
  oledPutString(th4, 5,0*6);
}

void pot_Menu(void){                   //tilt menu
  static int i = 1, Offset = 1;
  char c = '9';
  oledPutString(subHead, 0,7*6);
  oledPutString(instruction_Pot, 1, 5*6);
  for (i = 1; i < 7; i++){
    oledPutString(long_ex, Offset+i, 0*6);
    if (long_ex[17] == c){
      long_ex[16] = '1';
      long_ex[17] = '0';
      continue;
    }
    long_ex[17]++;
  }
  return;
}
  
void left_right_menu(void){                //sub tilt menu
  oledPutString(subHead, 0, 7*6); 
  oledPutString(instruction_left_right, 1, 2*6);
  oledPutString(ex, 3, 0*6);
  oledPutString(ex, 3, 5*6);
  oledPutString(ex, 3, 10*6);
  oledPutString(ex, 3, 15*6);
  oledPutString(exNum, 5, 2*6);
  exNum[0]++;
  oledPutString(exNum, 5, 7*6);
  exNum[0]++;
  oledPutString(exNum, 5, 12*6);
  exNum[0]++;
  oledPutString(exNum, 5, 17*6);
  exNum[0] = '1';
}

void shake_menu(void){
  oledPutString(subHead, 0, 7*6);
  oledPutString(instruction_shake, 1, 2*6);
  oledPutString(shake, 4, 6*6);
}

void subsub_menu(void){
  oledPutString(sub_sub_menu, 0, 4*6);
  oledPutString(instruction_sub_sub,1,0*6);
}

void menuPrinter(int menuNum){
  FillDisplay(0x00);
  switch (menuNum){
    case 0:
      mainMenu();
      break;
    case 1:
      tilt_Menu();
      break;
    case 2:
      pot_Menu();
      break;
    case 3:
      left_right_menu();
      break;
    case 4:
      shake_menu();
      break;
    case 5:
      subsub_menu();
      break;
  }
}


void main(void)
{
  int selection_flag = 0;
  int resultX, resultY, resultZ;
	char x[5], y[5], z[5];
  int selectMenu = 0, selection = 0, line = 0, potLastState = 0, left_right_LastState = 0, tiltCounter = 0, tiltLastState = 0;
  int functionallity = 0;
	int potValue, L, R, U, D;

 	InitializeADCON0();
	InitializeSystem();
  mTouchInit();
  mTouchCalibrate();
  mainMenu();
  
 	while(1){							//Main is Usualy an Endless Loop
    L = mTouchReadButton(3);							//read if left is being touched
    R = mTouchReadButton(0);							//read if right is being touched
    U = mTouchReadButton(1);              //read if up is being touched
	  D = mTouchReadButton(2);              //read if down is being touched
    ADCON0 = 0x11;
    if (CheckButtonPressed()){
      selectMenu = pressed(selection);
    }
    ADCON0 = 0x13;
    switch (selectMenu){
      case 0:                                  //main menu
        if (selection_flag != 0){
          menuPrinter(selectMenu);
          functionallity = 0;
        }
        selection_flag = 0;
        break;
      case 1:                                   //tilt menu
        if (selection_flag != 1){
          menuPrinter(selectMenu);
          functionallity = 1;
        }
        selection_flag = 1;
        break;
      case 2:                                   //pot menu
        if (selection_flag != 2){
          menuPrinter(selectMenu);
          functionallity = 2;
        }
        selection_flag = 2;
        break;
      case 3:                                   //left right menu
        if (selection_flag != 3){
          menuPrinter(selectMenu);
          functionallity = 3;
        }
        selection_flag = 3;
        break;
      case 4:
        if (selection_flag != 4){                 //forth_Menu();
          menuPrinter(selectMenu);
          functionallity = 4;
        }
        selection_flag = 4;
        break;
      case 5:
        if (selection_flag != 5){                 // press to reset
          menuPrinter(selectMenu);
          functionallity = 5;
        }
        selection_flag = 5;
        break;
    }
    if (functionallity == 0){                   // touchpad functionallity
        switch (selection){                     // navigate in the menu
          case 0:
            if (L < 600){
              selection = 1;  
              break;
            }    
            else if (R < 600){
              checkedLine(st1, nd2);
              menuPrinter(selectMenu);
              selection = 2;
              break;
            }
            else if (D < 600){
              checkedLine(st1, rd3);
              menuPrinter(selectMenu);
              selection = 3;
              break;
            }
            else if (U < 600){
              selection = 1;
              break;
            }
          case 1:
            if (L < 600){
              selection = 1;
              break;
            }
            else if (R < 600){
              checkedLine(st1, nd2);
              menuPrinter(selectMenu);
              selection = 2;
              break;
            }
            else if (D < 600){
              checkedLine(st1, rd3);
              menuPrinter(selectMenu);
              selection = 3;
              break;
            }
            else if (U < 600){
              selection = 1;
              break;
            }
          case 2:
              if (L < 600){
                checkedLine(nd2, st1);
                menuPrinter(selectMenu);
                selection = 1;
                break;
              }
              else if (R < 600){
                selection = 2;
                break;
              }
              else if (U < 600){
                selection = 2;
                break;
              }
              else if (D < 600){
                checkedLine(nd2, th4);
                menuPrinter(selectMenu);
                selection = 4;
                break;
              }
          case 3:
              if (L < 600){
                selection = 3;
                break;
              }
              else if (R < 600){
                checkedLine(rd3, th4);
                menuPrinter(selectMenu);
                selection = 4;
                break;
              }
              else if (U < 600){
                checkedLine(rd3, st1);
                menuPrinter(selectMenu);
                selection = 1;
                break;
              }
              else if (D < 600){
                selection = 3;
                break;
              }
          case 4:
              if (L < 600){
                checkedLine(th4, rd3);
                menuPrinter(selectMenu);
                selection = 3;
                break;
              }
              else if (R < 600){
                selection = 4;
                break;
              }
              else if (U < 600){
                checkedLine(th4, nd2);
                menuPrinter(selectMenu);
                selection = 2;
                break;
              }
              else if (D < 600){
                selection = 4;
                break;
              }
        }
    }
    else if (functionallity == 1){              // tilt functionallity
      resultX = readAcc(0x03);
      if (resultX > 10){
        while (resultX > 2)
          resultX = readAcc(0x03);
        tiltCounter++;
      }
      else if (resultX < -10){
        while (resultX < -2)
          resultX = readAcc(0x03);
        tiltCounter--;
      }
      if (tiltCounter < 0)
        tiltCounter = 0;
      if (tiltCounter > 4)
        tiltCounter = 4;
      if (tiltLastState != tiltCounter){
        switch(tiltCounter){
          case 0:
            st1[0] = '>';
            nd2[0] = ' ';
            rd3[0] = ' ';
            th4[0] = ' ';
            menuPrinter(selectMenu);
            selection = 0;
            break;
          case 1:
            st1[0] = ' ';
            nd2[0] = '>';
            rd3[0] = ' ';
            th4[0] = ' ';
            menuPrinter(selectMenu);
            selection = 1;
            break;
          case 2:
            st1[0] = ' ';
            nd2[0] = ' ';
            rd3[0] = '>';
            th4[0] = ' ';
            menuPrinter(selectMenu);
            selection = 2;
            break;
          case 3:
            st1[0] = ' ';
            nd2[0] = ' ';
            rd3[0] = ' ';
            th4[0] = '>';
            menuPrinter(selectMenu);
            selection = 3;
            break;
        }
      }
      tiltLastState = tiltCounter;
    }
    else if (functionallity == 2){              // pot functionallity
      potValue = readTouch(0x13);
      selection = potValue/60;
      if (potLastState != selection){
        if (selection < 6){
          long_ex[16] = '0';
          long_ex[17] = '1';
          menuPrinter(selectMenu);
        }
        else if ((6 <= selection) && (selection < 12)){
          long_ex[16] = '0';
          long_ex[17] = '7';
          menuPrinter(selectMenu);
        }
        else{
          long_ex[16] = '1';
          long_ex[17] = '3';
          menuPrinter(selectMenu);
        }
        potLastState = selection;
      }
      switch (potLastState){
        case 0:
          oledWriteChar1x('>', 0xB2, 0*6);
          oledWriteChar1x(' ', 0xB3, 0*6);
          selection = 0;
          break;
        case 1:
          oledWriteChar1x('>', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB2, 0*6);
          oledWriteChar1x(' ', 0xB4, 0*6);
          selection = 1;
          break;
        case 2:
          oledWriteChar1x('>', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          selection = 2;
          break;
        case 3:
          oledWriteChar1x('>', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          selection = 3;
          break;
        case 4:
          oledWriteChar1x('>', 0xB6, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB7, 0*6);
          selection = 4;
          break;
        case 5:
          oledWriteChar1x('>', 0xB7, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          selection = 5;
          break;
        case 6:
          oledWriteChar1x('>', 0xB2, 0*6);
          oledWriteChar1x(' ', 0xB7, 0*6);
          break;
        case 7:
          oledWriteChar1x('>', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB2, 0*6);
          break;
        case 8:
          oledWriteChar1x('>', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          break;
        case 9:
          oledWriteChar1x('>', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          break;
        case 10:
          oledWriteChar1x('>', 0xB6, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB7, 0*6);
          break;
        case 11:
          oledWriteChar1x('>', 0xB7, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          break;
        case 12:
          oledWriteChar1x('>', 0xB2, 0*6);
          oledWriteChar1x(' ', 0xB3, 0*6);
          break;
        case 13:
          oledWriteChar1x('>', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB2, 0*6);
          oledWriteChar1x(' ', 0xB4, 0*6);
          break;
        case 14:
          oledWriteChar1x('>', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB3, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          break;
        case 15:
          oledWriteChar1x('>', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB4, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          break;
        case 16:
          oledWriteChar1x('>', 0xB6, 0*6);
          oledWriteChar1x(' ', 0xB5, 0*6);
          oledWriteChar1x(' ', 0xB7, 0*6);
          break;
        case 17:
          oledWriteChar1x('>', 0xB7, 0*6);
          oledWriteChar1x(' ', 0xB6, 0*6);
          break;
        }
    }
    else if (functionallity == 3){               //left right functionallity
      switch (selection){
        case 0:
          oledWriteChar1x('^', 0xB7, 2*6);
          selection = 1;
          left_right_LastState = 1;
          break;
        case 1:
          if (left_right_LastState != selection){
            if (L < 600){
              while (L < 700)
                L = mTouchReadButton(3);
              selection = 4;
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 2*6);
              oledWriteChar1x('^', 0xB7, 17*6);
            }
            else if (R < 600){
              while (R < 700)
                R = mTouchReadButton(0);
              // move arrow to the right selection
              selection = 2;
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 2*6);
              oledWriteChar1x('^', 0xB7, 7*6);
            }
          }
          break;
        case 2:
          if (left_right_LastState != selection){
            if (L < 600){
              while (L < 700)
                L = mTouchReadButton(3);              
              // move arrow to the left selection
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 7*6);
              oledWriteChar1x('^', 0xB7, 2*6);
              selection = 1;
            }
            else if (R < 600){
              while (R < 700)
                R = mTouchReadButton(0);
              // move arrow to the right selection
              selection = 3;
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 7*6);
              oledWriteChar1x('^', 0xB7, 12*6);
            }
          }
          break;
        case 3:
          if (left_right_LastState != selection){
            if (L < 600){
              while (L < 700)
                L = mTouchReadButton(3);              
              // move arrow to the left selection
              menuPrinter(selectMenu);
              selection = 2;
              oledWriteChar1x(' ', 0xB7, 12*6);
              oledWriteChar1x('^', 0xB7, 7*6);
            }
            else if (R < 600){
              while (R < 700)
                R = mTouchReadButton(0);
              // move arrow to the right selection
              selection = 4;
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 12*6);
              oledWriteChar1x('^', 0xB7, 17*6);
            }
          }
          break;
        case 4:
          if (left_right_LastState != selection){
            if (L < 600){
              while (L < 700)
                L = mTouchReadButton(3);              
              // move arrow to the left selection
              menuPrinter(selectMenu);
              selection = 3;
              oledWriteChar1x(' ', 0xB7, 17*6);
              oledWriteChar1x('^', 0xB7, 12*6);
            }
            else if (R < 600){
              while (R < 700)
                R = mTouchReadButton(0);
              selection = 1;
              menuPrinter(selectMenu);
              oledWriteChar1x(' ', 0xB7, 17*6);
              oledWriteChar1x('^', 0xB7, 2*6);
            }
          }
          break;
      }
    }
    else if (functionallity == 4){               //shake execute
      resultY = readAcc(0x05);
      if (resultY > 500)
        selectMenu = 0;
      if (resultY > 100 && resultY < 500)
        oledPutString(shakeHard, 6, 5*6);
    }
    else if (functionallity == 5){                //push the button execute
      ADCON0 = 0x11;
      while (!CheckButtonPressed()){
        selectMenu = 0;
        selection = 0;
      }
      ADCON0 = 0x13;
    }
   } 
}//end main

/** EOF main.c *************************************************/