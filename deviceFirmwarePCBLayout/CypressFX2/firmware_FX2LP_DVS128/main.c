#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:      main.c
//   Description: FX2LP firmware for the TCVS320 and Tmpdiff128 (new small board) retina chip   
//
// created: 1/2008, cloned from tmpdiff128 stereo board firmware
// Revision: 0.01 
// authors raphael berner, patrick lichtsteiner, tobi delbruck
//
//-----------------------------------------------------------------------------
#include "lp.h"
#include "lpregs.h"
#include "syncdly.h"            // SYNCDELAY macro
#include "biasgen.h" 
#include "portsFX2.h"
#include "ports.h"
#include "micro.h"

extern BOOL GotSUD;             // Received setup data flag
//extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

//BYTE Configuration;             // Current configuration
//BYTE AlternateSetting;          // Alternate settings

//WORD packetSize;

#define CPLD_NOT_RESET 			PA3
#define RUN_CPLD				PA0
#define RESET_TS				PC0
#define TIMESTAMP_MASTER 		PC1
#define CFG_TIMESTAMP_COUNTER 	PC2
#define TIMESTAMP_MODE			PC3

#define DB_Addr 1 // zero if only one byte address is needed for EEPROM, one if two byte address

#define LED 	PA7

#define EEPROM_SIZE 0x8000
#define MAX_NAME_LENGTH 4
#define STRING_ADDRESS (EEPROM_SIZE - MAX_NAME_LENGTH)

#define MSG_TS_RESET 1

// vendor requests
#define VR_ENABLE_AE_IN 0xB3 // enable IN transfers
#define VR_DISABLE_AE_IN 0xB4 // disable IN transfers
#define VR_TRIGGER_ADVANCE_TRANSFER 0xB7 // trigger in packet commit (for host requests for early access to AE data) NOT IMPLEMENTED
#define VR_RESETTIMESTAMPS 0xBb 
#define VR_SET_DEVICE_NAME 0xC2
#define VR_TIMESTAMP_TICK 0xC3
#define VR_RESET_FIFOS 0xC4
#define VR_DOWNLOAD_CPLD_CODE 0xC5 
#define VR_READOUT_EEPROM 0xC9
#define VR_IS_TS_MASTER 0xCB
#define VR_MISSED_EVENTS 0xCC

#define VR_WRITE_BIASGEN 0xB8 // write bytes out to SPI
				// the wLengthL field of SETUPDAT specifies the number of bytes to write out (max 64 per request)
				// the bytes are in the data packet
#define VR_SET_POWERDOWN 0xB9 // control powerDown. wValue controls the powerDown pin. Raise high to power off, lower to power on.
#define VR_EEPROM_BIASGEN_BYTES 0xBa // write bytes out to EEPROM for power on default

#define VR_SETARRAYRESET 0xBc // set the state of the array reset
#define VR_DOARRAYRESET 0xBd // toggle the array reset low long enough to reset all pixels. TCVS320/DVS320 don't have this.
#define VR_SYNC_ENABLE 0xBe // sets whether sync events are sent on slave clock input instead of acting as slave clock.
//sbit arrayReset=IOE^5;	// arrayReset=0 to reset all pixels, this on port E.5 but is not bit addressable
// arrayReset is active low, low=reset pixel array, high=operate normally
#define ARRAY_RESET_MASK=0x20
#define NOT_ARRAY_RESET_MASK=0xdf;
#define setArrayReset() 	IOE=IOE&NOT_ARRAY_RESET_MASK	
#define releaseArrayReset()	IOE=IOE|ARRAY_RESET_MASK

#define BIAS_FLASH_START 9 // start of bias value (this is where number of bytes is stored

#define	VR_UPLOAD		0xc0
#define VR_DOWNLOAD		0x40
#define VR_EEPROM		0xa2 // loads (uploads) EEPROM
#define	VR_RAM			0xa3 // loads (uploads) external ram

#define EP0BUFF_SIZE	0x40

//BYTE operationMode;

#define NUM_BIAS_BYTES 36
unsigned int numBiasBytes; // number of bias bytes saved
/*                  
initial bias bytes values specified here
these come from DVS128Fast.xml settings which are designed to
bias quite strongly in order to charge the biases quickly if they are knocked out
of place by shift register power up values that are way off.

<entry key="DVS128.IPot.cas" value="1992"/>
<entry key="DVS128.IPot.injGnd" value="1108364"/>
<entry key="DVS128.IPot.reqPd" value="16777215"/>
<entry key="DVS128.IPot.puX" value="8159221"/>
<entry key="DVS128.IPot.diffOff" value="132"/>
<entry key="DVS128.IPot.req" value="159147"/>
<entry key="DVS128.IPot.refr" value="969"/>
<entry key="DVS128.IPot.puY" value="16777215"/>
<entry key="DVS128.IPot.diffOn" value="209996"/>
<entry key="DVS128.IPot.diff" value="13125"/>
<entry key="DVS128.IPot.foll" value="271"/>
<entry key="DVS128.IPot.Pr" value="217"/>
*/
xdata unsigned char biasBytes[]={0x00,0x07,0xc8,	// cas
								0x10,0xe9,0x8C,		// injGnd
								0xFF,0xFF,0xFF,		// reqPd
								0x7c,0x7f,0xf5,		// puX
								0x00,0x00,0x84,		// diffOff
								0x02,0x6d,0xab,		// req
								0x00,0x03,0xc9,		// refr
								0xFF,0xFF,0xFF,		// puY
								0x03,0x34,0x4c,		// diffOn
								0x00,0x33,0x45,		// diff
								0x00,0x01,0x0f,		// foll
								0x00,0x01,0x0f}; 	// Pr

long cycleCounter;
//long missedEvents;

BOOL JTAGinit;

#define	I2C_Addr 0x51 //adress is 0101_0001

void startMonitor(void);
void stopMonitor(void);

void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf);
void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf);
void EEPROMWriteBYTE(WORD addr, BYTE value);

void downloadSerialNumberFromEEPROM(void);


//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------

void TD_Init(void)              // Called once at startup
{
	BYTE i;  
	// set the CPU clock to 48MHz
	//CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1) ;
	CPUCS = 0x10 ; // 1_0000 : clockspeed 48MHz, don't drive output pin

	// set the slave FIFO interface to 30MHz, slave fifo mode
	IFCONFIG = 0xA3; // 1010_0011   // internal clock
	//IFCONFIG = 0x23; // 0010_0011  // extenal clock

	// disable interrupts by the input pins and by timers and serial ports:
	IE &= 0x00; // 0000_0000 

	// disable interrupt pins 4, 5 and 6
	EIE &= 0xE3; // 1110_0011;

	// Registers which require a synchronization delay, see section 15.14
	// FIFORESET        FIFOPINPOLAR
	// INPKTEND         OUTPKTEND
	// EPxBCH:L         REVCTL
	// GPIFTCB3         GPIFTCB2
	// GPIFTCB1         GPIFTCB0
	// EPxFIFOPFH:L     EPxAUTOINLENH:L
	// EPxFIFOCFG       EPxGPIFFLGSEL
	// PINFLAGSxx       EPxFIFOIRQ
	// EPxFIFOIE        GPIFIRQ
	// GPIFIE           GPIFADRH:L
	// UDMACRCH:L       EPxGPIFTRIG
	// GPIFTRIG
  
	//enable Port C and port E
	SYNCDELAY;
	PORTCCFG = 0x00;
	SYNCDELAY;
	PORTACFG = 0x00; // do not use interrupts 0 and 1
	SYNCDELAY;
	PORTECFG = 0x00;

	// hold CPLD in reset and configure 
	// TimestampCounter to 1 us Tick (0): 0000_0000
	IOC = 0x00; 
	IOA = 0x00;
	IOE=  0x02;          //set BiasClock low, biasLatch high

	OEC = 0x0D; // 0000_1101 // JTAG, timestampMode, timestampTick, timestampMaster, resetTimestamp
	OEE = 0xFE; // 1111_1110 configure only bit 0 (biasBitOut) as input
	OEA = 0x89; // 1000_1001 PA0: run, PA3: NotResetCPLD ;  PA7 LED
	
	EP1OUTCFG = 0x00;			// EP1OUT disabled
	SYNCDELAY;
	EP1INCFG = 0xA0;			// EP1IN enabled, bulk
	SYNCDELAY;                   
	EP2CFG = 0x00;				// EP2 disabled
	SYNCDELAY;                     
	EP4CFG = 0x00;				// EP4 disabled
	SYNCDELAY;                 
	EP6CFG = 0xE0;				// EP6 enabled, in bulk, quad buffered 
	SYNCDELAY;               
	EP8CFG = 0x00;				// EP8 disabled

	SYNCDELAY;
	REVCTL= 0x03;

	SYNCDELAY;
	FIFORESET = 0x80;
  	SYNCDELAY;
  	FIFORESET = 0x06;
  	SYNCDELAY;
  	FIFORESET = 0x00;
	SYNCDELAY;

	EP6AUTOINLENH=0x02;
	SYNCDELAY;
	EP6AUTOINLENL=0x00;

	SYNCDELAY;
	EP6FIFOCFG = 0x09 ; //0000_1001

	//set FIFO flag configuration: FlagB: EP6 full, flagC and D unused
	SYNCDELAY;
	PINFLAGSAB = 0xE8; // 1110_1000


	// initialize variables
//	operationMode=0;

	cycleCounter=0;
//	missedEvents=0xFFFFFFFF; // one interrupt is generated at startup, maybe some cpld registers start in high state
	LED=0;

	// biases, control ports
	biasInit();	// init biasgen ports and pins
	setArrayReset(); // keep pixels from spiking, reset all of them
	// pump powerdown to make sure masterbias is really started up
	for(i=0;i<20;i++)
	{
		setPowerDownBit();
		//EZUSB_Delay1ms();
		releasePowerDownBit();
		//EZUSB_Delay1ms();
	}
	EZUSB_Delay(10); // ms delay after masterbias (re)startup
	for (i=0;i<NUM_BIAS_BYTES;i++)
	{
		spiwritebyte(biasBytes[i]); // load hardcoded biases
	}
	latchNewBiases();	
	EZUSB_Delay(100); // ms
	releaseArrayReset(); // release arrayReset, let pixels spike



	EZUSB_InitI2C(); // init I2C to enable EEPROM read and write


	JTAGinit=TRUE;	

  	IT0=1;		// make INT0# edge-sensitive
	EX0=0;		// do not enable INT0#

	IT1=1; // INT1# edge-sensitve
	EX1=0; // do not enable INT1#

}

void TD_Poll(void)              // Called repeatedly while the device is idle
{ 	
	if(cycleCounter++>=50000){
		LED=!LED;	
		cycleCounter=0; // this makes a slow heartbeat on the LED to show firmware is running
	}		
}

void downloadSerialNumberFromEEPROM(void)
{
	BYTE i;

	char *dscrRAM;
	BYTE xdata buf[MAX_NAME_LENGTH];

	// get pointer to string descriptor 3
	dscrRAM =  (char *)EZUSB_GetStringDscr(3);

	// read string description from EEPROM
	EEPROMRead(STRING_ADDRESS, MAX_NAME_LENGTH, buf);
	
	//write string description (serial number) to RAM
	for (i=0;i<MAX_NAME_LENGTH;i++)
	{
		dscrRAM[2+i*2] = buf[i];
	}
}

void startMonitor(void)
{
	CPLD_NOT_RESET=1;
    RUN_CPLD=1;
}

void stopMonitor(void)
{
    RUN_CPLD=0;

  	// force last paket
  	
  	EP6FIFOCFG = 0x01; //0000_0001 disable auto-in
	SYNCDELAY;

	if(EP6FIFOFLGS==0x00)
	{ // if buffer available
    	INPKTEND=0x06; // force in paket
		SYNCDELAY;
	}

  	// reset fifo  	
  	FIFORESET = 0x80;
  	SYNCDELAY;
  	FIFORESET = 0x06;
  	SYNCDELAY;
  	FIFORESET = 0x00;
	SYNCDELAY;

	EP6FIFOCFG =0x09;  //0000_1001 set back to autoin
	SYNCDELAY;
}


void EEPROMWriteByte(WORD addr, BYTE value)
{
	BYTE		i = 0;
	BYTE xdata 	ee_str[3];
	if(DB_Addr)
		ee_str[i++] = MSB(addr); // if 16 bit, we need 2-byte address and 1 byte data

	ee_str[i++] = LSB(addr);
	ee_str[i++] = value;

	LED=1;
	EZUSB_WriteI2C(I2C_Addr, i, ee_str);
	LED=0;
    EZUSB_WaitForEEPROMWrite(I2C_Addr);
}

void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf)
{
	BYTE	i;
	for(i=0;i<length;++i)
		EEPROMWriteByte(addr++,buf[i]);
}

void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf)
{
	BYTE		i = 0;
	BYTE		j = 0;
	BYTE xdata 	ee_str[2];

	if(DB_Addr)
		ee_str[i++] = MSB(addr);

	ee_str[i++] = LSB(addr);

	EZUSB_WriteI2C(I2C_Addr, i, ee_str);

	for(j=0; j < length; j++)
		*(buf+j) = 0xcd;

	EZUSB_ReadI2C(I2C_Addr, length, buf);
}


BOOL TD_Suspend(void)          // Called before the device goes into suspend mode
{
  // reset CPLD
  CPLD_NOT_RESET =0;  
  
  return(TRUE);
}

BOOL TD_Resume(void)          // Called after the device resumes
{
  // activate CPLD 
   CPLD_NOT_RESET=1; 

   return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

/*BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}*/

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{
  if( EZUSB_HIGHSPEED( ) )
  { // FX2 enumerated at high speed
    SYNCDELAY;                  // 
    EP6AUTOINLENH = 0x02;       // set AUTOIN commit length to 512 bytes
    SYNCDELAY;                  // 
    EP6AUTOINLENL = 0x00;
    SYNCDELAY;                  
   // enum_high_speed = TRUE;
    }
  else
  { // FX2 enumerated at full speed
    SYNCDELAY;                   
    EP6AUTOINLENH = 0x00;       // set AUTOIN commit length to 64 bytes
    SYNCDELAY;                   
    EP6AUTOINLENL = 0x40;
    SYNCDELAY;                  
  //  enum_high_speed = FALSE;
  }

  //Configuration = SETUPDAT[2];
  return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = 0x00;//Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

/*BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
//   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}*/

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = 0x00;//AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

/*BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}*/

// the SETUPDAT array has the following 8 elements (see FX2 TRM Section 2.3)
//SETUPDAT[0] VendorRequest 0x40 for OUT type, 0xC0 for IN type
//SETUPDAT[1] The actual vendor request (e.g. VR_ENABLE_AE_IN below)
//SETUPDAT[2] wValueL 16 bit value LSB
//         3  wValueH MSB
//         4  wIndexL 16 bit field, varies according to request
//         5  wIndexH
//         6  wLengthL Number of bytes to transfer if there is a data phase
//         7  wLengthH
BYTE xsvfReturn;

BOOL DR_VendorCmnd(void)
{	
	WORD addr, len, bc; // xdata used here to conserve data ram; if not EEPROM writes don't work anymore
	WORD i;
	char *dscrRAM;
	unsigned char xdata JTAGdata[400];

	switch (SETUPDAT[1]){
		case VR_ENABLE_AE_IN: // enable IN transfers
			{
				startMonitor();
				break;  // handshake phase triggered below
			}
		case VR_DISABLE_AE_IN: // disable IN transfers
			{
				stopMonitor();
				break;
			}
		case VR_RESET_FIFOS: // reset in and out fifo
			{
				SYNCDELAY;
				EP6FIFOCFG = 0x01; //0000_0001  disable auto-in
				SYNCDELAY;
				FIFORESET = 0x80;
				SYNCDELAY;
				FIFORESET = 0x06;
				SYNCDELAY;
				FIFORESET = 0x00;


				SYNCDELAY;
				EP6FIFOCFG = 0x09 ; //0000_1001 reenable auto-in
				break;
			}
		case VR_DOWNLOAD_CPLD_CODE:
			{
			if (SETUPDAT[0]==VR_DOWNLOAD) {
				if (JTAGinit)
				{
					IOC=0x00;
					OEC = 0xBD;   // configure TDO (bit 6) and TSmaster as input  : 1011_1101
			
					xsvfInitialize();
					JTAGinit=FALSE;
					
				}

				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;

				if (len>400)
				{
					xsvfReturn=10;
					OEC = 0x0D;   // configure JTAG pins to float : 0000_1111
					JTAGinit=TRUE;
					break;
				}

				addr=0;

				resetReadCounter(JTAGdata);

				while(len)					// Move new data through EP0OUT 
				{							// one packet at a time.
					// Arm endpoint - do it here to clear (after sud avail)
					EP0BCH = 0;
					EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing

					while(EP0CS & bmEPBUSY);

					bc = EP0BCL; // Get the new bytecount

					for(i=0; i<bc; i++)
							JTAGdata[addr+i] = EP0BUF[i];							

					addr += bc;
					len -= bc;
				}
			

				if (SETUPDAT[2]==0x00) //complete
				{
					OEC = 0x0D;   // configure JTAG pins to float : 0000_1111
					JTAGinit=TRUE;
				} else
				{
					xsvfReturn=xsvfRun();
					if (xsvfReturn>0) // returns true if error
					{
						OEC = 0x0D;   // configure JTAG pins to float : 0000_1101
						JTAGinit=TRUE;
				
					//	return TRUE;
					}

				}
	
				/* EP0BUF[0] = SETUPDAT[1];
				EP0BCH = 0;
				EP0BCL = 1;
				EP0CS |= bmHSNAK;

				return(FALSE); */
				break;
			}
 			else //case VR_XSVF_ERROR_CODE:
			{
				EP0BUF[0] = SETUPDAT[1];
				EP0BUF[1]= xsvfReturn;
				EP0BCH = 0;
				EP0BCL = 2;
				EP0CS |= bmHSNAK;

				return(FALSE);
			}
			}
		case VR_SET_DEVICE_NAME:
			{
				*EP0BUF = SETUPDAT[1];
				EP0BCH = 0;
				EP0BCL = 1;
				EP0CS |= bmHSNAK;

				while(EP0CS & bmEPBUSY); //wait for the data packet to arrive

				dscrRAM = (char*)EZUSB_GetStringDscr(3); // get address of serial number descriptor-string in RAM

				if (EP0BCL > MAX_NAME_LENGTH)
				{
					len=MAX_NAME_LENGTH;
				} else 
				{
					len=EP0BCL;
				}
	
				for (i=0;i<len;i++)
				{
					EEPROMWriteBYTE(STRING_ADDRESS+i, EP0BUF[i]); // write string to EEPROM
					dscrRAM[2+i*2] = EP0BUF[i]; // write string to RAM
				}

				for (i=len; i<MAX_NAME_LENGTH; i++) // fill the rest with stop characters
				{
					EEPROMWriteBYTE(STRING_ADDRESS+i, ' '); // write string to EEPROM				
					dscrRAM[2+i*2] = ' '; // write string to RAM
				}

				EP0BCH = 0;
				EP0BCL = 0;

				return(FALSE);
			}		
		case VR_RESETTIMESTAMPS:
			{
				RESET_TS=1; // assert RESET_TS pin for one instruction cycle (four clock cycles)
				RESET_TS=0;

				break;
			}
		case VR_WRITE_BIASGEN: // write bytes to SPI interface
		case VR_EEPROM_BIASGEN_BYTES: // falls through and actual command is tested below
			{
				SYNCDELAY;
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
				numBiasBytes=len;
				while(len){					// Move new data through EP0OUT, one packet at a time
					// Arm endpoint - do it here to clear (after sud avail)
					EP0BCH = 0;
					EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing
					while(EP0CS & bmEPBUSY);  // spin here until data arrives
					bc = EP0BCL; // Get the new bytecount
					// Is this a  download to biasgen shift register?
					if(SETUPDAT[1] == VR_WRITE_BIASGEN){
						for(i=0; i<bc; i++){
							spiwritebyte(EP0BUF[i]);
							biasBytes[i]=EP0BUF[i];
						}
					}else{ // we write EEProm starting at addr with bc bytes from EP0BUF
						//					EEPROMWrite(addr,bc,(WORD)EP0BUF);
					}
					addr += bc;	// inc eeprom addr to write to, in case that's what we're doing
					len -= bc; // dec total byte count
				}
				if(SETUPDAT[1]==VR_WRITE_BIASGEN) {
					latchNewBiases();
					//setLatchTransparent(); // let values pass through latch from shift register -- these are new values
					//setLatchOpaque();
				}
				EP0BCH = 0;
				EP0BCL = 0;                   // Arm endpoint with 0 byte to transfer
				return(FALSE); // very important, otherwise get stall
			}
		case VR_SET_POWERDOWN: // control powerDown output bit
			{
				if (SETUPDAT[2])
				{
					setPowerDownBit();
				} else 
				{
					releasePowerDownBit();
				}
				*EP0BUF=VR_SET_POWERDOWN;
				SYNCDELAY;
				EP0BCH = 0;
				EP0BCL = 1;                   // Arm endpoint with 1 byte to transfer
				EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
				break; // very important, otherwise get stall

			}
		case VR_SETARRAYRESET: // set array reset, based on lsb of argument
			{
				if (SETUPDAT[2]&0x01)
				{
					setArrayReset(); //IOE|=arrayReset;
				} else
				{
					releaseArrayReset(); 
				}
			
				*EP0BUF=VR_SETARRAYRESET;
				SYNCDELAY;
				EP0BCH = 0;
				EP0BCL = 1;                   // Arm endpoint with 1 byte to transfer
				EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
				return(FALSE); // very important, otherwise get stall

			}
		case VR_SYNC_ENABLE: // sets sync event output or master/slave clocking, based on lsb of argument
			{
				if (SETUPDAT[2]&0x01)
				{
					setArrayReset(); //IOE|=arrayReset; // TODO change to SYNC_ENABLE
				} else
				{
					releaseArrayReset(); 
				}
			
				*EP0BUF=VR_SYNC_ENABLE;
				SYNCDELAY;
				EP0BCH = 0;
				EP0BCL = 1;                   // Arm endpoint with 1 byte to transfer
				EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
				return(FALSE); // very important, otherwise get stall

			}
		case VR_DOARRAYRESET: // reset array for fixed reset time
			{
				setArrayReset(); 
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();	// a few us
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				releaseArrayReset(); //IOE|=arrayReset;
				*EP0BUF=VR_DOARRAYRESET;
				SYNCDELAY;
				EP0BCH = 0;
				EP0BCL = 1;                   // Arm endpoint with 1 byte to transfer
				EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
				return (FALSE); // very important, otherwise get stall
			}
	/*	case VR_TIMESTAMP_TICK:
			{
				if (SETUPDAT[0]==VR_UPLOAD) //1010_0000 :vendor request to device, direction IN
				{
					EP0BUF[0] = SETUPDAT[1];
				
					EP0BUF[1]= operationMode;
					
					EP0BCH = 0;
					EP0BCL = 2;
					EP0CS |= bmHSNAK;
				} else
				{
					operationMode=SETUPDAT[2];
					if (operationMode==0)
					{
						TIMESTAMP_MODE = 0;
						CFG_TIMESTAMP_COUNTER = 0;
					}else if (operationMode==1)
					{
  						CFG_TIMESTAMP_COUNTER = 1;
						TIMESTAMP_MODE = 0;	
					}else if (operationMode==2)
					{
  						CFG_TIMESTAMP_COUNTER = 0;
						TIMESTAMP_MODE = 1;	
					}else if (operationMode==3)
					{
  						CFG_TIMESTAMP_COUNTER = 1;
						TIMESTAMP_MODE = 1;	
					}

					*EP0BUF = SETUPDAT[1];
					EP0BCH = 0;
					EP0BCL = 1;
					EP0CS |= bmHSNAK;	
				}
				return(FALSE);
			}*/
		case VR_IS_TS_MASTER:
			{
				EP0BUF[0] = SETUPDAT[1];
				EP0BUF[1]= TIMESTAMP_MASTER;
				EP0BCH = 0;
				EP0BCL = 2;
				EP0CS |= bmHSNAK;

				return(FALSE);
			}
	/*	case VR_MISSED_EVENTS:
			{
				EX1=0;
				EP0BUF[0] = SETUPDAT[1];
				EP0BUF[4]= (missedEvents & 0xFF000000) >> 24;
				EP0BUF[3]= (missedEvents & 0x00FF0000) >> 16;
				EP0BUF[2]= (missedEvents & 0x0000FF00) >> 8;
				EP0BUF[1]= missedEvents & 0x000000FF;
				EP0BCH = 0;
				EP0BCL = 5;
				EP0CS |= bmHSNAK;

				missedEvents=0;
				EX1=1;
				return(FALSE);
			}*/
		case VR_RAM:
		case VR_EEPROM:
		{
			addr = SETUPDAT[2];		// Get address and length
			addr |= SETUPDAT[3] << 8;
			len = SETUPDAT[6];
			len |= SETUPDAT[7] << 8;
			// Is this an upload command ?
			if(SETUPDAT[0] == VR_UPLOAD)  // this is automatically defined on host from direction of vendor request
			{
				while(len)					// Move requested data through EP0IN 
				{							// one packet at a time.

					while(EP0CS & bmEPBUSY);

					if(len < EP0BUFF_SIZE)
						bc = len;
					else
						bc = EP0BUFF_SIZE;

					// Is this a RAM upload ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = *((BYTE xdata *)addr+i);
					}
					else
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = 0xcd;
						EEPROMRead(addr,(WORD)bc,(WORD)EP0BUF);
					}

					EP0BCH = 0;
					EP0BCL = (BYTE)bc; // Arm endpoint with # bytes to transfer

					addr += bc;
					len -= bc;

				}
			}
			// Is this a download command ?
			else if(SETUPDAT[0] == VR_DOWNLOAD) // this is automatically defined on host from direction of vendor request
			{
				while(len)					// Move new data through EP0OUT 
				{							// one packet at a time.
					// Arm endpoint - do it here to clear (after sud avail)
					EP0BCH = 0;
					EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing

					while(EP0CS & bmEPBUSY);

					bc = EP0BCL; // Get the new bytecount

					// Is this a RAM download ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							*((BYTE xdata *)addr+i) = *(EP0BUF+i);
					}
					else
						EEPROMWrite(addr,bc,(WORD)EP0BUF);

					addr += bc;
					len -= bc;
				}
			}
			return(FALSE);
		}
		default:
		{ // we received an invalid command
			return(TRUE);
		}
	}

	*EP0BUF = SETUPDAT[1];
	EP0BCH = 0;
	EP0BCL = 1;
	EP0CS |= bmHSNAK;

	return(FALSE);
}

// no interrupts are used for TCVS320
// RESET HOST TIMESTAMP INTERRUPT not used
/*void ISR_TSReset(void) interrupt 3 {
	LED=0;
	
	SYNCDELAY; // reset fifos to delete events with the old timestamps
	FIFORESET = 0x80;
	SYNCDELAY;
	FIFORESET = 0x06;
	SYNCDELAY;
	FIFORESET = 0x00;

	SYNCDELAY;
	EP6FIFOCFG = 0x09 ; //0000_1001


	if (EP1INCS!=0x02)
	{
		EP1INBUF[0]=MSG_TS_RESET;
		SYNCDELAY;
		EP1INBC=1;
		SYNCDELAY;
		IE0=0; // clear interrupt
		EX0=1; // enable INT0# external interrupt
		LED=1;
	}
}

void ISR_MissedEvent(void) interrupt 3 {	
	missedEvents++;
}*/

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
    //  packetSize = 512;

   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
    //  packetSize = 64;
   }
   
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
//   Sleep = TRUE;
//   EZUSB_IRQ_CLEAR();
//   USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
    //  packetSize = 512;

   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
    //  packetSize = 64;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}
