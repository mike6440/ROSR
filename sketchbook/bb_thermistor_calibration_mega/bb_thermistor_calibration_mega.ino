#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


// NOTE ========= DEFINES
#define PROGRAMNAME "bb_thermistor_calibration_mega"
#define VERSION		"2"
#define EDITDATE	"150830, 1330L"
const byte	EEPROM_ID = 3;		// EEPROM
int istart; //test	set to 1 at startup so we can go to a menu.

//NOTE ====== INCLUDES
#include <string.h>
#include <math.h>
#include <EEPROM.h>

//Added for the ADS rosr code uses 'ads0' while the example gives 'ads'
Adafruit_ADS1115 ads0;	// ad0, u13, construct an ads1115 at address 0x48
// ads lines from rosr
Adafruit_ADS1115 ads1(0x49);	// ad1, u16, construct an ads1115 at address 0x49
//Adafruit_ADS1115 ads2(0x4A);	  // ad2, u14, construct an ads1115 at address 0x4A
//Adafruit_ADS1115 ads3(0x4B);	// ad3, u17, spare, 

//NOTE ====== DIGITAL LINES
const int REFSW = 5;	   // thermistor reference voltage

// ANALOG ASSIGNMENTS
// three 4-chan 16 bit adc, adc#=(unsigned)(x/4), chan#=x%4;
const int ADCMAX = 8;	   // =12 normally. maximum number of ADC channels
const int Ntcase = 0;		 // case thermistor 1
const int Ntdome = 1;		 // dome thermistor1
const int NRF	= 2;		// 5REF/2
const int NVin =	3;		// VIN/4 
//const int Nspare1 = 4;	 // Spare input 1 

//NOTE ====== CONSTANTS
const char OK = 1;
const char NOTOK = 0;
const int MISSING = -999;
const int POS = 1;
const int NEG = -1;
const int CCW = 1;			// forward direction
const int CW = -1;			// reverse direction
const int STOP = 0;			// motor stop
const int ON = 1;
const int OFF = 0;
const int OPEN = 1;
const int CLOSED = 0;
const char SPACE = ' ';
const char ZERO = '0';
const double ADC2VOLTS = 1;	 //.2504e-4;

// test menu
const char TEST=-1;
const char RUN=1;
// TEMPERATURE
const int WARMUPMILLISECS = 50;
const int ADCREADWAIT = 100;
const double TMAX = 50;
const double TMIN = -5;

//=========================================================================
//NOTE ====== GLOBAL VARIABLES
//=========================================================================
// system time
unsigned long msstart;	//set at start, milliseconds
unsigned long menustart;  


//======== EEPROM =============================
struct eeprom {
	byte id;
	double swcal,lwcal;
};
struct eeprom ee;
int eeSize;

	// DEFAULT EEPROM
const double default_swcal=9;
const double default_lwcal=3;


// user i/o
char	RunMode;

// READING THE ADC
unsigned long nsamp;						// counter for each sample.
double vmean[ADCMAX];						// adc16 read
double Vref;								//compute from adc16;

// ADC16 limits for BB temperature cal. volts
double threshold[2]={
	1.00,4.00};


//NOTE ====== FUNCTIONS
// i/o
void			Action(char*);
unsigned int	checksum_nmea(char *);
unsigned long	ElapsedTime (char *);
void			EepromDefault(void);
void			EepromStore(void);
void			EepromRead(void);
void			EepromPrint(void);
char *			floatToString(char *, double, byte, byte);
char *			floatToStringSci(char *, double, byte, byte);
double			GetAdcVolts (unsigned int );   // ads adc
unsigned int	GetAdcSample(unsigned int ch, double *vmean);	// ads adc
// unsigned int		GetThermCoefs(unsigned int, double *coef );	  // ads adc
long		Hex2Dec(char *);
void		MeanStdev(double *, double *, int, double );
void		PrintProgramID(void);
void		PrintBytes(char *);
unsigned	Ref(int);
int			sign (float);
double		SteinhartHart(double beta[], double r);
double		ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm);

//============================================================================
void setup() {	 
	// user interface
	Serial.begin(9600);	// serial 
	Serial.println("START");

	// DIGITAL
	pinMode(REFSW, OUTPUT);		// 5ref on/off

//	   digitalWrite(BB1, LOW);	   // BB1 heater start off;

	// signon
	PrintProgramID();

	// 5REF START OFF
	Ref(OFF);

	// set mem for eeprom
	eeSize=sizeof(struct eeprom);
	EepromRead();
	Serial.print("test EEPROM_ID = "); Serial.println(EEPROM_ID);
	Serial.print("test ID = "); Serial.println(ee.id);
	if(ee.id != EEPROM_ID ) {
		EepromDefault();
	};


	// ADS1115 ADC
	ads0.begin();
	// ads0.setGain(GAIN_ONE);		// 1x gain	 +/- 4.096V
	// ads0.setGain(GAIN_ONE);	   // 1x gain	+/- 4.096V	1 bit = 2mV
	// ads0.setGain(GAIN_TWO);	   // 2x gain	+/- 2.048V	1 bit = 1mV
	// ads0.setGain(GAIN_FOUR);	   // 4x gain	+/- 1.024V	1 bit = 0.5mV
	// ads0.setGain(GAIN_EIGHT);   // 8x gain	+/- 0.512V	1 bit = 0.25mV
	// ads0.setGain(GAIN_SIXTEEN); // 16x gain	+/- 0.256V 
	ads1.begin();
	// ads1.setGain(GAIN_ONE);		// 1x gain	 +/- 4.096V
//	   ads2.begin();
//	   ads2.setGain(GAIN_ONE);	   // 1x gain	+/- 4.096V
	
	msstart=millis();
	istart=1;
}

//=============================================================================
void loop() {
	int		i=0;		
	unsigned int	b=0;
	char	buf[20]; 
	char	OutStr[1024], AvStr[512], SsstStr[512];
	double ddum, fdum;
	unsigned long ldum;


	Serial.setTimeout(1000); // 1 sec wait for a key entry
	i=Serial.readBytes(buf,1);	 // normal operation -- wait for key entry
	if( i > 0 || istart==1){
		if ( buf[0] == 't' || buf[0] == 'T' || istart==1){
			menustart=millis();
			istart=0;	// normal operation
			RunMode = TEST;
			Serial.setTimeout(10000); // 10 sec wait for a key entry
			while( RunMode == TEST ) {
				// prompt
				//while(Serial.read()>0); //test clear the buffer
				Serial.print("> ");
				// Wait for a command
				i = Serial.readBytesUntil(13,buf,11);
				buf[i]='\0';
				if( i > 0 ){
					// G - go to main loop
					if( buf[0] == 'G' || buf[0] == 'g' ) {
						RunMode = RUN;
						Serial.println("Begin Operation...");
					}
					else {
						Serial.println("");
						Action(buf);
						menustart=millis();
					}
				}
			}
		}
	}
	// OUTPUT STRING	 
	// header
	strcpy(OutStr,"$TCAL1,\0");

	// Elapsed time, days, since startup
	ElapsedTime(buf);
	strcat(OutStr,buf);// end of AvStr strcat(AvStr,",");
	strcat(OutStr,",\0");	

	// ADC16 READ
	// REF POWER ON
	Ref(ON);
	delay(WARMUPMILLISECS);
	// ALL ADC CHANS
	for(i=0; i<ADCMAX; i++){
		GetAdcSample(i, (vmean+i));
		floatToString(buf,vmean[i],4,8);
		strcat(OutStr,buf);	 
		strcat(OutStr,",");
	}
	// REF POWER OFF
	Ref(OFF);
	// COMPUTE VREF
	Vref=vmean[3]*2;
	floatToString(buf,Vref,4,8);
	strcat(OutStr,buf);	 
	strcat(OutStr,",");
	// CHECKSUM AND END
	b = checksum_nmea(OutStr+1);
	Serial.print(OutStr); 
	Serial.print("*"); 
	Serial.print(b,HEX); 
	Serial.print("\r\n");
	delay(500);
}
// =============== END OF LOOP =======================================


//*****************************************************************
void	Action(char *cmd)
{
	//	Read message and take action.
	// Create an output packet in out_buffer and send;
	// input:
	// in_buffer = pointer to a message string
	// in_fromlist = char string, e.g. "121115"
	//	meaning the message came from 12 in the route 15->11->12
	char str[50], str1[10];
	char yesno[4], eok=0;
	double fdum, ddum, av1, av2, fsum1, fsq1, fsum2, fsq2;
	unsigned int nsum,ix;
	char *stop_at;
	byte i,ib;
	unsigned long Ldum;
	int n,n1;
	
	// TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
	if(cmd[0] == '?'){
		PrintProgramID();
		Serial.println("------- EEPROM -----------------------------------");
		Serial.println("E		 -- show eeprom ");
		Serial.println("Egfff.f -- SW Rad Cal			EGfff.f -- LW Rad Cal");
		Serial.println("");
		Serial.println("------- FUNCTIONS -----------------------------------");
		Serial.println("an -- ADC Chan n					 A		-- ADC all loop");
		Serial.println("c	-- 5REF OFF						 C		-- 5REF ON");
		Serial.println("t	  -- Read system clock");
		Serial.println("v/V  -- Program version");
		Serial.println("g/G  -- Continue sampling.");
	}
	// ADC LOOP
    else if(cmd[0] == 'a' && strlen(cmd) > 1){
        ix = cmd[1]-48;
        if(ix < 0 || ix > 7){
            Serial.println("Error");
        } 
        else { 
			GetAdcSample(ix, &fdum);
            Serial.println(fdum,4);
        }
    }
	else if(cmd[0] == 'A'){
		Serial.println("READ ALL ADC");
		Ref(ON);
        //  v0     v1     v2     v3     v4     v5     v6     v7 
		// 2.630 2.634 0.409 2.473 2.632 2.630 1.812 2.575 4.096 3.664 -0.000 2.464	 21.85	21.90  21.93  21.93	 52.31	25.19	22.97  39.11  14.7	 4.946	-0.0
        Serial.println(" v0       v1       v2       v3       v4       v5       v6      v7 ");
		for(n=0; n<10; n++){
			for(ix=0; ix<ADCMAX; ix++){
				GetAdcSample(ix, &fdum);
				Serial.print(fdum,4);Serial.print("   ");	
			}
			Serial.println("");
			delay(800);
		}
	 }
	// 5REF TOGGLE	ON/OFF
	else if(cmd[0] == 'C' ){
		Serial.println("5REF ON");
		Ref(ON);
	}
	else if(cmd[0] == 'c' ){
		Serial.println("5REF OFF");
		Ref(OFF);
	}
	// TIME
	else if(cmd[0] == 't'){
		Serial.println("msec	  dd.hhmmss");
		while(1){
			if( Serial.available()){
				break;
			}
			Ldum = ElapsedTime(str);
			Serial.print(Ldum); Serial.print("   "); Serial.println(str);
			delay(2000);
		}
	}
	// EEPROM
	else if( cmd[0] == 'E' || cmd[0]=='e'){
		if( strlen(cmd) <= 1 ){
			EepromPrint();
		}
		else {
			eok=0;
			if( cmd[1]=='g'){
				ddum = atof(cmd+2);
				//Serial.print("SW CAL = ");  Serial.println(ddum,3);
				ee.swcal=ddum;
				eok=1;
			}
			else if( cmd[1]=='G' ){
				ddum = atof(cmd+2);
				//Serial.print("SET LW CAL = ");  Serial.println(ddum,3);
				ee.lwcal=ddum;
				eok=1;
			}
			// STORE EEPROM 
			if( eok == 1 ) {
				EepromStore();
				EepromRead();
				EepromPrint();
			}
		}
	}
	// Test float
	else if(cmd[0] == 'f'){
		//floatToString(char * outstr, double val, byte precision, byte widthp)
		floatToStringSci(str, 0.00004567654,3,0);
		Serial.print("<"); Serial.print(str); Serial.println(">");
	}
	
	// VERSION
	else if(cmd[0] == 'v' || cmd[0] == 'V'){
		PrintProgramID();
	}
	// RETURN TO RUN
	else if(cmd[0] == 'g' || cmd[0] == 'G'){
		RunMode == RUN;
	}
	// DEFAULT
	else {
		Serial.print("<"); Serial.print(cmd); Serial.print(">  ");
		Serial.println(" not recognized.");
	}
	cmd[0]='\0';	
	return;
}

//==========================================================================
// double ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm){
// 
//	   double r;
//	   double a[3], t;
// 
//	   if(v<threshold[0]) return 0;
//	   if(v>threshold[1]) return 200;
// 
//	   GetThermCoefs(ntherm, a);
//	   r = Rref * (v / (Vref-v));
//	   t = SteinhartHart(a,r);
//	   return t;
// }
//***************************************************************************
unsigned int checksum_nmea(char *strPtr) {
	// code --	http://www.mculabs.com/snippets/nmea_checksum.html
	// validate -- http://www.hhhh.org/wiml/proj/nmeaxor.html
	int p;
	char c;
	byte chksum;

	c = strPtr[0]; // get first chr
	chksum = c;
	p = 1;
	while ( c != 0x00 ){
		c = strPtr[p]; // get next chr
		if ( c != 0x00 ) { 
			chksum = chksum ^ c; 
		}
		p++;
	}
	return chksum;
}
//*******************************************************************
void EepromDefault(){
	//int i;
	Serial.println("Default eeprom");
	ee.id = EEPROM_ID;
	ee.swcal = default_swcal;
	ee.lwcal = default_lwcal;
	EepromStore();
	EepromRead();
	EepromPrint();
}

//=============================================================================
void EepromStore()
//Determines the size of the structure and stores it in eeprom space
{
	Serial.print("test Store eeprom: eeSize="); Serial.println(eeSize);
	int i;
	// pointer to struct ee
	byte* a = &(ee.id);

	// for each byte in the eeprom structure 
	for(i=0; i < eeSize; i++){
		Serial.print(i); Serial.print(" ");
		EEPROM.write(i, *a );	// store this byte
		a++;
	}
	Serial.println("");
	return;
}

//=============================================================================
void EepromRead()
{
	Serial.print("test Read eeprom: eeSize="); Serial.println(eeSize);
	int i;
	// pointer to struct ee
	byte* a = &(ee.id);
	// for each byte in the eeprom structure 
	for(i=0; i<eeSize; i++)
	{
		Serial.print(i); Serial.print(" ");
		*a = EEPROM.read(i);  // get the byte
		a++;
	}
	Serial.println("");
	return;
}

//===============================================================================
void EepromPrint()
{
//	   int i;
	char str[10];
	Serial.print("test Print eeprom: eeSize="); Serial.println(eeSize);
	Serial.print("	 ID = ");	Serial.print(ee.id);  Serial.println(" ");
	floatToStringSci(str,ee.swcal,3,0);
	Serial.print("	 g SW Cal = "); Serial.println(str);// Serial.println(" microV/(W/m^2)");
	floatToStringSci(str,ee.lwcal,3,0);
	Serial.print("	 G LW Cal = "); Serial.println(str);// Serial.println(" microV/(W/m^2)");
	return;
}

/***************************************************************************************/
unsigned long ElapsedTime (char *ddhhmmss){

	unsigned long Ld,Lh,Lm,Ls, ms;
	char ch[3];
	ddhhmmss[0]='\0';

	// Elapsed time, days, since startup
	ms = millis()-msstart;

	//days
	Ld = ms/86400000;			// number of days
	if(Ld>=0 && Ld < 100){
		ch[2]='\0'; ch[1]=Ld%10+48; ch[0]=Ld/10+48;
	}else{strcpy(ch,"xx");}
	strcat(ddhhmmss,ch);
	strcat(ddhhmmss,".");
//	   Serial.print("days = ");  Serial.print(Ld);	
//	   Serial.print("	 string:");	 Serial.print(ch);
//	   Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);

	//hours
	Lh = (ms - Ld*86400000)/3600000;		// number hours
	if(Lh>=0 && Lh < 100){
		ch[2]='\0'; ch[1]=Lh%10+48; ch[0]=Lh/10+48;
	}else{strcpy(ch,"xx");}
	strcat(ddhhmmss,ch);
//	   Serial.print("hours = ");  Serial.print(Lh);	 
//	   Serial.print("	string = ");  Serial.print(ch);
//	   Serial.print("	ddhhmmss:");  Serial.println(ddhhmmss);

	//min
	Lm = (ms - Ld*86400000 - Lh*3600000)/60000;
	if(Lm>=0 && Lm < 100){
		ch[2]='\0'; ch[1]=Lm%10+48; ch[0]=Lm/10+48;
	}else{strcpy(ch,"xx");}
	strcat(ddhhmmss,ch);
//	   Serial.print("mins = ");  Serial.print(Lm);	
//	   Serial.print("	string = ");  Serial.print(ch);
//	   Serial.print("	ddhhmmss:");  Serial.println(ddhhmmss);

	//sec
	Ls = (ms - Ld*86400000 - Lh*3600000 - Lm*60000)/1000;
	if(Ls>=0 && Ls < 100){
		ch[2]='\0'; ch[1]=Ls%10+48; ch[0]=Ls/10+48;
	}else{strcpy(ch,"xx");}
	strcat(ddhhmmss,ch);
//	   Serial.print("secs = ");  Serial.print(Ls);	
//	   Serial.print("	string = ");  Serial.print(ch);
//	   Serial.print("	ddhhmmss:");  Serial.println(ddhhmmss);

	return ms;
}

//*******************************************************************
char * floatToString(char * outstr, double val, byte precision, byte widthp)
/********
 * Convert a double prec variable to a string of characters.
 * Example	 floatToString(s, 123.456, 2, 8) returns 
 * s = [ e ',' ','1','2','3','.','4','6','\0'] for 8 characters and a NULL
 *********/
{
	// http://forum.arduino.cc/index.php/topic,37391.0.html

	char temp[16];
	byte i;

	// ROUNDING
	double roundingFactor = 0.5;
	unsigned long mult = 1;
	for (i = 0; i < precision; i++)
	{								//			 *
		roundingFactor /= 10.0;		// .5, .05, .005, ...
		mult *= 10;					// 1,  10,	100, ...
	}


	// OUTSTRING
	temp[0]='\0';
	outstr[0]='\0';

	// NEGATIVE NUMBERS
	if(val < 0.0){
		strcpy(outstr,"-\0");
		val = -val;
	}

	// 123.461
	val += roundingFactor;
	strcat(outstr, ltoa(long(val),temp,10));	 //prints the int part
	
	if( precision > 0) {
		strcat(outstr, ".\0"); // print the decimal point

		unsigned long frac;
		unsigned long mult = 1;
		byte padding = precision -1;
		while(precision--)
			mult *=10;

		if(val >= 0)
			frac = (val - long(val)) * mult;
		else
			frac = (long(val)- val ) * mult;

		unsigned long frac1 = frac;
		while(frac1 /= 10)
			padding--;

		while(padding--)
			strcat(outstr,"0\0");

		strcat(outstr,ltoa(frac,temp,10));
	}
	//Serial.print("test2 ");Serial.println(outstr);

	// generate space padding 
	if ((widthp != 0)&&(widthp >= strlen(outstr))){
		byte J=0;
		J = widthp - strlen(outstr);

		for (i=0; i< J; i++) {
			temp[i] = ' ';
		}

		temp[i++] = '\0';
		strcat(temp,outstr);
		strcpy(outstr,temp);
	}

	return outstr;
}
//=======================================================================================
char * floatToStringSci(char * outstr, double val, byte precision, byte widthp)
{
	int n;
	char str[10];
		
	// -inf to .99999..
	if(val < 1) {
		n=0;
		while(val<1){
			val=val*10; n=n-1;
		}
		floatToString(outstr,val,precision,precision);
		outstr=strcat(outstr,"e");
		floatToString(str, (double)n,0,0);
		outstr=strcat(outstr,str);
	}
	// 10 to inf
	else if(val >= 10) {
		n=0;
		while(val>=10){
			val=val/10; n=n+1;
		}
		floatToString(outstr,val,precision,precision);
		outstr=strcat(outstr,"e");
		floatToString(str, (double)n,0,0);
		outstr=strcat(outstr,str);
	}
	// 1 to 9.9999...
	else {
		floatToString(outstr,val,precision,widthp);
	}
	return outstr;
}

//======================================================================================
double GetAdcVolts (unsigned int chan){
	//GetAdcVolts returns a single read of 16 bit adc channel ch.
	double v;
	int16_t adc;
	if(chan>=0 && chan<=3){
		adc=ads0.readADC_SingleEnded(chan);
	}
	else if(chan>=4 && chan<=7){
		adc=ads1.readADC_SingleEnded(chan-4);
	}
	else adc=0;
	v = (double)adc / 5332;
	return v;
} 
//======================================================================================
unsigned int GetAdcSample(unsigned int ch, double *vmean){
	//		okflag = GetAdcSample(ch, *vmean){
	// GetAdcSample takes nsap (=10) readings of the ADC channel ch. Tosses out the min and max
	// voltages and takes the mean of the remainder.

	double v, vi[10],vsum, vmn, vmx;
	unsigned int i, imx, imn, nav;
//	   v = GetAdcVolts(ch);
//	   *vmean=v;
	nav=10;
	vsum=0; 
	vmx=-1e15; 
	vmn=1e15;
	imx=imn=0;
	for(i=0; i<nav; i++){
		v=vi[i]=GetAdcVolts(ch);
		vsum += vi[i];
		if(v<vmn) {
			imn=i; 
			vmn=v;
		}
		if(v>vmx) {
			imx=i; 
			vmx=v;
		}
	}
	*vmean = (vsum-vi[imx]-vi[imn]) / (double)(nav-2);
	return 1;
}
//============================================================	
// unsigned int GetThermCoefs(unsigned int nt, double c[] ){
//	   // Give the correct SHH coefs for the thermnumber
//	   // COMPUTE TEML
// 
//	   double tcal[8][3]={
//		   {
//			   1.025579e-03,   2.397338e-04,   1.542038e-07						   }
//		   ,   // standard ysi
//		   {
//			   1.0108115e-03, 2.4212099e-04,   1.4525424e-07						}
//		   ,
//		   {
//			   1.0138029e-03, 2.4156995e-04,   1.4628056e-07						}
//		   ,
//		   {
//			   1.0101740e-03, 2.4208389e-04,   1.4485814e-07						}
//		   ,
//		   {
//			   1.0137647e-03, 2.4161708e-04,   1.4619775e-07						}
//		   ,
//		   {
//			   1.0136495e-03, 2.4158562e-04,   1.4769608e-07						}
//		   ,
//		   {
//			   1.0116767e-03, 2.4183780e-04,   1.4673176e-07						}
//		   ,
//		   {
//			   1.0077377e-03, 2.4235481e-04,   1.4556543e-07						}
//	   };
// 
//	   if(nt < 0 || nt > 7){
//		   Serial.print("Error in GetThermCoefs() -- bad thermnumber=");
//		   Serial.println(nt,DEC);
//		   c[0]=c[1]=c[2]=0;
//		   return 0;
//	   } 
//	   else {
//		   c[0]=tcal[nt][0]; 
//		   c[1]=tcal[nt][1]; 
//		   c[2]=tcal[nt][2];
//		   return 1;
//	   } 
// }
//==========================================================================
// long Hex2Dec(char *e)
// // Convert a 3-char hex string to a decimal number
// {
//	   unsigned int j;
//	   byte i;
// 
//	   //  Serial.print(e[0],DEC); Serial.print("	 "); 
//	   if(e[0]>=48 && e[0]<=57) i=e[0]-48;
//	   else if(e[0]>=65 && e[0]<=70) i=e[0]-55;
//	   else return MISSING;
//	   //  else {Serial.println(" BAD"); return MISSING;}
//	   j=i*256;
// 
//	   //  Serial.print(e[1],DEC); Serial.print("	 ");	 
//	   if(e[1]>=8 && e[1]<=57) i=e[1]-48;
//	   else if(e[1]>=65 && e[1]<=70) i=e[1]-55;
//	   else return MISSING;
//	   //  else {Serial.println(" BAD"); return MISSING;}
//	   j=j+i*16;
// 
//	   //  Serial.println(e[2],DEC);		
//	   if(e[2]>=48 && e[2]<=57) i=e[2]-48;
//	   else if(e[2]>=65 && e[2]<=70) i=e[2]-55;
//	   //  else {Serial.println(" BAD"); return MISSING;}
//	   else return MISSING;
//	   j=j+i;
// 
//	   return j;
// }
//====================================================================================================
// void	   MeanStdev(double *sum, double *sum2, int N, double missing)
// //  Compute mean and standard deviation from
// //  the count, the sum and the sum of squares.
// {
//	   if( N <= 2 ){
//		   *sum = missing;
//		   *sum2 = missing;
//	   }
//	   else{
//		   *sum /= (double)N;	   // mean value
//		   *sum2 = *sum2/(double)N - (*sum * *sum); // sumsq/N - mean^2
//		   *sum2 = *sum2 * (double)N / (double)(N-1); // (N/N-1) correction
//		   if( *sum2 < 0 ) *sum2 = 0;
//		   else *sum2 = sqrt(*sum2);
//	   }
//	   return;
// }
//==================================================================
// void PrintBytes(char *str){
//	   int i,ic;
//	   i = strlen(str);
//	   Serial.print("PrintBytes: ");
//	   for(ic=0; ic<i; ic++){
//		   Serial.print(str[ic],HEX); 
//		   Serial.print(", ");
//	   }
//	   Serial.println("");
// }
//==================================================================
void PrintProgramID(void)
{
	Serial.println("");
	Serial.print("PROGRAM:");
	Serial.print(PROGRAMNAME);
	Serial.print("	  VERSION:");
	Serial.print(VERSION);
	Serial.print("	 EDIT:");
	Serial.print(EDITDATE);
	Serial.println("");
}
//==================================================================
unsigned	Ref(int K) {
	if (K == ON){
		digitalWrite(REFSW,LOW);
//		   digitalWrite(LED32,HIGH);
		return OK;
	}
	else if (K == OFF){
		digitalWrite(REFSW,HIGH);
//		   digitalWrite(LED32,LOW);
		return OK;
	}
	return NOTOK;
}
//============================================================================
// int sign (float input) {
//	   return ((input < 0.0) ? NEG: POS);
// }
//===========================================================================
// double SteinhartHart(double beta[], double r)
// //input
// // beta is the 3x1 calibration vector where temp is computed by
// //	  xt = beta(1) + beta(2)*xr + beta(3)*xr^3
// //	where
// //	   xr = log(1/R)
// //	   T = 1/xt - 273.15
// // r = measured resistance, ohms
// //
// //output
// // t = calibrated temperature vector, degC
// {
//	   double vx, t2, t;
// 
//	   vx = log(r);
//	   //Serial.print("r="); Serial.print(r,3);
//	   //Serial.print("  vx="); Serial.print(vx,3);
//	   //Serial.print("  beta="); Serial.print(beta
//	   //fitted curve
//	   t2 = beta[0] + beta[1] * vx + beta[2] * vx * vx * vx;
//	   t = 1 / t2 - (double)273.15;
//	   return t;
// }
// 


