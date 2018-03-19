// NOTE DEFINES
#define PROGRAMNAME "logger_v1"
#define VERSION		"1"
#define EDITDATE	"140721"
const byte	EEPROM_ID = 7;

/*
TODO  
1. confirm rs232-2 works to 7017
2.  
 */

/* ======= SERIAL ALLOCATIONS =============
	Serial -- General user i/o, data
	Serial1 -- 2485, adam4520, 01=adam4017
 */

//NOTE INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


//NOTE -- CONSTANTS
// GENERAL
const char OK = 1;
const char NOTOK = 0;
const int MISSING = -999;
const int POS = 1;
const int NEG = -1;

// DIGITAL LINES
const int Dheartbeat = 13;	// uno led on pin 13.
const int SW_VREF = 7;		// BB thermistor reference voltage

const int BB1 = 1;
const int BB2 = 2;

const int STOP = 0;			// motor stop
const int ON = 1;
const int OFF = 0;
		// test menu
const char TEST=-1;
const char RUN=1;
		// TEMPERATURE
const int WARMUPMILLISECS = 100;
const double TMAX = 50;
const double TMIN = -5;

//NOTE -- GLOBAL VARIABLES
	// general use
char	RunMode;
	// 4017 ADC
double	Adc[8];
double	Rref[4]={10000,10000,10000,10000};
unsigned long secs_lapsed;

		//FUNCTIONS
void		Action(char*);
double		BB_TempCal(double, double, double);
void		Configure4017(void);
void		PrintBytes(char *);
void		Read4017 (double*,	char);

const int T2Rx = 2;	// TTL2 Rx
const int T2Tx = 3; // TTL2 Tx
SoftwareSerial a4017(T2Rx,T2Tx);	// software serial port for Uno

// =========================  SETUP ==============================================
void setup() {	 

			// USER Serial setup
	Serial.begin(9600);
			// ENCODER serial setup
	a4017.begin(9600);	// serial 

	pinMode(SW_VREF, OUTPUT);		// BB therm ref on/off
	pinMode(Dheartbeat, OUTPUT);     
	
	// sign on
	Serial.println("PROGRAM START.");
	PrintProgramID();
}

// ============================ LOOP =============================================================================
void loop() {
	int		i=0;		
	unsigned int	b=0;
	char	buf[20]; 
	char	OutStr[100];
	double ddum, fdum;
	unsigned long ldum;
	const int WARMUPMILLISECS = 500;
	const double TMAX = 50;
	const double TMIN = -5;
	

	// TEST MENU
	Serial.setTimeout(5000);
	//Serial.println("Enter 'T' or 't' to go directly to test mode.");
	i=Serial.readBytes(buf,1);
	if( i > 0 ){
		if ( buf[0] == 't' || buf[0] == 'T'){
			RunMode = TEST;
		   Serial.println("TEST mode.");
			while( RunMode == TEST ) {
						// prompt
				Serial.print("> ");
						// Wait for a command
				i = Serial.readBytesUntil(13,buf,11);
				buf[i]='\0';
				if( i > 0 ){
							// G - go to main loop
					if( buf[0] == 'G' || buf[0] == 'g' ) {
						RunMode = RUN;
						Serial.println("Go to RUN mode.");
					}
					else {
						Action(buf);
					}
				}
			}
		} 
	}
	
			// OUTPUT STRING	 
			// header
	strcpy(OutStr,"$Tcal0,\0");
		// BB TEMPS
		// 5ref on, wait for warm up
	digitalWrite(Dheartbeat, HIGH);
	digitalWrite(SW_VREF,LOW);
	delay(WARMUPMILLISECS);
		// read adc
	Read4017(Adc, -1);
		// 5ref off
	digitalWrite(SW_VREF,HIGH);
	
	for(i=0;i<8;i++){
		floatToString(buf,Adc[i],4,7); buf[7]='\0';
		strcat(OutStr,buf);
		if(i<7) strcat(OutStr,",\0");
		else strcat(OutStr,"\0");
	}
			// CHECKSUM AND END
	b = checksum_nmea(OutStr+1);
	Serial.println(b,DEC);
	Serial.print(OutStr); 
	Serial.print("*"); 
	Serial.print(b,HEX); 
	Serial.print("\r\n");
	
//Serial.println("ID	   VREF	   V0      V1       V2      V3     V4       V5     V6   *CHK");
//                $WIROS, 4.9234, 4.9234, 4.9234, 4.9234, 4.9234, 4.9234, 4.9234, 4.9234*47
		// heartbeat off
	digitalWrite(Dheartbeat, LOW);
		// wait for next sample
	delay(950);
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
	char str[50];
	char ix, eok=0;
	double fdum, ddum,Vref,Rref;
	char *stop_at;
	byte i;

	//Serial.print("Action cmd = ");  Serial.println(cmd);
	Rref=10000;
	
	// TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
	if(cmd[0] == '?'){
		PrintProgramID();
		Serial.println("    a	 -- Read 4017			   Ad	  -- 4017 chan d (0-7)");
        Serial.println("    t    -- Temperatures            T     -- Temperature loop");
		Serial.println("  c/C	 -- 4017 configuration");
		Serial.println("  v/V	 -- Program version");
		Serial.println("  g/G	 -- Continue sampling.");
	}
	// READ ADC 4017 CHANNEL
	else if(cmd[0] == 'A' && strlen(cmd) > 1){
		ix = cmd[1]-48;
		Serial.print("Chan "); Serial.println(ix,DEC); 
		Read4017(Adc, ix);
		Serial.print("Chan ");	Serial.print(ix,DEC); Serial.print("  ");  Serial.println(Adc[ix],4);
	}
	// READ	 4017 ADC
	else if(cmd[0] == 'a'){
		Serial.println("ADC read all: ");
		Read4017(Adc, -1);
		for(i=0;i<8;i++){
			Serial.print(i); Serial.print("	 "); Serial.println(Adc[i],4);
		}
	}
	// CONFIGURE THE 4017
	else if(cmd[0] == 'c' || cmd[0] == 'C'){
		Configure4017();
	}


			// TEMPERATURE
	else if(cmd[0] == 't'){
			// Turn on power to the thermistors, wait for warm up
		digitalWrite(SW_VREF,LOW);
		delay(WARMUPMILLISECS);
		// 5ref on, wait for warm up
			// read voltages
		Read4017(Adc, -1);
		delay(100);
			// power off
		digitalWrite(SW_VREF,HIGH);
			// Print results
		Vref=1.99399 * Adc[7];
		Serial.print("VREF = ");Serial.println(Vref,4);
		
		for(i=0;i<=6;i++){
			Serial.print("Therm "); Serial.print(i); Serial.print("	 "); 
			Serial.print(Adc[i],4); Serial.print(" v,	 ");
			ddum=BB_TempCal(Adc[i],Vref,Rref);
			Serial.print(ddum,3); Serial.println(" C");
		}
	}
	else if(cmd[0] == 'T'){
		Serial.println(" N	V0    V1    V2    V3    V4    V5    V6    V7");
		ix=0;
		while(! Serial.available()){
					// index
			Serial.print(ix,DEC); Serial.print("  ");
				// Turn on power to the thermistors, wait for warm up
			digitalWrite(SW_VREF,LOW);
			delay(500);

					// read ADC
			Read4017(Adc,-1);
			delay(100);
					// power off
			digitalWrite(SW_VREF,HIGH);
					// Therms
			for(i=0;i<=7;i++){
						// vt
				Serial.print(Adc[i],4); Serial.print("	");
			}
			Serial.println();
			delay(900);
			ix++;
		}
		Serial.read();
	}

	// testing Serial1
	else if(cmd[0] == 'q' ){
		while(! Serial.available()){
			Serial.println("HIGH");
			digitalWrite(SW_VREF,HIGH);
			delay(5000);
			Serial.println("LOW");
			digitalWrite(SW_VREF,LOW);
			delay(5000);
		}
		Serial.read();
		Serial.println();
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
		Serial.print(cmd); 
		Serial.println(" not recognized.");
	}
	return;
}


//============================================================================
double BB_TempCal(double vt, double Vref, double Rref)
// vt = voltage of thermister circuit, volts
//	 this series will be scrubed of all bad data
// Vref = reference voltage, fixed	volts
// Rref = reference resistor, fixed
// global $missing	$Tabs
{
	double rt, beta[3];
	double y,t2,t;
	
	//Serial.print("BB_TempCal: vt=");Serial.print(vt,3);Serial.print(", Vref=");Serial.print(Vref,3);
	//Serial.print(", Rref=");Serial.println(Rref,1);
	
	if ( vt == MISSING ) return MISSING;
	
	rt = Rref * vt / ( Vref - vt ) ;
	// NO CORRECTION FOR SELF HEATING
	// From ysi spec, swelf heating in still air is 4 mW/C.
	// The thermistors are in highly conductive material inside the 
	// bb, so this number is a maximum.	 It is probably much less.
	// p = vt .* vt ./ rt;
	// Tcorr = p ./ .004;
	// Tcorr = ( vt * vt / rt ) / .004;

	// YSI SUPER PRECISION THERMISTORS
	// STEINHART-HART COMPUTATION
	//-------------------
	// COMPUTED FROM THE YSI46041 TABLE FOR 20-40 C
	// beta = 1.025579e-03, 2.397338e-04, 1.542038e-07
	//--------------------
	// COMPUTED FOR 0--40 C
	// Steinhart-Hart Fit for T from 0.0 to 40.0
	// beta = 1.025579e-03, 2.397338e-04, 1.542038e-07
	//---------------------
	// beta = [1.025579e-03, 2.397338e-04, 1.542038e-07]';	% 20-40C
	// t2 = beta(1) + beta(2) .* vx + beta(3) .* vx .* vx .* vx;
	// T2 = 1 ./ t2 - 273.15;
	beta[0] = 1.0271173e-3;	 beta[1] = 2.394707e-4;	 beta[2] = 1.553299e-7; // old SH coefs.
	if ( rt <= 0 ) return MISSING;
	else {
		y = log( rt );
		t2 = beta[0] + beta[1] * y + beta[2] * y * y * y;
		t = 1 / t2 - 273.15;
		return t;
	}
}


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
//============================================================================
void		Configure4017(void)
// %0101090600
//See adam 4000 manual page 5-4
{
	char cmd1[5];
	char cmd2[12];
	char strin[15];
	byte i, j, ic, chrin,lencmd,rxlen;
	unsigned long millistart, millilapsed; // define input wait time
	
	strcpy(cmd1,"$012");
	strcpy(cmd2,"%0101090600");

	// SEND CMD1
	for(i=0;i<4;i++){a4017.write(cmd1[i]);}
	a4017.write(13); 
	
	delay(10);
	
	// Receive string
	millistart = millis();
	j=0;
	while(j<11){
		if( a4017.available() ){
			chrin=a4017.read();
			if(chrin == 13){break;}
			else{ strin[j++]=chrin; }
		}
		millilapsed=millis()-millistart;
		if ( millilapsed > 700 ) break;
	}
	strin[j]='\0';
	Serial.print("Configuration: ");Serial.println(strin);
	
	Serial.println("Configuration should be	   %0101090600");
	Serial.println("Do you want to set the configuration? (enter y or Y)");
	while ( Serial.available()==0 );
	chrin=Serial.read();
	if ( chrin == 'y' || chrin == 'Y' ){
		Serial.println("Change to default.	Valid reply is !01");
		// SEND CMD2
		for(i=0;i<11;i++){a4017.write(cmd2[i]);}
		a4017.write(13); 
	
		delay(10);
	
		// Receive string
		millistart = millis();
		j=0;
		while(j<11){
			if( a4017.available() ){
				chrin=a4017.read();
				if(chrin == 13){break;}
				else{ strin[j++]=chrin; }
			}
			millilapsed=millis()-millistart;
			if ( millilapsed > 700 ) break;
		}
		strin[j]='\0';
		Serial.print("Reply: ");Serial.println(strin);
	} else {
		Serial.println("Do NOT change configuration.");
	}
}


//*******************************************************************
char * floatToString(char * outstr, double val, byte precision, byte widthp)
{

	// http://forum.arduino.cc/index.php/topic,37391.0.html

	char temp[16];
	byte i;

	// compute the rounding factor and fractional multiplier
	double roundingFactor = 0.5;
	unsigned long mult = 1;
	for (i = 0; i < precision; i++)
	{
		roundingFactor /= 10.0;
		mult *= 10;
	}

	temp[0]='\0';
	outstr[0]='\0';

	if(val < 0.0){
		strcpy(outstr,"-\0");
		val = -val;
	}

	val += roundingFactor;

	strcat(outstr, itoa(int(val),temp,10));	 //prints the int part
	if( precision > 0) {
		strcat(outstr, ".\0"); // print the decimal point

		unsigned long frac;
		unsigned long mult = 1;
		byte padding = precision -1;
		while(precision--)
			mult *=10;

		if(val >= 0)
			frac = (val - int(val)) * mult;
		else
			frac = (int(val)- val ) * mult;

		unsigned long frac1 = frac;
		while(frac1 /= 10)
			padding--;

		while(padding--)
			strcat(outstr,"0\0");

		strcat(outstr,itoa(frac,temp,10));
	}

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
//==========================================================================
unsigned int Hex2Dec(char *e)
	// Convert a 3-char hex string to a decimal number
{
	unsigned int j;
	byte i;
	
	//Serial.println(e);
	
	if(e[0]<58 && e[0]>48) i=e[0]-48;
	else if(e[0]>=65 && e[0]<=70) i=e[0]-55;
	else return MISSING;
	j=i*256;
	
	if(e[1]<58 && e[1]>48) i=e[1]-48;
	else if(e[1]>=65 && e[1]<=70) i=e[1]-55;
	else return MISSING;
	j=j+i*16;

	if(e[2]<58 && e[2]>48) i=e[2]-48;
	else if(e[2]>=65 && e[2]<=70) i=e[2]-55;
	else return MISSING;
	j=j+i;
	
	return j;
}
//==================================================================
void PrintBytes(char *str){
	int i,ic;
	i = strlen(str);
	Serial.print("PrintBytes: ");
	for(ic=0; ic<i; ic++){
		Serial.print(str[ic],HEX); 
		Serial.print(", ");
	}
	Serial.println("");
}
//==================================================================
void PrintProgramID(void)
{
	Serial.print("PROGRAM:");
	Serial.print(PROGRAMNAME);
	Serial.print("	 VERSION:");
	Serial.print(VERSION);
	Serial.print("	EDIT:");
	Serial.print(EDITDATE);
	Serial.println("");
}


//*******************************************************
void Read4017 (double *adc, char chan)
// Issue a command to the 4017, jump to receive, read in the string.
// See Adam 4000 manual page 4-14, 
//		#01<cr> for all channels,	>+7.2111+7.23453+...<cr> 
//			">+02.368+02.393+02.404+02.399+01.685+00.866+00.461+00.237"	  len=57
//		#01x<cr> where x=0-7,		>+1.4567<cr>
//			">+02.394"		len=8
//		adc[7] is a float array of the result of the read.
{
	char strcmd[6];
	char strin[65];
	byte i, j, ic, chrin,lencmd,rxlen;
	unsigned long millistart, millilapsed; // define input wait time
	
	//Serial.print("Read4017: ");	Serial.println(chan,DEC);

	// MAKE COMMAND
	// all channels
	if ( chan > 7 || chan < 0 ) {
		strncpy(strcmd,"#01",3);
		strcmd[3] = '\0';
		lencmd = 3;
		rxlen=59;
	} 
	// single channel
	else {
		// individual channel
		strncpy(strcmd,"#01",3);
		strcmd[3]=chan+48;			
		strcmd[4]='\0';
		lencmd=4;
		rxlen=9;
	}

	// Command sent
	for(i=0;i<lencmd;i++){
		a4017.write(strcmd[i]);
	}
	a4017.write(13); 
	
	delay(10);
	
	// Receive string
	millistart = millis();
	j=0;
	while(j<rxlen){
		if( a4017.available() ){
			chrin=a4017.read();
			if(chrin == 13){break;}
			else{ strin[j++]=chrin; }
		}
		millilapsed=millis()-millistart;
		if ( millilapsed > 700 ) break;
	}
	strin[rxlen]='\0';
	
	ic = strlen(strin);
	//Serial.print(ic); Serial.println(" bytes received.");
	//Serial.println(strin);
	
	if(rxlen==9){
		adc[chan]=atof( strin+1	 );
		//Serial.println(adc[0],2);
	}
	else {
		j=0;
		for(i=1; i<rxlen; i++){
			if(strin[i]=='+'|| strin[i]=='-'){
				adc[j]=atof( strin+i );
				//Serial.print(j);Serial.print("  "); Serial.println(adc[j],2);
				j++;
			}
		}
	}
}




