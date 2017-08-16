
// NOTE ========= DEFINES
#define PROGRAMNAME "rosr_main"
#define VERSION     "21"
#define EDITDATE    "150427"
const byte  EEPROM_ID = 9;


//NOTE ====== INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//Added for the ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads0;  // ad0, u13, construct an ads1115 at address 0x48
Adafruit_ADS1115 ads1(0x49);    // ad1, u16, construct an ads1115 at address 0x49
Adafruit_ADS1115 ads2(0x4A);    // ad2, u14, construct an ads1115 at address 0x4A
//Adafruit_ADS1115 ads3(0x4B);  // ad3, u17, spare, 

//NOTE ====== DIGITAL LINES
const int Serial4Tx = 2;    // Testing KT15 Tx;  TILT TX
const int Serial4Rx = 3;    // Testing KT15 Rx;  TILT RX
const int RAIN = 4;         // RAIN DETECTOR -- Drives the LED
const int DHe1 = 5;         // INPUT Hall Effect Switch 1
const int DHe2 = 6;         // IN Hall Effect Switch 2
const int D2a = 7;          // shutter motor 
const int D2b = 8;          // shutter motor 
const int BB1 = 9;          // bb1 heater on/off
const int BB2 = 10;         // bb2 heater on/off
const int D1a = 11;         // motor forward
const int D1b = 12;         // motor !forward
//const int HBT = 13;           // heartbeat  LED31
const int LED11 = 24;       // Green Scan Drum motor
const int LED12 = 26;       // Red shutter motor
const int LED21 = 28;       // green BB2 heater on
const int LED22 = 30;       // red BB1 heat on
const int LED31 = 32;       // green -- continuous on = power
const int LED32 = 13;       // red -- heartbeat. red 5REF
const int REFSW = 22;       // BB thermistor reference voltage

// ANALOG ASSIGNMENTS
// three 4-chan 16 bit adc, adc#=(unsigned)(x/4), chan#=x%4;
const int ADCMAX = 12;      // =12 normally. maximum number of ADC channels
const int Nbb11 = 0;        // BB1 thermistor 1
const int Nbb12 = 4;        // BB2 thermistor1
const int Nbb21 = 1;        // BB1 thermistor 2
const int Nbb22 = 5;        // BB2 thermistor 2
const int Nkt15 = 2;        // KT15 analog output
const int NRF   = 3;        // 5REF/2
const int Npwr = 6;         // Circuit power temp
const int Nwindow = 7;      // Window thermistor
const int Ntkt15 =  8;      // KT15 body temperature
const int NVin =    9;      // VIN/4 
const int Nrain =  10;      // Rain / 4
const int Nspare1 = 11;     // Spare input 1 

//NOTE ====== CONSTANTS
const char OK = 1;
const char NOTOK = 0;
const int MISSING = -999;
const int POS = 1;
const int NEG = -1;
const int CCW = 1;          // forward direction
const int CW = -1;          // reverse direction
const int STOP = 0;         // motor stop
const int ON = 1;
const int OFF = 0;
const int OPEN = 1;
const int CLOSED = 0;
const char SPACE = ' ';
const char ZERO = '0';

//encoder
const char EncCmd = 0x11;
// test menu
const char TEST=-1;
const char RUN=1;
// TEMPERATURE
const int WARMUPMILLISECS = 50;
const int ADCREADWAIT = 100;
const double TMAX = 50;
const double TMIN = -5;
// SPECTRON TILT
const double C1 = 0.0129;
const double C2 = 0;
const double C3 = -0.0000000003;
const double P1=-1;  // use -1 for a reversed direction
const double P0=0;
const double R1=-1;  // use -1 for a reversed direction
const double R0=0;

// SHUTTER
// CCW = open shutter
const double CCWSHUTTERCURRENTTHRESHOLD = 50;
const unsigned CCWSHUTTERTIMEOUT = 1400;
const unsigned CCWSTOPMILLISECS = 240;
// CW = close
const double CWSHUTTERCURRENTTHRESHOLD = 50;
const unsigned CWSHUTTERTIMEOUT = 2000;
const unsigned CWSTOPMILLISECS = 400;

// mega adc
const int I1a = 0;          // shutter motor
const int I1b = 1;          // shutter motor
const int Arain = 10;       // rain sensor ADC16 channel


//=========================================================================
//NOTE ====== GLOBAL VARIABLES
//=========================================================================
// system time
unsigned long msstart;  //set at start, milliseconds
unsigned long menustart;  


// user i/o
char    RunMode;
int     ScanMotorFlag;
float   EncoderAngle = 0;
int     ShutterState;

// READING THE ADC
unsigned long nsamp;                        // counter for each sample.
double vmean[ADCMAX];                       // adc16 read
unsigned int BbThermIndex[4]={
    0,1,4,5};       // v[i] index for adc channels. {0,1,4,5}; (eeprom)
double Vref;                                //compute from adc16;


// ADC16 limits for BB temperature cal. volts
double threshold[2]={
    1.00,4.00};

// AVERAGES
// BB temps
double Tbb[4]={
    0,0,0,0};   // for the four precision bb therms
double Tbbav[4]={
    0,0,0,0};
double Tbbsd[4]={
    0,0,0,0}; 
int     nbb[4]={
    0,0,0,0};
double T1, T2;
double ssst;

// ENCODER
double enav[4]={
    0,0,0,0};   // for the four views.
double ensd[4]={
    0,0,0,0};
int     nen[4]={
    0,0,0,0};
// TILT
double pitav=0;
double pitsd=0;
double rolav=0;
double rolsd=0;
int npit=0;
int nrol=0;
// POINTANGLE = combine encoder angle and roll. Used in ssst calculation.
double sea_angle;
double e_sea;

// KT RAD
double ktav[4]={
    0,0,0,0};   // for the four views.
double ktsd[4]={
    0,0,0,0};
int     nkt[4]={
    0,0,0,0};


// OTHER TEMPS
double  Tkt=0;  // 4-20 ma KT15 analog output. v[
double  Tktx=0; // KT15 external, case temperature 
double  Twin=0; // window temperature
double  Tpwr=0; // Temp of the 


// RAIN
unsigned long  millisec_rain;   // the clock count to open.  0 if open
char    RainState;
unsigned long secs_lapsed;


//NOTE ====== EEPROM
// Scan angles: abb1, abb2, asky, asea, a1, a2
const float default_abb1 = 280;
const float default_abb2 = 325;
const float default_asky = 45;
const float default_aocean = 135;
const float default_encoderref = 0;
const byte default_Nbb = 30;
const byte default_Nsky = 10;
const byte default_Nocean = 40;
const float default_rain_threshold = .090;
const char default_shutter = 1;
const unsigned long default_rainsecs = 600;
const char default_bbheat[2]={
    OFF,ON};
// THERMISTOR S/N, 0=>std YSI cal. (eeprom)
const unsigned int default_ntherm[4]={
    1,2,3,4};
//const char default_bbheater=BB2;
const double    default_Rref[7]={
    10000,10000,10000,10000,10000,10000,10000}; // BB11, BB12, BB21, BB22, Tktx, Twin, Tpwr
// See rmrtools/MakeRadTempTables.m to compute these numbers.
const double default_pt2b[4] = {
    -1.12206540049944e-08,6.5886824242969e-05,0.0117168998631263,0.64265621081911};
const double default_pb2t[4] = {
    11.3632032970786,-52.9876174795602,137.697099404017,-69.5189906138133};
const double default_Acorr=1;
const double default_Offset=0;
const double default_ebb=1;

const float SCAN_TOLERANCE = .1;


struct eeprom {
    byte id, Nbb, Nsky, Nocean, ShutterFlag;
    float abb1, abb2, aocean, asky, encoderref, rain_threshold, ScanTolerance;
    unsigned long rainsecs;
    char bbheat[2];
    unsigned int ntherm[4];  // bb11, bb12, bb21, bb22
    //char bbheater;;     
    double Rref[7];
    double pt2b[4];
    double pb2t[4];
    double Acorr, Offset,ebb;
};
struct eeprom ee;
int eeSize;

int isample, iangle; // iangle = 0-3 for pointing positions.
float pointangle[4];
byte nsample[4];
byte ianglemax=3;  // max = 1 during rain or 3 when open
byte kavflag; // signals when to compute averages and ssst.
int istart; // set to 1 at startup so we can go to a menu.

//NOTE ====== FUNCTIONS
// i/o
void        Action(char*);
double      BB_TempCal(double, double, double);
void        BBHeater(int, int);
void        CheckRain(double *, unsigned long *);
unsigned int        checksum_nmea(char *);
unsigned long ElapsedTime (char *);
float       DiffAngle(float, float);
void        EepromDefault(void);
void        EepromStore(void);
void        EepromRead(void);
void        EepromPrint(void);
char *      floatToString(char *, double, byte, byte);
double      GetAdcVolts (unsigned int );   // ads adc
unsigned int GetAdcSample(unsigned int ch, double *vmean);   // ads adc
double      GetEmis( double vsea, double missing);
unsigned int GetThermCoefs(unsigned int, double *coef );   // ads adc
double      GetMotorCurrent();
double      GetRadFromTemp (double);
double      GetTempFromRad (double);
long        Hex2Dec(char *);
double      KTanalogTemp(double);
void        MeanStdev(double *, double *, int, double );
void        PrintProgramID(void);
void        PrintBytes(char *);
float       PointScanDrum(float);
float       ReadEncoder (float);
void        ReadKT15(double *irrad, double *irtemp);
void        ReadTilt(double*, double*);
unsigned    Ref(int);
int         ScanMotor(int);
int         ShutterMotor(int);
void        ShutterOpen(void);
void        ShutterClose(void);
int         sign (float);
double      SteinhartHart(double beta[], double r);
double      ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm);

//============================================================================
void setup() {   
    // USER Serial setup
    Serial.begin(9600);
    // ENCODER serial setup
    Serial1.begin(9600);    // serial 
    // TILT
    Serial2.begin(19200);   // serial 
    // KT15
    Serial3.begin(9600);    // serial kt15

    // BLACK BODIES
    pinMode(REFSW, OUTPUT);     // BB therm ref on/off
    pinMode(BB1, OUTPUT);           // BB1 heater on/off
    pinMode(BB2, OUTPUT);           // BB2 heater on/off

    digitalWrite(BB1, LOW);     // BB1 heater start off;
    digitalWrite(BB2, LOW);     // BB2 heater start off;

    // sign on
    PrintProgramID();

    // Set system clock
    msstart=millis();

    // 5REF START OFF
    Ref(OFF);

    // MISC
    //pinMode(HBT, OUTPUT); // WDT heartbeat.
    pinMode(LED11, OUTPUT);
    pinMode(LED12, OUTPUT);
    pinMode(LED21, OUTPUT);
    pinMode(LED22, OUTPUT);
    pinMode(LED31, OUTPUT);  // HBT
    pinMode(LED32, OUTPUT);

    pinMode(RAIN, INPUT); // LED OUTPUT

        // MOTORS
    pinMode(D1a, OUTPUT);           // Drum motor
    pinMode(D1b, OUTPUT);           // Drum motor
    pinMode(D2a, OUTPUT);           // Shutter motor (in4)
    pinMode(D2b, OUTPUT);           // Shutter motor (in3)
    //     ScanMotor(STOP);             // start with motor stopped

    // HE SWITCHES
    pinMode(DHe1, INPUT);           // HE#1, 0 => shutter is open
    pinMode(DHe2, INPUT);           // HE#2, 0 => shutter is closed

    // set mem for eeprom
    eeSize=sizeof(struct eeprom);
    //  Serial.print("Check EEPROM.  ");
    EepromRead();
    //Serial.print("EEPROM_ID = "); Serial.println(EEPROM_ID);
    //Serial.print("ee.id = "); Serial.println(ee.id);
    if(ee.id != EEPROM_ID ) {
        Serial.println("ID NO  match => use default.");
        EepromDefault();
    };// else Serial.println("ID MATCH.");


    pointangle[0]=ee.abb1; 
    pointangle[1]=ee.abb2;
    pointangle[2]=ee.asky;   
    pointangle[3]=ee.aocean;
    nsample[0]=nsample[1]=ee.Nbb; 
    nsample[2]=ee.Nsky;  
    nsample[3]=ee.Nocean;

    // ADS1115 ADC
    //     Serial.println("Getting single-ended readings from AIN0..3");
    //     Serial.println("ADC Range: +/- 6.144V (1 bit = .15mV)");
    ads0.begin();
    ads0.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V
    //  ads0.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
    //  ads0.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
    //  ads0.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
    //  ads0.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
    //  ads0.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V 
    ads1.begin();
    ads1.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V
    ads2.begin();
    ads2.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V


    //  POINT THE ENCODER TO BB1
    //EncoderAngle = PointScanDrum(ee.abb2);
    iangle=0;  // bb1 position
    isample=3; //
    ianglemax=1;   // open=3, rain=1;
    kavflag=0;      // when =1 a WIRAV record is generated.


    //SHUTTER IS CLOSED
    if( digitalRead(DHe1)==0 && digitalRead(DHe2)==1) {
        //Serial.println("SHUTTER IS OPEN");
        ShutterState=OPEN;
        digitalWrite(LED12,LOW);
    } 
    else    if( digitalRead(DHe1)==1 && digitalRead(DHe2)==0) {
        //Serial.println("SHUTTER IS CLOSED");
        ShutterState=CLOSED;
        digitalWrite(LED12,HIGH);
    } 
    else {
        Serial.println("SHUTTER POSITION UNDETERMINED");
        digitalWrite(LED12,LOW);
    }

    // Drum motor setup;
    ScanMotor(STOP);                // start with motor off; Set ScanMotorFlag.
    // Shutter
    ShutterMotor(STOP);

    istart=1;
}

//=============================================================================
void loop() {
    int     i=0;        
    unsigned int    b=0;
    char    buf[20]; 
    char    OutStr[1024], AvStr[512], SsstStr[512];
    double ddum, fdum;
    unsigned long ldum;


    Serial.setTimeout(1000); // 1 sec wait for a key entry
    //test Serial.println("Enter 'T' or 't' to go directly to test mode.");
    i=Serial.readBytes(buf,1);  // normal operation -- wait for key entry
    if( i > 0 || istart==1){
        if ( buf[0] == 't' || buf[0] == 'T' || istart==1){
            menustart=millis();
            istart=0;   // normal operation
            RunMode = TEST;
            Serial.setTimeout(10000); // 10 sec wait for a key entry
            //Serial.println("TEST mode.");
            while( RunMode == TEST ) {
                if( millis()-menustart > 60000){
                    break;
                }
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
                        Serial.println("");
                        Action(buf);
                        menustart=millis();
                    }
                }
            }
        }
    }
    BBHeater(BB1,ee.bbheat[0]);
    BBHeater(BB2,ee.bbheat[1]);

    // Point drum
    EncoderAngle = PointScanDrum(pointangle[iangle]);
    // OUTPUT STRING     
    //$WIROS,drum,adc0,adc1,adc2,adc3,adc4,adc5,adc6,adc7,adc8,adc9,adc10,adc11,vref,t11,t12,t21,t22,
    //pitch,roll,
    // header
    strcpy(OutStr,"$WIROS,\0");

    // Elapsed time, days, since startup
    ElapsedTime(buf);
    strcat(OutStr,buf);// end of AvStr strcat(AvStr,",");
    strcat(OutStr,",\0");   

    // ReadEncoder
    //EncoderAngle=ReadEncoder(ee.encoderref);
    floatToString(buf,EncoderAngle,2,6);
    strcat(OutStr,buf);  
    strcat(OutStr,",\0");
    // Accumulate encoder
    if(EncoderAngle > 0){
        enav[iangle]+=EncoderAngle;
        ensd[iangle]+=EncoderAngle*EncoderAngle;
        nen[iangle]++;
    }

    // RADIOMETER
    ReadKT15(&ddum, &fdum);
    floatToString(buf,ddum,0,7);
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    floatToString(buf,fdum,2,5);
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    // Accumulate kt 
    if(ddum > 0){
        ktav[iangle]+=ddum;
        ktsd[iangle]+=ddum*ddum;
        nkt[iangle]++;
    }
    // ADC16 READ
    // REF POWER ON
    Ref(ON);
    delay(WARMUPMILLISECS);
    // ALL ADC CHANS
    for(i=0; i<ADCMAX; i++){
        GetAdcSample(i, (vmean+i));
        floatToString(buf,vmean[i],3,5);
        strcat(OutStr,buf);  
        strcat(OutStr,",");
    }
    // REF POWER OFF
    Ref(OFF);
    // COMPUTE VREF
    Vref=vmean[3]*2;
    floatToString(buf,Vref,3,5);
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    // BBTEMP
    for(i=0;i<4;i++){
        Tbb[i]=ThermistorTemp(vmean[BbThermIndex[i]], Vref, ee.Rref[i], ee.ntherm[i]);
        if(Tbb[i] < TMIN || Tbb[i] > TMAX){
            strcat(OutStr,"-999,");
        } 
        else { 
            floatToString(buf,Tbb[i],2,5);
            strcat(OutStr,buf);  
            strcat(OutStr,",");
            Tbbav[i]+=Tbb[i];
            Tbbsd[i]+=Tbb[i]*Tbb[i];
            nbb[i]++;
        }
    }
    // TILT
    ReadTilt(&ddum, &fdum);
    // Pitch
    if( ddum==MISSING ) {
        strcat(OutStr,"-999,");
    } 
    else {
        floatToString(buf,ddum,1,4);
        strcat(OutStr,buf);  
        strcat(OutStr,",");
        npit++;
        pitav+=ddum;
        pitsd+=ddum*ddum;
    }
    //Roll
    if( fdum==MISSING ) {
        strcat(OutStr,"-999,");
    } 
    else {
        floatToString(buf,fdum,1,4);
        strcat(OutStr,buf);  
        strcat(OutStr,",");
        nrol++;
        rolav+=fdum; //v21 add +
        rolsd+=fdum*fdum; //v21 add +
    }
    // HOUSEKEEPING
    Tkt = KTanalogTemp(vmean[2]);
    floatToString(buf,Tkt,1,4);  
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    Tktx = ThermistorTemp(vmean[11], Vref, ee.Rref[4], 0); // 0 = standard steinhart-hart coefs
    floatToString(buf,Tktx,1,4);  
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    Twin = ThermistorTemp(vmean[7], Vref, ee.Rref[5], 0); // 0 = standard steinhart-hart coefs
    floatToString(buf,Twin,1,4);  
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    Tpwr = ThermistorTemp(vmean[6], Vref, ee.Rref[6], 0); // 0 = standard steinhart-hart coefs
    floatToString(buf,Tpwr,1,4);  
    strcat(OutStr,buf);  
    strcat(OutStr,",");
    ddum = vmean[9]*4; // Vin
    floatToString(buf,ddum,1,4);  
    strcat(OutStr,buf);  
    strcat(OutStr,",");

    //RAIN STATE
    CheckRain(&ddum, &ldum);
    if(ddum == MISSING) {
        strcat(OutStr,"-999,-999");
    } 
    else {
        floatToString(buf,ddum,1,4);
        strcat(OutStr,buf);  
        strcat(OutStr,",");
        floatToString(buf,(double)ldum,0,3);
        strcat(OutStr,buf);  // End of OutStr.  strcat(OutStr,",");
    }
    // SHUTTER ACTION
    if( RainState == 1 ) { 
        ShutterClose();
    } 
    else {
        ShutterOpen();
    }
    // CHECKSUM AND END
    b = checksum_nmea(OutStr+1);
    Serial.print(OutStr); 
    Serial.print("*"); 
    Serial.print(b,HEX); 
    Serial.print("\r\n");
    // SHUTTER OPEN/CLOSE
    // RAIN
    if( RainState == 1 ) {
        ianglemax=1;
        if( iangle > 1 ){
            iangle=0;
            isample=0;
        }
    }
    else ianglemax=3;
    // SWITCH ANGLES
    isample++;
    if(isample >= nsample[iangle]){
        iangle++;
        if( iangle > ianglemax ) {
            iangle=0;
            if( ianglemax == 3 ){
                kavflag=1;
            }
        }
        // PRINT HEADER -- used during test
        //Serial.println("ID     drum    KT     Tkt   adc0  adc1  adc2  adc3  adc4  adc5  adc6  adc7  adc8  adc9  adc10  adc11 vref  t11   t12   t21   t22    pit  rol  Tkt  Tktx Twin Tpwr Vin vrain sec*chk");
        //              $WIROS,280.02, 174545,24.86,2.537,2.541,0.414,2.472,1.854,1.853,1.776,2.484,4.096,3.634,-0.000,2.487,4.944,23.66,23.70,38.22,38.21,-2.8,-1.1,-0.0,  0,*50

        // PRINT AVERAGES
        if(kavflag==1){
            strcpy(AvStr,"$WIRAV,\0");
            strcpy(SsstStr,"$WIRST,\0");

            // Elapsed time, days, since startup
            ElapsedTime(buf);
            strcat(AvStr,buf);  
            strcat(AvStr,",");
            strcat(SsstStr,buf); 
            strcat(SsstStr,",");


            // ENCODER VIEWS
            for(i=0; i<4; i++){
                MeanStdev( (enav+i), (ensd+i), nen[i], MISSING);
                floatToString(buf,enav[i],2,6);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
                strcat(SsstStr,buf);     
                strcat(SsstStr,",");
                //floatToString(buf,ensd[i],1,4);
                //strcat(AvStr,buf);   strcat(AvStr,",");
            }

            // KT
            for(i=0; i<4; i++){
                MeanStdev( (ktav+i), (ktsd+i), nkt[i], MISSING);
                //mean
                floatToString(buf,ktav[i],0,6);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
                strcat(SsstStr,buf);     
                strcat(SsstStr,",");
                //stdev
                floatToString(buf,ktsd[i],1,4);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
            }
            // BB TEMPS
            for(i=0; i<4; i++){
                MeanStdev( (Tbbav+i), (Tbbsd+i), nbb[i], MISSING);
                floatToString(buf,Tbbav[i],2,5);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
                floatToString(buf,Tbbsd[i],2,4);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
            }
            // FINAL BB TEMPERATURES FOR SST COMPUTATION
            T1=0; 
            i=0;
            //Serial.print("Tbbav[0] = "); Serial.println(Tbbav[0],2);
            //Serial.print("Tbbav[1] = "); Serial.println(Tbbav[1],2);
            if(Tbbav[0] > -20){ 
                T1=Tbbav[0]; 
                i=1;
            }
            if(Tbbav[1]>-20){
                T1+=Tbbav[1]; 
                i++;
            }
            if(i==0){
                T1=MISSING; 
                strcat(AvStr,"-999,");
                strcat(SsstStr,"-999,");
            } 
            else {
                T1=T1/(double)i;
                floatToString(buf,T1,3,6);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
                strcat(SsstStr,buf); 
                strcat(SsstStr,",");
            }
            //Serial.print("T1 = "); Serial.println(T1,2);

            T2=0; 
            i=0;
            if(Tbbav[2] > -20){ 
                T2=Tbbav[2]; 
                i=1;
            }
            if(Tbbav[3] > -20){ 
                T2+=Tbbav[3]; 
                i++;
            }
            if(i==0){
                T2=MISSING; 
                strcat(AvStr,"-999,");
                strcat(SsstStr,"-999,");
            } 
            else {
                T2=T2/(double)i;
                floatToString(buf,T2,3,6);
                strcat(AvStr,buf);   
                strcat(AvStr,",");
                strcat(SsstStr,buf); 
                strcat(SsstStr,",");
            }
            //Serial.print("T2 = "); Serial.println(T2,2);


            // PITCH & ROLL
            MeanStdev( &pitav, &pitsd, npit, MISSING);
            floatToString(buf,pitav,2,5);
            strcat(AvStr,buf);   
            strcat(AvStr,",");
            floatToString(buf,pitsd,2,5);
            strcat(AvStr,buf);   
            strcat(AvStr,",");
            //Serial.print("Mean pitch = "); Serial.println(pitav,2);
            //Serial.print("Stdev pitch = "); Serial.println(pitsd,2);

            MeanStdev( &rolav, &rolsd, nrol, MISSING);
            floatToString(buf,rolav,2,5);
            strcat(AvStr,buf);   
            strcat(AvStr,",");
            floatToString(buf,rolsd,2,5);
            strcat(AvStr,buf);   
            strcat(AvStr,",");
            //Serial.print("Mean roll = "); Serial.println(rolav,2);
            //Serial.print("Stdev roll = "); Serial.println(rolsd,2);

            // SEA ANGLE 
            sea_angle = enav[3]-rolav;
            floatToString(buf,sea_angle,1,5);
            strcat(AvStr,buf);   
            strcat(AvStr,",");
            strcat(SsstStr,buf); 
            strcat(SsstStr,",");
            //Serial.print("sea angle = "); Serial.println(sea_angle,2);

            // EMISSIVITY
            e_sea=GetEmis(sea_angle,MISSING);
            floatToString(buf,e_sea,6,8);
            strcat(AvStr,buf); //  strcat(AvStr,",");
            strcat(SsstStr,buf); 
            strcat(SsstStr,",");

            //ssst = ComputeSSST(T1,T2,ktav[0],ktav[1],ktav[2],ktav[3],e_sea,ebb,Acorr,Offset,MISSING)
            ssst = ComputeSSST(T1,T2,ktav[0],ktav[1],ktav[2],ktav[3],e_sea,ee.ebb,ee.Acorr,ee.Offset,MISSING);
            floatToString(buf,ssst,2,5);
            strcat(SsstStr,buf); //strcat(SsstStr,",");

            // CHECKSUM AND END
            // AvStr -- averages
            b = checksum_nmea(AvStr+1);
            Serial.print(AvStr); 
            Serial.print("*"); 
            Serial.print(b,HEX); 
            Serial.print("\r\n");
            // SsstStr 
            b = checksum_nmea(SsstStr+1);
            Serial.print(SsstStr); 
            Serial.print("*"); 
            Serial.print(b,HEX); 
            Serial.print("\r\n");

            // RESET FOR NEXT CYCLE
            for(i=0; i<4; i++) {
                // encoder -- 4 views
                enav[i]=ensd[i]=0;
                nen[i]=0;
                // kt15 -- 4 views
                ktav[i]=ktsd[i]=0;
                nkt[i]=0;
                // bb therms -- 4 each
                Tbbav[i]=Tbbsd[i]=0;
                nbb[i]=0;
            }
            // tilt sensor
            pitav = pitsd=0; 
            npit=0;
            rolav = rolsd = 0;  
            nrol=0;


            kavflag=0;
        }
        isample=0;
    }
}
// =============== END OF LOOP =======================================


//*****************************************************************
void    Action(char *cmd)
{
    //  Read message and take action.
    // Create an output packet in out_buffer and send;
    // input:
    // in_buffer = pointer to a message string
    // in_fromlist = char string, e.g. "121115"
    //  meaning the message came from 12 in the route 15->11->12
    char str[50], str1[10];
    char ix, yesno[4], eok=0;
    double fdum, ddum, av1, av2, fsum1, fsq1, fsum2, fsq2;
    unsigned int nsum;
    char *stop_at;
    byte i,ib;
    unsigned long Ldum;
    int n,n1;

    //Serial.print("Action cmd = ");  Serial.println(cmd);


    // TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
    if(cmd[0] == '?'){
        PrintProgramID();
        Serial.println("------- EEPROM -----------------------------------");
        Serial.println("E       -- show eeprom ");
        Serial.println("Ecfff.f -- BB1 point angle          ECfff.f -- BB2 point angle");
        Serial.println("EHnn    -- set bb1 and bb2 heater, 0/1 = off,on.  Standard=01"); 
        Serial.println("Epfff.f -- SKY point angle          EPfff.f -- OCEAN point angle");
        Serial.println("EBnn    -- Black body sample count  EMn     -- Shutter motor on=1, disable=0");
        Serial.println("EUnn    -- SKY sample count         ETnn    -- OCEAN (target) count");
        Serial.println("EDfff.f -- Drum zero ref "); 
        Serial.println("ERfff.f -- Rain threshold volts     Ernn    -- Shutter open delay, nn secs");
        Serial.println("");
        Serial.println("Egfff.f -- Cal slope Acorr          EGfff.f -- Cal offset, degC");
        Serial.println("EAnff.f -- Ref R[n] = fff.f ohms    Ehfff.f -- BB Emissivity");
        Serial.println("------- FUNCTIONS -----------------------------------");
        Serial.println("an -- ADC Chan n                    A      -- ADC all loop");
        Serial.println("bn -- BB n heater OFF               Bn     -- BB n heater ON");
        Serial.println("c  -- 5REF OFF                      C      -- 5REF ON");
        Serial.println("d[ff.f] -- Point to angle f.f,      Omit f.f for current position");
        Serial.println("Dff.f   -- Set the encoder to ff.f");
        Serial.println("fo   -- Shutter Open (CCW)          fc    -- Shutter Close (CW)");
        Serial.println("F    -- Shutter 20x or keystroke");
        Serial.println("h    -- HE switches                 H     -- HE loop");
        Serial.println("k    -- KT15 RAD                    K     -- KT15 loop");
        Serial.println("l    -- Send KT15 command ");
        Serial.println("m    -- Drum motor CCW              M     -- Drum motor CW");
        Serial.println("p    -- pitch/roll single           P     -- pitch/roll loop");
        Serial.println("r    -- Rain check single           R     -- Rain check loop");
        Serial.println("t    -- Read system clock           T     -- Set default f.p. day");
        Serial.println("sn   -- LED test:  1=Heartbeat   2=Drum   3=Shutter/Rain");
        Serial.println("wff.f-- GetEmis(ff.f,MISSING)");
        Serial.println("Wff.f-- GetRadFromTemp(ff.f) --> GetTempFromRad");
        Serial.println("x    -- Compute SSST");
        Serial.println("v/V  -- Program version");
        Serial.println("g/G  -- Continue sampling.");
    }

    // READ ADC CHANNEL
    else if(cmd[0] == 'a' && strlen(cmd) > 1){
        ix = cmd[1]-48;
        Serial.print("Chan "); 
        Serial.println(ix,DEC);
        if(ix < 0 || ix > 15){
            Serial.println("Error");
        } 
        else { 
            GetAdcSample(ix, (vmean+ix));
            Serial.print("Chan ");  
            Serial.print(ix,DEC); 
            Serial.print("  ");  
            Serial.println(vmean[ix],4);
        }
    }
    // ADC LOOP
    else if(cmd[0] == 'A'){
        Serial.println("");
        //    v0    v1    v2    v3    v4    v5    v6    v7    v8    v9    v10   v11  T11(0) T12(1) T21(4) T22(5) TKT(2) TKX(11) TWI(7) TPW(6) VIN(9) VREF(3) VRA(10)
        // 2.630 2.634 0.409 2.473 2.632 2.630 1.812 2.575 4.096 3.664 -0.000 2.464  21.85  21.90  21.93  21.93  52.31  25.19   22.97  39.11  14.7   4.946  -0.0
        Serial.println("    v0    v1    v2    v3    v4    v5    v6    v7    v8    v9    v10   v11  T11(0) T12(1) T21(4) T22(5) TKT(2) TKX(11) TWI(7) TPW(6) VIN(9) VREF(3) VRA(10)");
        while( !Serial.available()  ){
            for(ix=0; ix<ADCMAX; ix++){
                GetAdcSample(ix, (vmean+ix));
                Serial.print(" ");   
                Serial.print(vmean[ix],3);
            }
            Vref=vmean[3]*2;
            for(i=0; i<4; i++){
                Tbb[i]=ThermistorTemp(vmean[BbThermIndex[i]], Vref, ee.Rref[i], ee.ntherm[i]);
                Serial.print("  ");  
                Serial.print(Tbb[i],2);
            }
            //         0      1    2        3      4   5     6   7     8      9    10      11
            // Rref =  BB11  BB12  BB21     BB22  Tktx Twin Tpwr
            // vmean=  BB11  BB12  KTanalog Ref/2 BB21 BB22 Tpwr Twin Tkt15  Vin/4 Vrain/2 Tktx 
            // KT15 4-20
            Tkt = KTanalogTemp(vmean[2]);
            Serial.print("  ");  
            Serial.print(Tkt,2);
            // KT15 case thermistor
            Tktx = ThermistorTemp(vmean[11], Vref, ee.Rref[4], 0); // 0 = standard steinhart-hart coefs
            Serial.print("  ");  
            Serial.print(Tktx,2);
            // window temp
            Twin = ThermistorTemp(vmean[7], Vref, ee.Rref[5], 0); // 0 = standard steinhart-hart coefs
            Serial.print("   ");     
            Serial.print(Twin,2);
            // temp on the UWR power
            Tpwr = ThermistorTemp(vmean[6], Vref, ee.Rref[6], 0); // 0 = standard steinhart-hart coefs
            Serial.print("  ");  
            Serial.print(Tpwr,2);
            // VIN
            ddum = vmean[9]*4; // 0 = standard steinhart-hart coefs
            Serial.print("  ");  
            Serial.print(ddum,1);
            // VREF
            Serial.print("   ");     
            Serial.print(Vref,3);
            // VRAIN
            ddum = vmean[10]*4;
            Serial.print("  ");  
            Serial.print(ddum,1);
            Serial.println("");
            delay(500);
        }
    }

    // BB HEATER ON
    else if(cmd[0] == 'B' && strlen(cmd) > 1){
        ix = cmd[1]-48;
        Serial.print("BB"); 
        Serial.print(ix,DEC); 
        Serial.println(" ON.");
        if(ix < 1 || ix > 2){
            Serial.println("Error");
        } 
        else { 
            if(ix==1) BBHeater(BB1,ON);
            else if(ix==2) BBHeater(BB2,ON);
        }
    }
    // BB HEATER OFF
    else if(cmd[0] == 'b' && strlen(cmd) > 1){
        ix = cmd[1]-48;
        Serial.print("BB"); 
        Serial.print(ix,DEC); 
        Serial.println(" OFF.");
        if(ix < 1 || ix > 2){
            Serial.println("Error");
        } 
        else { 
            if(ix==1) BBHeater(BB1,OFF);
            else if(ix==2) BBHeater(BB2,OFF);
        }
    }

    // 5REF TOGGLE  ON/OFF
    else if(cmd[0] == 'C' ){
        Serial.println("5REF ON");
        Ref(ON);
    }
    else if(cmd[0] == 'c' ){
        Serial.println("5REF OFF");
        Ref(OFF);
    }
    // SHUTTER MOTOR
    else if(cmd[0] == 'f' && cmd[1] == 'o'){ 
        ShutterOpen(); 
    }
    else if(cmd[0] == 'f' && cmd[1] == 'c'){ 
        ShutterClose(); 
    }
    else if(cmd[0] == 'F'){
        ib=20;  
        i=0;
        Serial.print("Shutter cycle ");
        Serial.print(ib);
        Serial.println(" times");
        if (digitalRead(DHe1) == 0 ) {
            ShutterClose();
            delay(1000);
        }
        while(i<ix){
            Serial.print("Cycle ");
            Serial.print(i);
            Serial.print(" of ");
            Serial.println(ib);
            ShutterOpen();  
            delay(10000);
            ShutterClose(); 
            delay(10000);
            if( Serial.available()){
                break;
            }
        }
    }

    // HALL EFFECT (HE) SWITCHES
    else if(cmd[0] == 'h' || cmd[0] == 'H' ){
        Serial.println("CLOSED  OPEN");
        while( !Serial.available()  ){
            Serial.print(digitalRead(DHe1));
            Serial.print("  ");
            Serial.println(digitalRead(DHe2));
            delay(1000);
        }
    }

    // TIME
    else if(cmd[0] == 't'){
        Serial.println("Elapsed time as dd.hhmmss---");
        while(1){
            Ldum = ElapsedTime(str);
            Serial.print("Millisecs elapsed: "); Serial.print(Ldum);
            Serial.print("   "); Serial.println(str);
            if( Serial.available()){
                break;
            }
            delay(2000);

        }
    }

    // POINT SCAN DRUM
    else if(cmd[0] == 'd'){
        EncoderAngle = ReadEncoder(ee.encoderref);
        Serial.print("Drum = "); 
        Serial.print(EncoderAngle,2); 
        Serial.println(" deg");
        if(strlen(cmd) > 1) {
            ddum = atof(cmd+1);
            Serial.print("Request ");  
            Serial.println(ddum,2);
            EncoderAngle = PointScanDrum(ddum);
            Serial.print("Final EncoderAngle = "); 
            Serial.println(EncoderAngle,2);
        }
    }
    else if(cmd[0] == 'D' && strlen(cmd) > 1){
        EncoderAngle = ReadEncoder(ee.encoderref);
        Serial.print("Encoder now = "); 
        Serial.print(EncoderAngle,2); 
        Serial.println(" deg");
        Serial.print("NOTE--SET ENCODER ANGLE TO "); 
        Serial.println((cmd+1));
        strcpy(yesno,"n\0");  // default
        Serial.println("Are you sure? Enter 'y' or 'n':");
        Serial.readBytes(yesno, 1); 
        yesno[1]='\0';
        if( yesno[0] == 'y' ){
            Serial.println("yes"); 
            ddum=atof(cmd+1) - EncoderAngle + ee.encoderref;
            strcpy(str,"ED\0"); 
            floatToString(str1,ddum,1,5); 
            str1[5]='\0';
            strcat(str,str1);   
            strcat(str,",\0");
            Serial.println(str);
            Action(str);
        } 
        else {
            Serial.println("no");
        }       
    }

    // KT15
    else if(cmd[0] == 'k'){
        ReadKT15(&ddum, &fdum);
        Serial.print("rad = ");
        Serial.print(ddum,0);
        Serial.print("    irt = ");
        Serial.println(fdum,3);
    }
    else if(cmd[0] == 'K'){
        fsum1=fsq1=fsum2=fsq2=0;
        nsum=0;
        Serial.println(" N  RAD  IRT");
        ix=0;
        while(! Serial.available()){
            // index
            Serial.print(ix,DEC); 
            Serial.print("  ");
            ReadKT15(&ddum,&fdum);
            fsum1+=ddum; 
            fsum2+=fdum;
            fsq1+=ddum*ddum;  
            fsq2+=fdum*fdum;
            nsum+=1;
            Serial.print(ddum,0); 
            Serial.print("  ");
            Serial.println(fdum,3);
            delay(1000);
            ix++;
        }
        if(nsum>3){
            Serial.print("------");
            Serial.print("  ");
            Serial.println("------");
            av1=fsum1/nsum;  
            av2=fsum2/nsum;
            Serial.print(av1,0);
            Serial.print("  ");
            Serial.println(av2,3);
            ddum=fsq1/(nsum-1)-nsum*av1*av1/(nsum-1);
            ddum=sqrt(ddum);
            fdum=fsq2/(nsum-1)-nsum*av2*av2/(nsum-1);
            fdum=sqrt(fdum);
            Serial.print(ddum,0); 
            Serial.print("  "); 
            Serial.println(fdum,3);
            //ddum=100*ddum/av1;  fdum=100*fdum/av2;
            //Serial.print(ddum,1);Serial.print("  ");Serial.println(fdum,1);
        }
    }

    // KT15 COMMAND
    else if(cmd[0] == 'l'){
        Serial.println("SEND A COMMAND TO KT15");
        Serial.println("Enter a command: ");
        Serial.setTimeout(30000);
        ib=Serial.readBytesUntil('\r',str,49);
        str[ib]='\0';
        Serial.print("You entered: "); 
        Serial.println(str);
        while(Serial3.available()){
            Serial3.read();
        }               // clear the input buffer
        Serial3.println(str);                                   // Send the string with lf
        Serial3.setTimeout(100);                                // set the wait time
        ib=Serial3.readBytesUntil('\n',str,45);                 // capture incoming bytes
        str[ib]='\0';                                           // end the string
        Serial.print("Reply: "); 
        Serial.println(str);
    }

    // EEPROM
    else if( cmd[0] == 'E' || cmd[0]=='e'){
        if( strlen(cmd) <= 1 ){
            EepromPrint();
        }
        else if (strlen(cmd) > 2) {
            eok=0;
            if( cmd[1]=='c'){
                ddum = atof(cmd+2);
                Serial.print("SET BB1 ANGLE = "); 
                Serial.println(ddum,2);
                ee.abb1=ddum;
                eok=1;
            }
            else if( cmd[1]=='C' ){
                ddum = atof(cmd+2);
                Serial.print("SET BB2 ANGLE = "); 
                Serial.println(ddum,2);
                ee.abb2=ddum;
                eok=1;
            }
            else if( cmd[1]=='p' ){
                ddum = atof(cmd+2);
                Serial.print("SET SKY ANGLE = "); 
                Serial.println(ddum,2);
                ee.asky=ddum;
                eok=1;
            }
            else if( cmd[1]=='P' ){
                ddum = atof(cmd+2);
                Serial.print("SET OCEAN ANGLE = "); 
                Serial.println(ddum,2);
                ee.aocean=ddum;
                eok=1;
            }
//             else if( cmd[1]=='P' ){
//                 ddum = atof(cmd+2);
//                 Serial.print("SET OCEAN ANGLE = "); 
//                 Serial.println(ddum,2);
//                 ee.aocean=ddum;
//                 eok=1;
//             }
            else if( cmd[1]=='g' ){
                ddum = atof(cmd+2);
                Serial.print("CAL SLOPE CORRECTION = "); 
                Serial.println(ddum,5);
                ee.Acorr=ddum;
                eok=1;
            }
            else if( cmd[1]=='G' ){
                ddum = atof(cmd+2);
                Serial.print("CAL OFFSET = "); 
                Serial.println(ddum,3);
                ee.Offset=ddum;
                eok=1;
            }
            else if( cmd[1]=='h' ){
                ddum = atof(cmd+2);
                Serial.print("BB EMISSIVITY = "); 
                Serial.println(ddum,6);
                ee.ebb=ddum;
                eok=1;
            }
            else if( cmd[1]=='H' ){
                if( strlen(cmd)==4) {
                    ee.bbheat[0]=cmd[2]-48; 
                    ee.bbheat[1]=cmd[3]-48; 
                } 
                else {
                    ee.bbheat[0] = 0;
                    ee.bbheat[1] = 1;
                }
                eok=1;
            }
            else if( cmd[1]=='A' ){
                ix = cmd[2]-48;
                if (ix<0 || ix>3 ){
                    Serial.println("Error.");
                } 
                else {
                    ddum = atof(cmd+3);
                    Serial.print("Set BB Rref["); 
                    Serial.print(ix,DEC); 
                    Serial.print("] = "); 
                    Serial.print(ddum,1); 
                    Serial.println("ohms.");
                    ee.Rref[ix]=ddum;
                }
            }
            else if( cmd[1]=='R' ){
                ddum = atof(cmd+2);
                Serial.print("SET RAIN THRESHOLD = "); 
                Serial.println(ddum,2);
                ee.rain_threshold=ddum;
                eok=1;
            }
            else if( cmd[1]=='r' ){
                ix = atoi(cmd+2);
                Serial.print("SET RAIN DELAY SECS = ");  
                Serial.println(ix,DEC);
                ee.rainsecs = ix;
                eok=1;
            }
            else if( cmd[1]=='D' || cmd[1]=='d' ){
                ddum = atof(cmd+2);
                Serial.print("SET ENCODER REF = "); 
                Serial.println(ddum,2);
                ee.encoderref=ddum;
                eok=1;
            }
            else if( cmd[1]=='A' || cmd[1]=='a' ){
                ddum = atof(cmd+2);
                Serial.print("SET SCAN TOLERANCE = "); 
                Serial.println(ddum,2);
                ee.ScanTolerance=ddum;
                eok=1;
            }
            else if( cmd[1]=='B' || cmd[1]=='b' ){
                ix = atoi(cmd+2);
                Serial.print("SET BB Count = "); 
                Serial.println(ix);
                ee.Nbb=ix;
                eok=1;
            }
            else if( cmd[1]=='U' || cmd[1]=='u' ){
                ix = atoi(cmd+2);
                Serial.print("SET Sky Count = "); 
                Serial.println(ix);
                ee.Nsky=ix;
                eok=1;
            }
            else if( cmd[1]=='T' || cmd[1]=='t' ){
                ix = atoi(cmd+2);
                Serial.print("SET Ocean Count = "); 
                Serial.println(ix);
                ee.Nocean=ix;
                eok=1;
            }
            else if( cmd[1]=='M' || cmd[1]=='m' ){
                ix = atoi(cmd+2);
                Serial.print("Shutter on/off = "); 
                Serial.println(ix);
                ee.ShutterFlag=ix;
                eok=1;
            }
            if( eok == 1 ) {
                EepromStore();
                EepromPrint();
            }
        }
    }


    // SCAN DRUM MOTOR
    else if(cmd[0] == 'm' || cmd[0] == 'M' ){
        if ( cmd[0] == 'm' ) {
            ScanMotor(CCW);
            Serial.println("Scan drum CCW");
        }
        else {
            ScanMotor(CW);
            Serial.println("Scan drum CW");
        }
        while( !Serial.available() );
        ScanMotor(STOP);
        Serial.println("Scan drum STOP");
    }
    // PITCH/ROLL
    else if(cmd[0] == 'p'){
        ReadTilt(&ddum, &fdum);
        Serial.print("pitch = ");
        Serial.print(ddum,2);
        Serial.print("    roll = ");
        Serial.println(fdum,2);
    }
    else if(cmd[0] == 'P'){
        Serial.println(" N  PITCH  ROLL");
        ix=0;
        while(! Serial.available()){
            // index
            Serial.print(ix,DEC); 
            Serial.print("  ");
            ReadTilt(&ddum,&fdum);
            Serial.print(ddum,2); 
            Serial.print("  ");
            Serial.println(fdum,2);
            delay(1000);
            ix++;
        }
    }

    // RAIN
    else if(cmd[0] == 'r'){
        CheckRain( &ddum, &Ldum );
        Serial.print("Rain volts = ");
        Serial.print(ddum,2);
        Serial.print("   Sec to open = ");  
        Serial.print(Ldum);
        Serial.print("   State = ");  
        Serial.println(RainState,DEC);
    }
    else if(cmd[0] == 'R'){
        Serial.println(" N  STATUS   VOLTS   SECS");
        ix=0;
        while(! Serial.available()){
            // index
            Serial.print(ix,DEC); 
            Serial.print("  ");
            CheckRain( &ddum, &Ldum );
            Serial.print(RainState,DEC); 
            Serial.print("   "); 
            Serial.print(ddum,2); 
            Serial.print("  "); 
            Serial.println( Ldum );
            delay(1000);
            ix++;
        }
    }
    // EMISSIVITY
    else if(cmd[0] == 'w' && strlen(cmd) > 1){
        ddum=atof(cmd+1);
        fdum=GetEmis(ddum,MISSING);
        Serial.print("View angle="); 
        Serial.print(ddum,1);
        Serial.print("   Emissivity="); 
        Serial.println(fdum,6);
    }    
    // PLANCK CALCULATIONS  GetRadFromTemp
    else if(cmd[0] == 'W' && strlen(cmd) > 1){
        ddum=atof(cmd+1);
        fdum= GetRadFromTemp(ddum);
        Serial.print("T   = "); 
        Serial.print(ddum,3);
        Serial.print("   RAD="); 
        Serial.println(fdum,6);
        ddum= GetTempFromRad(fdum);
        Serial.print("RAD = "); 
        Serial.print(fdum,6);
        Serial.print("   T  ="); 
        Serial.println(ddum,3);
    }    
    // COMPUTE SSST
    else if(cmd[0] == 'x'){
        //ssst = ComputeSSST(T1,T2,ktav[0],ktav[1],ktav[2],ktav[3],e_sea,ebb,Acorr,Offset,MISSING
        Serial.println("T1, T2, k1, k2, ksky, ksea, e_sea, ebb, Acorr, Offset");
        Serial.println("20.00, 35.00, 165000, 190000, 150000, 160000, 0.98, 1.00000, 1.000, 0.000");
        ddum = ComputeSSST(20,35,165000,190000,150000,160000,0.98,1,1,0,MISSING);
        Serial.print("SSST = "); 
        Serial.print(ddum,3);
        Serial.println(" degC");
    }
    else if(cmd[0] == 'X'){
        ddum=1234567890;
        floatToString(str, ddum,0,12);
        Serial.print(ddum,6);
        Serial.print("   ");
        Serial.print(str);
    }

    else if(cmd[0] == 's' && strlen(cmd) > 1){
        ix = cmd[1]-48; 
        n=1;
        if(ix==1){
            Serial.println("HEARTBEAT LED FLASH"); 
            n1=LED31;
        }
        else if(ix==2){
            Serial.println("DRUM LED FLASH"); 
            n1=LED11;
        }    
        else if(ix==3){
            Serial.println("SHUTTER LED FLASH");
            n1=LED12;
        }
        else {
            Serial.println("HEARTBEAT LED FLASH"); 
            n1=LED31;
        }
        // flash led
        while(! Serial.available()){
            digitalWrite(n1, HIGH);
            delay(500);
            digitalWrite(n1, LOW);
            delay(500);
        }
    }
    // testing Serial1
    //  else if(cmd[0] == 'q' ){
    //      while(! Serial.available()){
    //          Serial.println("HIGH");
    //          Ref(OFF);
    //          delay(5000);
    //          Serial.println("LOW");
    //          Ref(ON);
    //          delay(5000);
    //      }
    //      Serial.read();
    //      Serial.println();
    //  }
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

//==================================================================
void BBHeater(int j, int k)
/***************
 * input:
 * j = BB1 or BB2
 * k = ON or OFF
    **********/
{
    if(j==BB1){
        if(k==ON) {
            digitalWrite(BB1,HIGH);
            digitalWrite(LED22, HIGH);
        }
        if(k==OFF) {
            digitalWrite(BB1,LOW);
            digitalWrite(LED22, LOW);
        }
    }
    else if(j==BB2){
        if(k==ON) {
            digitalWrite(BB2,HIGH);
            digitalWrite(LED21, HIGH);
        }
        if(k==OFF) {
            digitalWrite(BB2,LOW);
            digitalWrite(LED21, LOW);
        }
    }
    return;
}
//==========================================================================
double ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm){

    double r;
    double a[3], t;

    if(v<threshold[0]) return 0;
    if(v>threshold[1]) return 200;

    GetThermCoefs(ntherm, a);
    r = Rref * (v / (Vref-v));
    t = SteinhartHart(a,r);
    return t;
}
//======================================================================================
void CheckRain(double *v, unsigned long *rainsecs)
// Output
//      v = analog (volts)
//      rainsecs = seconds until shutter opens
// RETURN 1 or 0 for rain mode and shutter control. 
// Global in
//  ee.rain_threshold;
//  ee.rainsecs;
// Global out
//   unsigned long  millisec_rain;  // clock count for opening
//   RainState
{
    int a;

    GetAdcSample(Arain, v);
    *v *= 4;
    //Serial.print("Chan ");  Serial.print(Arain,DEC); Serial.print("  ");  Serial.println(*v,4);

    // RAIN
    if ( *v > ee.rain_threshold ) {
        RainState=1;
        millisec_rain = millis();
        *rainsecs = ee.rainsecs;
    }
    // NO RAIN
    else {
        // SHUTTER IS CLOSED
        if ( RainState == 1 ) {
            secs_lapsed = ( millis() - millisec_rain )/1000;
            // TIME TO OPEN
            if ( secs_lapsed > ee.rainsecs ){
                RainState=0;
                *rainsecs = 0;
            }
            // DRYING TIME
            else {
                RainState=1;
                *rainsecs = ee.rainsecs - secs_lapsed;
            }
        }
        else {
            *rainsecs=0;
            RainState=0;
        }
    }
    return;    
}
//***************************************************************************
unsigned int checksum_nmea(char *strPtr) {
    // code --  http://www.mculabs.com/snippets/nmea_checksum.html
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
// void     Configure4017(void)
// // %0101090600
// //See adam 4000 manual page 5-4
// {
//     char cmd1[5];
//     char cmd2[12];
//     char strin[15];
//     byte i, j, ic, chrin,lencmd,rxlen;
//     unsigned long millistart, millilapsed; // define input wait time
//     
//     strcpy(cmd1,"$012");
//     strcpy(cmd2,"%0101090600");
// 
//     // SEND CMD1
//     for(i=0;i<4;i++){Serial2.write(cmd1[i]);}
//     Serial2.write(13); 
//     
//     delay(10);
//     
//     // Receive string
//     millistart = millis();
//     j=0;
//     while(j<11){
//      if( Serial2.available() ){
//          chrin=Serial2.read();
//          if(chrin == 13){break;}
//          else{ strin[j++]=chrin; }
//      }
//      millilapsed=millis()-millistart;
//      if ( millilapsed > 700 ) break;
//     }
//     strin[j]='\0';
//     Serial.print("Configuration: ");Serial.println(strin);
//  
//  Serial.println("Configuration should be    %0101090600");
//  Serial.println("Do you want to set the configuration? (enter y or Y)");
//  while ( Serial.available()==0 );
//  chrin=Serial.read();
//  if ( chrin == 'y' || chrin == 'Y' ){
//      Serial.println("Change to default.  Valid reply is !01");
//      // SEND CMD2
//      for(i=0;i<11;i++){Serial2.write(cmd2[i]);}
//      Serial2.write(13); 
//  
//      delay(10);
//  
//      // Receive string
//      millistart = millis();
//      j=0;
//      while(j<11){
//          if( Serial2.available() ){
//              chrin=Serial2.read();
//              if(chrin == 13){break;}
//              else{ strin[j++]=chrin; }
//          }
//          millilapsed=millis()-millistart;
//          if ( millilapsed > 700 ) break;
//      }
//      strin[j]='\0';
//      Serial.print("Reply: ");Serial.println(strin);
//  } else {
//      Serial.println("Do NOT change configuration.");
//  }
// }

//========================================================================
float DiffAngle(float a2, float a1) {
    // Compute the smallest angle arc between the a2 and a1.
    float arc;
    arc = a2 - a1;
    if( abs(arc) > 180 ){
        if( sign(arc) > 0 ) arc -= 360;
        else arc += 360;
    }
    return arc;
}

//*******************************************************************
void EepromDefault(){
    int i;
    Serial.println("Initialize eeprom...");
    ee.id = EEPROM_ID;
    ee.abb1 = default_abb1;
    ee.abb2 = default_abb2;
    ee.asky = default_asky;
    ee.aocean = default_aocean;
    ee.encoderref = default_encoderref;
    ee.Nbb = default_Nbb;
    ee.Nsky = default_Nsky;
    ee.Nocean = default_Nocean;
    ee.ShutterFlag = default_shutter;
    ee.rain_threshold = default_rain_threshold;
    ee.rainsecs = default_rainsecs;
    for(i=0; i<7; i++){
        ee.Rref[i]=default_Rref[i];
    }
    ee.bbheat[0]=default_bbheat[0];
    ee.bbheat[1]=default_bbheat[1];
    ee.ntherm[0]=default_ntherm[0];
    ee.ntherm[1]=default_ntherm[1];
    ee.ntherm[2]=default_ntherm[2];
    ee.ntherm[3]=default_ntherm[3];
    //ee.bbheater=default_bbheater;
    ee.pt2b[0]=default_pt2b[0];
    ee.pt2b[1]=default_pt2b[1];
    ee.pt2b[2]=default_pt2b[2];
    ee.pt2b[3]=default_pt2b[3];
    ee.pb2t[0]=default_pb2t[0];
    ee.pb2t[1]=default_pb2t[1];
    ee.pb2t[2]=default_pb2t[2];
    ee.pb2t[3]=default_pb2t[3];
    ee.ScanTolerance=SCAN_TOLERANCE;
    ee.Acorr = default_Acorr;
    ee.Offset = default_Offset;
    ee.ebb = default_ebb;
    EepromStore();
    EepromRead();
    EepromPrint();
}

//=============================================================================
void EepromStore()
//Determines the size of the structure and stores it entirely
//in eeprom space
{
    Serial.println("StoreUee...");
    int i;
    // pointer to struct ee
    //struct eeprom* ptr = &ee;
    byte* a = &(ee.id);

    // for each byte in the eeprom structure 
    for(i=0; i < eeSize; i++){
        EEPROM.write(i, *a );   // store this byte
        a++;
    }
    return;
}

//=============================================================================
void EepromRead()
{
    Serial.println("ReadUee:");
    int i;
    // pointer to struct ee
    byte* a = &(ee.id);
    // for each byte in the eeprom structure 
    for(i=0; i < eeSize; i++)
    {
        *a = EEPROM.read(i);  // get the byte
        a++;
    }
    return;
}

//===============================================================================
void EepromPrint()
{
    int i;
    Serial.println("EepromPrint: ");
    Serial.print("  ID = "); 
    Serial.print(ee.id);    
    Serial.println(" ");
    //Serial.print("    E BBHeater = ");    
    //Serial.println(ee.bbheater,DEC);  xyx 
    Serial.print("  c BB1 angle = ");   
    Serial.print(ee.abb1,2);    
    Serial.println(" deg");
    Serial.print("  C BB2 angle = ");   
    Serial.print(ee.abb2,2);    
    Serial.println(" deg");
    Serial.print("  S sky angle = ");   
    Serial.print(ee.asky,2);    
    Serial.println(" deg");
    Serial.print("  O ocean angle = ");
    Serial.print(ee.aocean,2); 
    Serial.println(" deg");
    Serial.print("  D drum zero ref = ");
    Serial.print(ee.encoderref,2); 
    Serial.println(" deg");
    Serial.print("  A Encoder scan tolerance = "); 
    Serial.print(ee.ScanTolerance,2);  
    Serial.println(" deg");
    Serial.print("  B BB sample count = "); 
    Serial.print(ee.Nbb);  
    Serial.println("");
    Serial.print("  U sky sample count = ");    
    Serial.print(ee.Nsky);  
    Serial.println("");
    Serial.print("  T ocean sample count = ");  
    Serial.print(ee.Nocean);    
    Serial.println("");
    Serial.print("  K Shutter on/off  = ");  
    Serial.print(ee.ShutterFlag,2);  
    Serial.println(" deg");
    Serial.print("  R Rain threshold  = ");  
    Serial.print(ee.rain_threshold,2);   
    Serial.println(" volts");
    Serial.print("  r Rain shutter delay  = ");  
    Serial.print(ee.rainsecs);   
    Serial.println(" secs");

    Serial.print("  g Acorr = "); 
    Serial.print(ee.Acorr,5);  
    Serial.println("");
    Serial.print("  G Offset = "); 
    Serial.print(ee.Offset,4);  
    Serial.println("");
    Serial.print("  - BB Emis = "); 
    Serial.print(ee.ebb,5);  
    Serial.println("");


    Serial.print("  Ref esistors = ");
    for(i=0; i<7; i++){ 
        Serial.print(ee.Rref[i],1); 
        Serial.print("  ");
    }
    Serial.println("");
    Serial.print("  BB heater configuration = "); 
    Serial.print(ee.bbheat[0],DEC); 
    Serial.print("  "); 
    Serial.println(ee.bbheat[1],DEC);
    Serial.print("  BB Therm SNs = "); 
    for(i=0; i<4; i++){ 
        Serial.print(ee.ntherm[i],1); 
        Serial.print("  ");
    }
    Serial.println("");

    Serial.print("  T to B Quadratic = "); 
    for(i=0; i<4; i++){ 
        Serial.print(ee.pt2b[i],8); 
        Serial.print(" ");
    }
    Serial.println("");

    Serial.print("  B to T Quadratic = "); 
    for(i=0; i<4; i++){ 
        Serial.print(ee.pb2t[i],8); 
        Serial.print(" ");
    }
    Serial.println("");
    return;
}


//*******************************************************************
char * floatToString(char * outstr, double val, byte precision, byte widthp)
/********
 * Convert a double prec variable to a string of characters.
 * Example   floatToString(s, 123.456, 2, 8) returns 
 * s = [' ',' ','1','2','3','.','4','6','\0'] for 8 characters and a NULL
 *********/
{
    // http://forum.arduino.cc/index.php/topic,37391.0.html

    char temp[16];
    byte i;

    // ROUNDING
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for (i = 0; i < precision; i++)
    {                               //           *
        roundingFactor /= 10.0;     // .5, .05, .005, ...
        mult *= 10;                 // 1,  10,  100, ...
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
    strcat(outstr, ltoa(long(val),temp,10));     //prints the int part
    
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
//======================================================================================
double GetAdcVolts (unsigned int chan){
    //GetAdcVolts returns a single read of 16 bit adc channel ch.
    double v;
    int16_t adc;
    if(chan>=0 && chan<=3)
        adc=ads0.readADC_SingleEnded(chan);
    else if(chan>=4 && chan<=7)
        adc=ads1.readADC_SingleEnded(chan-4);
    else if(chan>=8 && chan<=11)
        adc=ads2.readADC_SingleEnded(chan-8);
    else adc=0;
    v = (double)adc / 8000;
    return v;
} 


//======================================================================================
unsigned int GetAdcSample(unsigned int ch, double *vmean){
    //      okflag = GetAdcSample(ch, *vmean){
    // GetAdcSample takes nsap (=10) readings of the ADC channel ch. Tosses out the min and max
    // voltages and takes the mean of the remainder.

    double v, vi[10],vsum, vmn, vmx;
    unsigned int i, imx, imn, nav;
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
//=================================================================#
double GetEmis( double vsea, double missing)
/* COMPUTE EMISSIVITY OF THE SEA SURFACE FROM A GIVEN 
 VIEWING ANGLE.  See email from Donlon 040313
 Call: emis = GetEmis($viewangle, $missing)
 
 INPUT
 $viewangle = the isar drum angle toward the sea. From 90-180 deg.
 OUTPUT
 emissivity: viewangle from 90-120 emis=$missing, from 120-140 deg, emai is calculated, from 140-180 deg, emis = 0.99
 20150211 rmr
 */
{
    double e_sea;
    double vx;
    double p[3]={
        -7.00848940533115e-05,       0.00581739911141455,         0.867605108660296        };
    double a1;
    double a2;
    double e1;
    double e2;
    int i;
    double va[21]={
        40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60    };
    double esea[21]={
        0.9893371,0.9889863,0.9885924,0.9881502,0.9876541,0.9870975,0.9864734,0.9857735,
        0.9849886,0.9841085,0.9831214,0.9820145,0.9807728,0.9793796,0.9778162,0.9760611,
        0.9740904,0.9718768,0.9693894,0.9665933,0.9634488    };
    e_sea=missing;
    vx=180-vsea;    // vx = nadir angle

    // NEAR VERTICAL POINTING
    if ( vx >= 0 && vx < 40 ) e_sea = 0.99;

    // NEAR HORIZONTAL, APPROXIMATION 
    // I worked out a matlab fit
    else if( vx > 60 && vx < 90) { 
        e_sea = vx * vx * p[0] + vx * p[1] + p[2];  // quadratic fit
        //p[2] = {-0.00251543428571427,          1.11488077142857}; // linear fit
        //esea = vx * p[0] + p[1];
        // FINAL NUDGE
        //esea += .05;
    }   
    // 40 <= vx <= 60    // donlon email 040317
    else if(vx >=40 && vx <= 60) {
        for(i=1; i<=20; i++ ) {
            if ( va[i] >= vx ) { 
                a2 = va[i];    
                a1 = va[i-1];
                e2 = esea[i];  
                e1 = esea[i-1];
                e_sea = e1 + (vx - a1) * (e2 - e1) / (a2 - a1);
                break;
            }
        }
    }
    return(e_sea);
}
//===========================================================================
double GetMotorCurrent(void) {
    // Read two analog channels and take the difference.
    double vdiff=0;
    int v1=0, v2=0;
    //Serial.println("C");
    v1 = analogRead(I1a);
    v2 = analogRead(I1b);
    vdiff = (double)(v1 - v2);
    //Serial.print(v1);
    //Serial.print("  ");
    //Serial.print(v2);
    //Serial.print("  diff=");
    //Serial.println(vdiff);
    return vdiff;
}
//============================================================  
unsigned int GetThermCoefs(unsigned int nt, double c[] ){
    // Give the correct SHH coefs for the thermnumber
    // COMPUTE TEML

    double tcal[8][3]={
        {
            1.025579e-03,   2.397338e-04,   1.542038e-07                        }
        ,   // standard ysi
        {
            1.0108115e-03, 2.4212099e-04,   1.4525424e-07                        }
        ,
        {
            1.0138029e-03, 2.4156995e-04,   1.4628056e-07                        }
        ,
        {
            1.0101740e-03, 2.4208389e-04,   1.4485814e-07                        }
        ,
        {
            1.0137647e-03, 2.4161708e-04,   1.4619775e-07                        }
        ,
        {
            1.0136495e-03, 2.4158562e-04,   1.4769608e-07                        }
        ,
        {
            1.0116767e-03, 2.4183780e-04,   1.4673176e-07                        }
        ,
        {
            1.0077377e-03, 2.4235481e-04,   1.4556543e-07                        }
    };

    if(nt < 0 || nt > 7){
        Serial.print("Error in GetThermCoefs() -- bad thermnumber=");
        Serial.println(nt,DEC);
        c[0]=c[1]=c[2]=0;
        return 0;
    } 
    else {
        c[0]=tcal[nt][0]; 
        c[1]=tcal[nt][1]; 
        c[2]=tcal[nt][2];
        return 1;
    } 
}


//==========================================================================
long Hex2Dec(char *e)
// Convert a 3-char hex string to a decimal number
{
    unsigned int j;
    byte i;

    //  Serial.print(e[0],DEC); Serial.print("  "); 
    if(e[0]>=48 && e[0]<=57) i=e[0]-48;
    else if(e[0]>=65 && e[0]<=70) i=e[0]-55;
    else return MISSING;
    //  else {Serial.println(" BAD"); return MISSING;}
    j=i*256;

    //  Serial.print(e[1],DEC); Serial.print("  ");     
    if(e[1]>=8 && e[1]<=57) i=e[1]-48;
    else if(e[1]>=65 && e[1]<=70) i=e[1]-55;
    else return MISSING;
    //  else {Serial.println(" BAD"); return MISSING;}
    j=j+i*16;

    //  Serial.println(e[2],DEC);       
    if(e[2]>=48 && e[2]<=57) i=e[2]-48;
    else if(e[2]>=65 && e[2]<=70) i=e[2]-55;
    //  else {Serial.println(" BAD"); return MISSING;}
    else return MISSING;
    j=j+i;

    return j;
}
//====================================================================================================
double      KTanalogTemp(double v) {
    double t;
    double r=50.0;          // loop resistor => .2 to 1.0 v
    double imin=0.0;        // 4-20 ma loop
    double imax=20.0;       // ditto
    double tmin=-100;       // from manual
    double tmax=200.0;      // ditto
    t = tmin + (tmax-tmin) * (v - imin*r/1000) / ((imax-imin)*r/1000);
    return t;
}

//====================================================================================================
void    MeanStdev(double *sum, double *sum2, int N, double missing)
//  Compute mean and standard deviation from
//  the count, the sum and the sum of squares.
{
    if( N <= 2 ){
        *sum = missing;
        *sum2 = missing;
    }
    else{
        *sum /= (double)N;      // mean value
        *sum2 = *sum2/(double)N - (*sum * *sum); // sumsq/N - mean^2
        *sum2 = *sum2 * (double)N / (double)(N-1); // (N/N-1) correction
        if( *sum2 < 0 ) *sum2 = 0;
        else *sum2 = sqrt(*sum2);
    }
    return;
}


//====================================================================================================
float PointScanDrum(float requestpos) {
    // Move the encoder angle to a requested position
    // Choose the smallest arc angle and move in that direction.
    // Within +/-8 degrees of the requested position
    // We need to cycle the drive very fast to get the necessary read
    // resolution using ReadEncoder().  Without this approach,
    // ReadEncoder() returns a position of 2-3 degrees at full speed
    // which is way beyond the precision of the A2 encoder.
    //
    // We may overshoot as we approach the requestpos and if there
    // is not an unequal shift fwd/back, we enter a hysteresis state.
    //
    //INPUT 
    //  requestpos = the desired angle
    // OUTPUT
    //  pos = the final encoder angle
    //

    float pos=0, diff=0,lmmult=1000;
    int direction=CW;  //, iOldDirection=CW;
    int msdelay;
    byte ib;
    unsigned long ulTime;

    // Get the initial encoder position
    pos = ReadEncoder (ee.encoderref);

    if(pos == MISSING){
        Serial.println("PointScanDrum error, ReadEncoder=MISSING.");
        return MISSING;
    }

    // Compute the smallest arc between pos and the requestpos
    diff = DiffAngle(requestpos,pos);
    //Serial.print("diff = ");
    //Serial.println(diff,2);

    // Only move the encoder if necessary
    if( fabs(diff) > ee.ScanTolerance ){
        // LED11 on 
        digitalWrite(LED11, HIGH);
        // set timer
        ulTime = millis(); // The beginning of the positioning.

        // If the encoder is +/-5 degrees from requestpos
        // start the motors continuously until we are within
        // ~5 degrees
        while(fabs(diff) > 5) {
            if( sign(diff) < 0 ) { 
                //Serial.print("Motor CCW.");
                ScanMotor(CCW); 
            }
            else { 
                //Serial.print("Motor CW.");
                ScanMotor(CW); 
            }
            delay(10);
            pos = ReadEncoder(ee.encoderref);
            diff = DiffAngle(requestpos,pos);

            if( millis() - ulTime > 10000 ) {
                Serial.println("PointScanDrum course timeout");
                digitalWrite(LED11, LOW);
                return pos;
            }
        }
        ScanMotor(STOP);
        pos = ReadEncoder(ee.encoderref);
        //Serial.print("End of course, ");Serial.println(pos,2);

        // NUDGING (W TIMEOUT)
        //Serial.println("Begin nudging. Pos=");Serial.println(pos,2); 
        while (fabs(diff) >= ee.ScanTolerance) {
            pos = ReadEncoder(ee.encoderref);
            diff=DiffAngle(requestpos,pos);
            // DIRECTION
            if( sign(diff) < 0 ){ 
                direction = CCW;
            }
            else { 
                direction = CW; 
            }
            // MOTOR ON TIME
            msdelay = abs(diff) * 8;
            if (msdelay < 2) msdelay=2;
            ScanMotor(direction);
            delay(msdelay);
            ScanMotor(STOP);

            // SETTLE AND CHECK
            delay(120);
            pos = ReadEncoder(ee.encoderref);
            diff=DiffAngle(requestpos,pos);
            // GIVE UP AFTER A SMALL EFFORT
            if( millis() - ulTime > 5000 ) {
                Serial.println("PointScanDrum timeout"); 
                return pos;
            }
        }
    }

    // Read the final position
    pos =  ReadEncoder(ee.encoderref); 
    digitalWrite(LED11, LOW);
    return pos;
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
    Serial.println("");
    Serial.print("PROGRAM:");
    Serial.print(PROGRAMNAME);
    Serial.print("   VERSION:");
    Serial.print(VERSION);
    Serial.print("  EDIT:");
    Serial.print(EDITDATE);
    Serial.println("");
}


//*******************************************************
// void Read4017 (double *adc, char chan)
// // Issue a command to the 4017, jump to receive, read in the string.
// // See Adam 4000 manual page 4-14, 
// //       #01<cr> for all channels,   >+7.2111+7.23453+...<cr> 
// //           ">+02.368+02.393+02.404+02.399+01.685+00.866+00.461+00.237"   len=57
// //       #01x<cr> where x=0-7,       >+1.4567<cr>
// //           ">+02.394"      len=8
// //       adc[7] is a float array of the result of the read.
// {
//     char strcmd[6];
//     char strin[65];
//     byte i, j, ic, chrin,lencmd,rxlen;
//     unsigned long millistart, millilapsed; // define input wait time
//     
//     //Serial.print("Read4017: ");   Serial.println(chan,DEC);
// 
//     // MAKE COMMAND
//     // all channels
//     if ( chan > 7 || chan < 0 ) {
//         strncpy(strcmd,"#01",3);
//         strcmd[3] = '\0';
//         lencmd = 3;
//         rxlen=59;
//     } 
//     // single channel
//     else {
//         // individual channel
//         strncpy(strcmd,"#01",3);
//         strcmd[3]=chan+48;           
//         strcmd[4]='\0';
//         lencmd=4;
//         rxlen=9;
//     }
// 
//     // Command sent
//     for(i=0;i<lencmd;i++){
//         Serial2.write(strcmd[i]);
//     }
//     Serial2.write(13); 
//     
//     delay(10);
//     
//     // Receive string
//     millistart = millis();
//     j=0;
//     while(j<rxlen){
//      if( Serial2.available() ){
//          chrin=Serial2.read();
//          if(chrin == 13){break;}
//          else{ strin[j++]=chrin; }
//      }
//      millilapsed=millis()-millistart;
//      if ( millilapsed > 700 ) break;
//     }
//     //Serial2.setTimeout(700);
//     //Serial2.readBytes(strin,rxlen);
//     
//     //ic = strlen(strin);
//     //Serial.print(ic); 
//     //Serial.println(" bytes received.");
//     //Serial.println(strin);
//  
//  strin[rxlen]='\0';
//  if(rxlen==9){
//      adc[chan]=atof( strin+1  );
//      //Serial.println(adc[0],2);
//  }
//  else {
//      j=0;
//      for(i=1; i<rxlen; i++){
//          if(strin[i]=='+'|| strin[i]=='-'){
//              adc[j]=atof( strin+i );
//              //Serial.print(j);Serial.print("  "); Serial.println(adc[j],2);
//              j++;
//          }
//      }
//  }
// }
// 

//*******************************************************
float ReadEncoder (float ref) 
// Read the USDIGITAL encoder
// Absolute position is computed using output = encoder angle - ref
//  input: zero reference angle.
//  output: encoder angle - ref OR MISSING
//  global variables:
//      MISSING (=-999) 
{
    unsigned char count=0;              // number of trys for input;
    unsigned long microstart, micronow; // define input wait time
    byte i=0;                           // number of bytes in the buffer.
    float angle = MISSING;
    byte e[4];

    count=0;
    while( count < 3 )  {
        // position command
        Serial1.write(EncCmd);

        // get three bytes
        microstart = micros();
        i=0;
        while(i<=2) {
            // i=0 get the command byte
            if (i==0 && Serial1.available()) {
                e[0]=Serial1.read();
                if (e[0]==EncCmd){
                    //Serial.print("0-");Serial.print(e[0],HEX);
                    i++;
                }
            }
            // i=1 First byte
            else if (Serial1.available() && i==1) {
                e[1]=Serial1.read();
                //Serial.print("  1-");Serial.print(e[1],HEX);
                i++;
            }
            // i=2, second byte
            else if (Serial1.available() && i==2) {
                e[2]=Serial1.read();
                //Serial.print("  2-");Serial.println(e[2],HEX);
                i++;
            }
            // timeout
            else if ( micros() - microstart > 10000 ) {
                //Serial.println("ReadEncoder timeout.");
                break;
            }
        }
        // DO WE HAVE A GOOD ANGLE
        if( i >= 2 ) {
            // This next calc assumes an encoder setting of 14400 resolution
            angle = (float)(e[1]*256+e[2]) / 40.0; 
            //Serial.print("angle=");Serial.print(angle,2);
            if(angle>=0 && angle <=360){
                count=100;
            }
        }
        // if not, try again
        else { 
            count++; 
        }
    }
    //Serial.print("angle = ");Serial.println(angle,3); 
    if ( angle != (float)MISSING ) {
        //Serial.print("ref = ");Serial.println(ref,3);
        angle += ref;
        if ( angle >= 360 ) angle -= 360;
        if ( angle < 0 ) angle += 360;
    }
    return angle;
}
//============================================================================
void        ReadKT15(double *irrad, double *irtemp)
{
    char e[10], chrin;
    byte i, count;
    double ddum;

    Serial3.setTimeout(100);

    // READ IN RAD
    while(Serial3.available()){
        Serial3.read();
    }
    Serial3.println("RAD");
    i=Serial3.readBytesUntil('\n',e,7);
    e[i]='\0';
    *irrad = atof(e);

    // READ IN RAD
    while(Serial3.available()){
        Serial3.read();
    }
    Serial3.println("TEMP");
    i=Serial3.readBytesUntil('\n',e,7);
    e[i]='\0';
    *irtemp = atof(e);

    return;
}

//============================================================================
void        ReadTilt(double *pitch, double *roll)
{
    unsigned long microstart; // define input wait time
    char e[4], chrin;
    byte i, count;
    double ddum;

    Serial2.setTimeout(100);

    // Clear buffer
    //  while( Serial2.available() ) Serial2.read();

    // READ IN PITCH, TRY SEVERAL TIMES
    //    count=0;
    //  Serial.print("pitch ");
    //    while( count < 3 )    {
    //      count++;
    Serial2.write(66);
    delay(10); //v6
    Serial2.readBytesUntil('\n',e,4);
    e[3]='\0';
    //      Serial.print(e[0],DEC);Serial.print("  ");  
    //      Serial.print(e[1],DEC);Serial.print("  ");  
    //      Serial.println(e[2],DEC);   
    //  }
    ddum=(double) Hex2Dec(e);
    if(ddum != MISSING && ddum >= 0 && ddum <= 4097){
        ddum-=2048;
        if(ddum < -1660) ddum=-1660;
        if(ddum > 1660) ddum=1660;
        *pitch=P1*(C1*ddum + C2*ddum*ddum + C3*ddum*ddum*ddum)+P0;
    } 
    else *pitch=MISSING;
    //  Serial.print("Pitch = "); Serial.println(*pitch,1);


    // READ IN ROLL, TRY SEVERAL TIMES
    // Clear buffer
    //  while( Serial2.available() ) Serial2.read();
    //    count=0;
    //  Serial.print("roll ");
    //    while( count < 3 )    {
    //      count++;
    Serial2.write(67);
    delay(10); //v6
    Serial2.readBytesUntil('\n',e,4);
    e[3]='\0';
    //      Serial.print(e[0],DEC);Serial.print("  ");  
    //      Serial.print(e[1],DEC);Serial.print("  ");  
    //      Serial.println(e[2],DEC);   
    //  }
    ddum=(double) Hex2Dec(e);
    if(ddum != MISSING && ddum >= 0 && ddum <= 4097){
        ddum-=2048;
        if(ddum < -1660) ddum=-1660;
        if(ddum > 1660) ddum=1660;
        *roll=R1*(C1*ddum + C2*ddum*ddum + C3*ddum*ddum*ddum)+R0;
    } 
    else *roll=MISSING;
    //  Serial.print("Roll = "); Serial.println(*roll,1);
    return;
}

//==================================================================
unsigned    Ref(int K) {
    if (K == ON){
        digitalWrite(REFSW,LOW);
        digitalWrite(LED32,HIGH);
        return OK;
    }
    else if (K == OFF){
        digitalWrite(REFSW,HIGH);
        digitalWrite(LED32,LOW);
        return OK;
    }
    return NOTOK;
}

//==================================================================
int ScanMotor(int direction)
/***************
 *   Operate the scan motor in a CCW, CW, or STOP mode.
 *   Output= ScanMotorFlag;
 *   defines: CCW, CW, STOP
 *   global variables: ScanMotorFlag;
 * 
 *   History
 *   08-01-2000 M R Reynolds
 *   2013-11-26 moved to Arduino
    **********/
{
    int flag;

    switch(direction){
    case CCW:
        digitalWrite(LED11, HIGH);
        digitalWrite(D1a, LOW);
        digitalWrite(D1b, HIGH);
        flag = CCW;
        break;

    case CW:
        digitalWrite(LED11, HIGH);
        digitalWrite(D1a, HIGH);
        digitalWrite(D1b, LOW);
        flag = CW;
        break;

    case STOP:
        digitalWrite(LED11, LOW);
        digitalWrite(D1a, LOW);
        digitalWrite(D1b, LOW);
        flag = STOP;
        break;
    }
    //ScanMotorFlag = flag;
    return flag;
}

//==================================================================
int ShutterMotor(int direction)
/***************
 *   Operate the shutter motor in a CCW, CW, or STOP mode.
 *   Output= flag;
 *   defines: CCW, CW, STOP
 *   global variables: ShutterMotorFlag;
 * 
 *   History
 *   From ScanMotor()
 *   2013-11-26 moved to Arduino
    **********/
{
    int flag;

    switch(direction){
    case CCW:
        digitalWrite(LED12,LOW);
        digitalWrite(D2a, LOW);
        digitalWrite(D2b, HIGH);
        flag = CCW;
        break;

    case CW:  // close
        digitalWrite(LED12,HIGH);
        digitalWrite(D2a, HIGH);
        digitalWrite(D2b, LOW);
        flag = CW;
        break;

    case STOP:
        digitalWrite(D2a, LOW);
        digitalWrite(D2b, LOW);
        flag = STOP;
        break;
    }
    //ShutterMotorFlag = flag;
    return flag;
}


//==============================================================================
void ShutterOpen(void) {
    unsigned long t0;
    double fdum;

    // CHECK STATE  
    if(digitalRead(DHe1)==0 && digitalRead(DHe2)==1) {
        ShutterState=OPEN;
        return;
    }

    // STOPWATCH
    t0=millis();
    // START MOTOR  CCW=open    CW=CLOSE
    ShutterMotor(CCW); 
    // STOP MOTOR keystroke || current || timeout || HEswitch 
    while( !Serial.available()  ){
        delay(100);
        // CURRENT
        fdum = abs(GetMotorCurrent());      // absolute value
        //Serial.println(fdum,2); // test
        if(fdum >= CCWSHUTTERCURRENTTHRESHOLD && (unsigned)(millis() - t0) > 100){
            Serial.println("SHUTTER OPEN, CCW CURRENT");
            ShutterMotor(STOP);
            break;
        }
        // TIMEOUT
        if ( (unsigned)(millis() - t0) > CCWSHUTTERTIMEOUT ){
            Serial.println("SHUTTER OPEN, CCW TIMEOUT");
            ShutterMotor(STOP);
            break;
        }
        // HE SWITCH
        if( digitalRead(DHe1)==0 ) {
            Serial.println("SHUTTER OPEN, HE1");
            delay(CCWSTOPMILLISECS);
            ShutterMotor(STOP);
            break;
        }
    }

    Serial.print((millis() - t0));
    Serial.println(" msecs");
    ShutterMotor(STOP);
    ShutterState = OPEN;
    digitalWrite(LED12,LOW);
    return;
}
//==============================================================================
void ShutterClose(void) {
    unsigned long t0;
    double fdum;
    // DISABLED?
    if ( ee.ShutterFlag > 0 ) {
        // CHECK DOOR POSITION
        if(digitalRead(DHe1)==1 && digitalRead(DHe2)==0) {
            //Serial.println("SHUTTER CLOSED");
            ShutterState=CLOSED;
        } 
        // CLOSE THE DOOR
        else {
            // STOPWATCH
            t0=millis();
            // START MOTOR  CCW=open    CW=CLOSE
            ShutterMotor(CW); 
            // STOP MOTOR keystroke || current || timeout || HEswitch 
            while( !Serial.available()  ){
                delay(100);
                // CURRENT
                fdum = abs(GetMotorCurrent() && (unsigned)(millis() - t0) > 100);       // absolute value
                //Serial.println(fdum,2);
                if(fdum >= CWSHUTTERCURRENTTHRESHOLD){
                    Serial.println("SHUTTER CLOSED, CW CURRENT");
                    ShutterMotor(STOP);
                    break;
                }
                // TIMEOUT
                if ( (unsigned)(millis() - t0) > CWSHUTTERTIMEOUT ){
                    Serial.println("SHUTTER CLOSED, CW TIMEOUT");
                    ShutterMotor(STOP);
                    break;
                }
                // HE SWITCH
                if( digitalRead(DHe2)==0 ) {
                    Serial.println("SHUTTER CLOSED, HE2");
                    delay(CWSTOPMILLISECS);
                    ShutterMotor(STOP);
                    break;
                }
            }
            Serial.print((millis() - t0));
            Serial.println(" msecs");
            ShutterMotor(STOP);
            ShutterState=CLOSED;
            digitalWrite(LED12,HIGH);
        }
    }
    return;
}




//============================================================================
int sign (float input) {
    return ((input < 0.0) ? NEG: POS);
}
//===========================================================================
double SteinhartHart(double beta[], double r)
//input
// beta is the 3x1 calibration vector where temp is computed by
//     xt = beta(1) + beta(2)*xr + beta(3)*xr^3
//   where
//      xr = log(1/R)
//      T = 1/xt - 273.15
// r = measured resistance, ohms
//
//output
// t = calibrated temperature vector, degC
{
    double vx, t2, t;

    vx = log(r);
    //Serial.print("r="); Serial.print(r,3);
    //Serial.print("  vx="); Serial.print(vx,3);
    //Serial.print("  beta="); Serial.print(beta
    //fitted curve
    t2 = beta[0] + beta[1] * vx + beta[2] * vx * vx * vx;
    t = 1 / t2 - (double)273.15;
    return t;
}

//*************************************************************/
double ComputeSSST(double t1,double t2,double k1,double k2,double ksky,double ksea,
double esea, double ebb, double Acorr, double Offset,double missing)
// where
//   t1 = black body 1, ambient, temperature, degC
//   t2 = heated BB temp, degC
//   k1, k2, ksky, ksea = kt15 readings for the different pointing angles, adc counts or mV
//   esea = ocean emissivity from viewing angle.
//   ebb = estimated emissivity of the black bodies, usually set to 1.
//   Acorr = calibration parameter, multiplier of the interpolation slope. Typ = 1 +/- 0.01
//   Offset = final sst adjustment, deg C. Typ +/-0.1.
//   missing = value for bad data, usually = -999
//  example
// 150211 rmr -- moved to perl module Isar.pm
{
    double s1,s2,sk,s1v,s2v,Ad,sd,sdv,Au,suv;
    double s_reflected, s_skin, ssst, ssst_uncorrected;

    if ( t1 != missing && t2 != missing && t1 > 0 && (t2-t1) > 5 && 
        ksky != missing && ksea != missing && k1 != missing && k2 != missing &&
        k2 > 0 && k1 > 0 && k2 > k1 ) {
        //Serial.print("SSST computed emis = ");Serial.println(esea,6);

        s1=GetRadFromTemp(t1);
        s2=GetRadFromTemp(t2);
        sk = s1;  // approximation of planck radiance from the kt15 lens
        //Serial.print("s1 = ");Serial.println(s1,6);
        //Serial.print("s2 = ");Serial.println(s2,6);

        // VIEW IRRADIANCES DEPEND ON THE EMISSIVITIES OF THE BB'S AND SOURCE
        s1v=ebb*s1 - (1-ebb)*sk;
        s2v=ebb*s2 - (1-ebb)*sk;
        //Serial.print("s1v = ");Serial.println(s1v,6);
        //Serial.print("s2v = ");Serial.println(s2v,6);

        // ---DOWN VIEW RADIANCE---
        Ad = (ksea-k1) / (k2-k1);
        //Serial.print("Ad = ");Serial.println(Ad,6);

        // Correct for the irt beam spread
        Ad = Acorr * Ad;
        //Serial.print("Ad corrected = ");Serial.println(Ad,6);

        // down view radiance
        sdv = s1v + (s2v-s1v)*Ad;
        //Serial.print("sdv = ");Serial.println(sdv,6);

        // up view radiance
        Au = Acorr * (ksky-k1) / (k2-k1);
        //Serial.print("Au corrected = ");Serial.println(Au,6);

        // up view radiance
        suv = s1v + (s2v-s1v)*Au;
        //Serial.print("suv = ");Serial.println(suv,6);

        // reflected sky radiance
        s_reflected = suv * (1-esea);
        //Serial.print("s_reflected = ");Serial.println(s_reflected,6);

        // planck radiance from the skin
        s_skin = (sdv - s_reflected) / esea;
        //Serial.print("s_skin = ");Serial.println(s_skin,6);

        // uncorrected ssst, ignore reflection
        ssst_uncorrected = GetTempFromRad(sdv/esea) + Offset;
        //Serial.print("ssst_uncorrected = ");Serial.println(ssst_uncorrected,3);

        // corrected skin temperature
        ssst = GetTempFromRad(s_skin) + Offset;
        //Serial.print("ssst = ");Serial.println(ssst,3);
    }
    else {
        ssst_uncorrected = ssst = missing;
    }
    return ssst;
}

/*****************************************************************************************/
double      GetRadFromTemp (double t){
    double x;
    x=ee.pt2b[0]*t*t*t + ee.pt2b[1]*t*t + ee.pt2b[2]*t + ee.pt2b[3];
    return x;
}



/*****************************************************************************************/
double      GetTempFromRad (double r){
    double x;
    x=ee.pb2t[0]*r*r*r + ee.pb2t[1]*r*r + ee.pb2t[2]*r + ee.pb2t[3];
    return x;
}


/***************************************************************************************/
unsigned long ElapsedTime (char *ddhhmmss){

    unsigned long Ld,Lh,Lm,Ls, ms;
    char ch[3];
    ddhhmmss[0]='\0';

    // Elapsed time, days, since startup
    ms = millis()-msstart;

    //days
    Ld = ms/86400000;           // number of days
    if(Ld>=0 && Ld < 100){
        ch[2]='\0'; ch[1]=Ld%10+48; ch[0]=Ld/10+48;
    }else{strcpy(ch,"xx");}
    strcat(ddhhmmss,ch);
    strcat(ddhhmmss,".");
//     Serial.print("days = ");  Serial.print(Ld);    
//     Serial.print("   string:");  Serial.print(ch);
//     Serial.print("   ddhhmmss:");  Serial.println(ddhhmmss);

    //hours
    Lh = (ms - Ld*86400000)/3600000;        // number hours
    if(Lh>=0 && Lh < 100){
        ch[2]='\0'; ch[1]=Lh%10+48; ch[0]=Lh/10+48;
    }else{strcpy(ch,"xx");}
    strcat(ddhhmmss,ch);
//     Serial.print("hours = ");  Serial.print(Lh);    
//     Serial.print("  string = ");  Serial.print(ch);
//     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

    //min
    Lm = (ms - Ld*86400000 - Lh*3600000)/60000;
    if(Lm>=0 && Lm < 100){
        ch[2]='\0'; ch[1]=Lm%10+48; ch[0]=Lm/10+48;
    }else{strcpy(ch,"xx");}
    strcat(ddhhmmss,ch);
//     Serial.print("mins = ");  Serial.print(Lm);    
//     Serial.print("  string = ");  Serial.print(ch);
//     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

    //sec
    Ls = (ms - Ld*86400000 - Lh*3600000 - Lm*60000)/1000;
    if(Ls>=0 && Ls < 100){
        ch[2]='\0'; ch[1]=Ls%10+48; ch[0]=Ls/10+48;
    }else{strcpy(ch,"xx");}
    strcat(ddhhmmss,ch);
//     Serial.print("secs = ");  Serial.print(Ls);    
//     Serial.print("  string = ");  Serial.print(ch);
//     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

    return ms;
}



