/*
  blink3
  Turns LED on and off.
  Some serial output
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int Dheartbeat = 13;
const int SEITX = 23;

int Da=8;

#define		PROGRAMNAME	"blink3"
#define		VERSION		"2"
#define		EDITDATE	"131218" 
#define  EEPROM_ID		1

int i = 0;

// the setup routine runs once when you press reset:
void setup() {                
	Serial.begin(9600);
    Serial.begin(9600);	// serial 
	// initialize the digital pin as an output.
	pinMode(Dheartbeat, OUTPUT);     
  	pinMode(SEITX, OUTPUT);  // RS485 control
	pinMode(Da, OUTPUT);                               // 485 0/1 = tx/rx
	Serial.print(PROGRAMNAME);Serial.print("\n");
	i=10;
	while (i>0){
		Serial.print(i);Serial.print(", ");
		i--;
	}
Serial.println("");
  digitalWrite(SEITX, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
  	digitalWrite(SEITX, HIGH);
	digitalWrite(Dheartbeat, HIGH);
	digitalWrite(Da,HIGH);       // Rcv mode, D->0
	delay(2000);
  	digitalWrite(SEITX, LOW);
	digitalWrite(Dheartbeat, LOW);
	digitalWrite(Da,LOW);         // Tx mode, D->1
	delay(1000);
	Serial.println("blink3_micro");
}
