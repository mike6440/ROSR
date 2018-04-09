
// USED FOR ROSR3 AND AFTER. USES 485 CIRCUIT
//*******************************************************
float ReadEncoder (float ref)
// Read the USDIGITAL encoder
// Absolute position is computed using output = encoder angle - ref
//  input: zero reference angle.
//  output: encoder angle - ref OR MISSING
//  global variables:
//      MISSING (=-999)
{
	unsigned char count = 0;            // number of trys for input;
	byte i = 0;                         // number of bytes in the buffer.
	float angle = MISSING;
	byte e[4];

	while ( count < 2 )  { // test
		// Turn to transmit 485
		digitalWrite(SEITX, HIGH);
		delay(2); // short delay for things to settle
	// clear buffer
		while (Serial1.available()) {
			Serial1.read();
		}
		// position command -- send 0x1E
		Serial1.write(EncCmd);
		// =900 for rosr1--3. For rosr4 I needed to use 1000 for reliable operation.
		delayMicroseconds(1000); // !! short delay for tx to complete
		// back to receive 485
		digitalWrite(SEITX, LOW);
		delay(20); // 
		// get bytes
		i = 0;
		while ( Serial1.available() ) {
			e[i] = Serial1.read();
			//Serial.print("byte ");Serial.print(i,DEC);Serial.print(" = ");Serial.println(e[i],DEC);
			i++;
			if(i>2) break;
		}
		// DO WE HAVE A GOOD ANGLE
		if ( i >= 2 ) {
			// This next calc assumes an encoder setting of 14400 resolution
			angle = ((float)e[0] * 256 + (float)e[1]) / 40.0;
			if (angle >= 0 && angle <= 360) break;
			else angle=MISSING;
			//Serial.print("test angle = ");Serial.println(angle,3);
		}
		count++;
	}
	if ( angle != (float)MISSING ) {
		//Serial.print("ref = ");Serial.println(ref,3);
		angle += ref;
		if ( angle >= 360 ) angle -= 360;
		if ( angle < 0 ) angle += 360;
	}
	return angle;
}
