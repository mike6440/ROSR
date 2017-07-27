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
// NOTE ENCODER S/R FOR ROSR 1 AND 2
// !! USED FOR ROSR1 AND ROSR2
//*******************************************************
// float ReadEncoder (float ref)
// Read the USDIGITAL encoder
// Absolute position is computed using output = encoder angle - ref
//  input: zero reference angle.
//  output: encoder angle - ref OR MISSING
//  global variables:
//      MISSING (=-999)
// {
//   unsigned char count = 0;            // number of trys for input;
//   unsigned long microstart, micronow; // define input wait time
//   byte i = 0;                         // number of bytes in the buffer.
//   float angle = MISSING;
//   byte e[4];
// 
//   count = 0;
//   while ( count < 3 )  {
//     // position command
//     Serial1.write(EncCmd);
// 
//     // get three bytes
//     microstart = micros();
//     i = 0;
//     while (i <= 2) {
//       // i=0 get the command byte
//       if (i == 0 && Serial1.available()) {
//         e[0] = Serial1.read();
//         if (e[0] == EncCmd) {
//           //Serial.print("0-");Serial.print(e[0],HEX);
//           i++;
//         }
//       }
//       // i=1 First byte
//       else if (Serial1.available() && i == 1) {
//         e[1] = Serial1.read();
//         //Serial.print("  1-");Serial.print(e[1],HEX);
//         i++;
//       }
//       // i=2, second byte
//       else if (Serial1.available() && i == 2) {
//         e[2] = Serial1.read();
//         //Serial.print("  2-");Serial.println(e[2],HEX);
//         i++;
//       }
//       // timeout
//       else if ( micros() - microstart > 10000 ) {
//         //Serial.println("ReadEncoder timeout.");
//         break;
//       }
//     }
//     // DO WE HAVE A GOOD ANGLE
//     if ( i >= 2 ) {
//       // This next calc assumes an encoder setting of 14400 resolution
//       angle = (float)(e[1] * 256 + e[2]) / 40.0;
//       //Serial.print("angle=");Serial.print(angle,2);
//       if (angle >= 0 && angle <= 360) {
//         count = 100;
//       }
//     }
//     // if not, try again
//     else {
//       count++;
//     }
//   }
//   //Serial.print("angle = ");Serial.println(angle,3);
//   if ( angle != (float)MISSING ) {
//     //Serial.print("ref = ");Serial.println(ref,3);
//     angle += ref;
//     if ( angle >= 360 ) angle -= 360;
//     if ( angle < 0 ) angle += 360;
//   }
//   return angle;
// }

