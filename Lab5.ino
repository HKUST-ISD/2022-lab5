//**************************//
// Skeleton code of LAB 5   //
//**************************//
// Task 1
// Description:
//  - Using software serial (UART) library to communicate with GY-25Z
//  - Get Euler Angles (Yaw, Pitch and Roll)
//  - Check the result by Serial Plotter of Arduino IDE
// Connection:
//  (Arduino) D10 - (GY-25Z) TX
//  (Arduino) D11 - (GY-25Z) RX
//===============================================
// Task 2
// Description:
//  - Due to the communication error, there may be some invalid data received
//  - Checksum is one of the method to verify the received data
//  - Screen out the invalid packet
//  - Check the result again by Serial Plotter
//===============================================
// Task 3
// Description:
//  - indicate if the breadboard is turned LEFT or RIGHT
//===============================================

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);    // PIN10 as RX of Arduino; PIN11 as TX of Arduino;
int YPR[3];                         // Euler Angles
unsigned char receive_buf[30];      // receive buffer from GY-25Z
unsigned char counter = 0;          // counter of received buffer
boolean packet_rdy = false;
int pre_yaw_angle = 0;              // previous yaw angle

//-----------------------------------------------------------
void setup()
{
  Serial.begin(115200);     // initial serial monitor at baud rate 115200
  mySerial.begin();         // initial serial communication with GY-25Z at baud rate 115200
  mySerial.listen();        // listen to mySerial
  delay(3000);              // wait 3 sec before send command to GY-25Z

  mySerial.write();         // According to spec, set GY-25Z output Euler Angles only
  mySerial.write();         // Command is: 0xA5, 0x55, 0x10, checksum
  mySerial.write();         
  mySerial.write();         // checksum of this command = sum of TX data (lower byte) except checksum
  delay(100); 
 
  mySerial.write(0XA5); 
  mySerial.write(0X56);
  mySerial.write(0X02);     // Set GY-25Z as output data automatically
  mySerial.write(0XFD);
  delay(100);
}
//-------------------------------------------------------------
void loop(){
  while(mySerial.available())       // when data recevied from mySerial
  {   
    receive_buf[counter] = (unsigned char)mySerial.read();    // put the received data into buffer
    if(counter == 0 && receive_buf[0] != 0x5A)                // check the 1st byte of packet if it is 0x5A
      return;        
    counter++;
    if(counter == 11)               // total bytes of packet (Euler Angles only) should be 11
    {    
       counter = 0;                 // reset the counter for next packet
       packet_rdy = true;
    }       
  }

  // Received packet of Euler Angles from GY-25Z:
  //  _________________________________________________________________________________________________
  //  |    Header   | type | bytes|       Roll       |       Pitch      |        YAW       | Checksum |
  //  | 0x5A | 0x5A | 0x10 | 0x06 | R[15:8] | R[7:0] | P[15:8] | P[7:0] | Y[15:8] | Y[7:0] |   [7:0]  |
  //  -------------------------------------------------------------------------------------------------
  if(packet_rdy)
  { 
    if(receive_buf[0] == 0x5A && receive_buf[1] == 0x5A)        // check the header: 0x5A, 0x5A
    {
      //==========================================================================
      // Task 2: error checking of the packet received by checksum (last byte)
      // adding first 10 bytes of the packet to get the checksum value
      // compare with the last byte to verify the packet
      //==========================================================================
      /*
       * Add error checking here for Task 2  
       */
       // if(){

        // combine the 16 bits data received from the packet
        // divide the data by 100 as the data included 2 digits after decimal point
        YPR[0] =         // Roll
        YPR[1] =         // Pitch
        YPR[2] =         // Yaw

        Serial.print(YPR[0], DEC); Serial.print("\t");          // print Roll
        Serial.print(YPR[1], DEC); Serial.print("\t");          // print Pitch
        Serial.println(YPR[2], DEC);                            // print Yaw
      //}
    }
    packet_rdy = false;

    //========================
    // Task 3
    //========================
    // Assume GY-25Z is horizontally attached on the breadboard,
    // we can check the Yaw angle to identify if it is turned LEFT or RIGHT
    //task3(YPR[2]);
  } 
}

//=============================================
// Task 2
//---------------------------------------------
// check sum function:
//  input:  buf       - buffer array
//          num_bytes - total numbers of bytes to do the addition
//---------------------------------------------
unsigned char checksum(unsigned char *buf, int num_bytes)
{
  unsigned char sum = 0;
  // sum of the input buffer array

  return sum;
}

//=============================================
// Task 3
//---------------------------------------------
// Compare the current yaw angle with the previous yaw angle
// Determine if it is turned left or right
//---------------------------------------------
void task3(int yaw_angle){
  // add if else case

  pre_yaw_angle = yaw_angle;            // update the current YAW angle
}
