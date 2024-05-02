
// Libararies:
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

// Definitions
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define straightServo 56 // The angle used to drive straight

// Global variables
long enc1_count, enc2_count; // encoder variables
const double pulseVal = 0.13; //cm travalled in a pulse of encoders
double angZ, wantedAngle; // used for MPU
const int rs = 13, en = 12, d4 = 25, d5 = 26, d6 = 27, d7 = 14; // ports for the LCD
//numpad ports:
const byte ROWS = 4; //four rowsS
const byte COLS = 3; //three columns
byte rowPins[ROWS] = {2, 4, 5, 18}; 
byte colPins[COLS] = {19, 16, 15};


// Objects
MPU6050 mpu6050(Wire);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Numpad 
//Initiallise keypad
char keys[ROWS][COLS] = 
{
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


void setup()
{
  // start proccesses
  Wire.begin();
  Serial.begin(9600);  // start serial for output
  mpu6050.begin();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //Setup MPU
  mpu6050.calcGyroOffsets(true);

  delay(250);
  //Sanity check
  Serial.println("==Setup done==");
  lcd.print("Hello world");
  delay(100);
}

//Instruction List Init
int listL = 0; //For counter; marks end of list; List of instructions to be executed
char instruction[20];
char lcdDisplayTxt[32]; //32 long char array for max characters in LCD
char tempLCD[32]; // Variable to keep resetting for when intructions are excecuted

int displayL; //Length of display array

void loop()
{
  // Update variables every loop
  char key = keypad.getKey(); //register keystrokes

  //Only executes anything involving the keypad when a key pressed
  if(key)
  { 
    Serial.print("Key = ");
    Serial.println(key);

    if (key == '#')
    {
      Serial.println("\n");
      Serial.println("Start");
      for (int i =0 ; i < listL; i++)
      {
        // Display next instructions:
        
        // Create array of next instructions
        memset(tempLCD, '\0', 32*sizeof(char)); // Empty lcdDisplayTxt to repopulate with the instructions that are left
        for(int j = 0; j < listL-i - 1; j++) // -1 in the length to account for looking at next value
        {
          tempLCD[j] = lcdDisplayTxt [i+j+1]; // need to add j to find the next values
          // Display character if it is less than 16
          if(j<16)
          {
            lcd.setCursor(j, 0); //move cursor
            lcd.print(tempLCD[j]);
          }
          else
          {
            lcd.setCursor(j - 16, 1); //move cursor to second line when needed
            lcd.print(tempLCD[j]);
          }
        }

        // Excecute Instructions and Display the instruction being excecuted at the moment
        lcd.setCursor(0, 1); // Move cursor to display text for current instruction
        switch(instruction[i])
        {
          case '8':
            lcd.print("Reversing!!!");
            driveDistance(-10);
            break;

          case '6':
            lcd.print("Going Right!!!");
            turnAngle(90);
            break;

          case '4':
            lcd.print("Going Left!!!");
            turnAngle(-90);
            break;

          case '2':
            lcd.print("ONWARDS!!!");
            driveDistance(10);
            break;
        }
      }
      // Stop Motors
      driveMotor(0, 0, straightServo);

      // Clear instruction list
      Serial.println("Clear");
      memset(instruction, '0', 20*sizeof(char));
      memset(lcdDisplayTxt, '\0', 32*sizeof(char)); //clear LCD variable
      memset(tempLCD, '\0', 32*sizeof(char)); // Empty tempuary list for displaying values      lcd.setCursor(0, 0); //move cursor
      lcd.print("All");
      lcd.setCursor(0, 1); //move cursor
      lcd.print("Done!");
      listL = 0;

    }
    else if ((key == '8')||(key == '6')||(key == '4')||(key == '2'))
    {
      Serial.print(key);
      if (listL < 20)
      {
        // Add instructions to lists
        instruction[listL] = key;
        // Create list of instructions as characters
        switch(key)
        {
          case '8':
            lcdDisplayTxt[listL] = 'B';
            break;

          case '6':
            lcdDisplayTxt[listL] = 'R';
            break;

          case '4':
            lcdDisplayTxt[listL] = 'L';
            break;

          case '2':
            lcdDisplayTxt[listL] = 'F';
            break;
        }

        // Display the added instructions on the LCD
        switch(listL)
        {
          // Will display the "Stored: " string then move to the default case to move the cursor to write more values
          case 0:
            lcd.setCursor(0, 0);
            lcd.print("Stored: ");
          
          default:
            if(listL + 8 < 16) //If cursor is in the first row:
            {
              // Adding 8 as "Stored: " is 8 characters long
              lcd.setCursor(listL + 8, 0);
              lcd.print(lcdDisplayTxt[listL]);
            }
            else
            {
              // Will have only printed 8 characters from listL,
              lcd.setCursor(listL - 8, 1);  
              lcd.print(lcdDisplayTxt[listL]);
            }
        }
        listL = listL + 1;
        Serial.print("listL: ");
        Serial.println(listL);
      }
      else
      {
        Serial.println("20 Items in list, list full.");
        // print to LCD list is filled 
        // List is full.\n clear or execute
      }
    }
    else
    {
      Serial.println("Clear");
      memset(instruction, '0', 20*sizeof(char));
      memset(lcdDisplayTxt, '\0', 32*sizeof(char)); //clear LCD variable      lcd.setCursor(0, 0); //move cursor
      lcd.print("All");
      lcd.setCursor(0, 1); //move cursor
      lcd.print("Clear!");
      listL = 0;    
    }
  }
  /* 
  updateSense();

  //Encoders
  Serial.print(enc1_count);
  Serial.print(" + ");
  Serial.println(enc2_count);

  //MPU
  Serial.print("MPU val: ");
  Serial.print(angZ);
  Serial.println("\n");*/
}

void driveMotor(int16_t leftMotorSpeed, int16_t rightMotorSpeed, int16_t servoValue)
{
  //Ensure values are between -255 and 255 for both motors
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  servoValue = constrain(servoValue, 2, 180);

  //Send instructions to the ESP mainboard
  Wire.beginTransmission(I2C_SLAVE_ADDR);

  Wire.write((byte)((leftMotorSpeed & 0x0000FF00) >> 8 )); 
  Wire.write((byte)(leftMotorSpeed & 0x000000FF)); //writes last byte

  Wire.write((byte)((rightMotorSpeed & 0x0000FF00) >> 8 ));
  Wire.write((byte)(rightMotorSpeed & 0x000000FF)); //writes last byte

  Wire.write((byte)((servoValue & 0x0000FF00) >> 8)); 
  Wire.write((byte)(servoValue & 0x000000FF)); //writes last byte

  // sends six bytes in total
  Wire.endTransmission();    // stop transmitting

  /*/ Change speed of motors
  Serial.print("Motor speeds (L R): ");
  Serial.print(leftMotorSpeed);
  Serial.print(" + ");
  Serial.println(rightMotorSpeed);//*/
}

// Update MPU and Encoders
void updateSense()
{
  //Get values from mainboard
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t enc1_16_9 = Wire.read();  // receive bits 16 to 9 of enc1 (one byte)
  uint8_t enc1_8_1 = Wire.read();   // receive bits 8 to 1 of enc1 (one byte)
  uint8_t enc2_16_9 = Wire.read();   // receive bits 16 to 9 of enc2 (one byte)
  uint8_t enc2_8_1 = Wire.read();   // receive bits 8 to 1 of enc2 (one byte)

  //update global variables
  enc1_count = (enc1_16_9 << 8) | enc1_8_1;
  enc2_count = (enc2_16_9 << 8) | enc2_8_1;

  mpu6050.update(); //Get current mpu vals

  //assign MPU values
  angZ = mpu6050.getAngleZ();
}

void driveDistance(double targetDistance) //where each unit is 1cm
{
  updateSense();

  // Variable Space
  int lSpeed, rSpeed;
  //Original values to compare OG start angle (maintain course) and encoder value (measure distance) 
  int orgEnc1 = enc1_count;
  int orgEnc2 = enc2_count;
  // Variable for finding intended gap between encoder  values
  int variance;

  double distanceTravelled; // To measure average distance travelled

  //Determine if forwards or backwards
  bool forwardFlag = true;
  if (targetDistance < 0)
  {
    targetDistance = fabs(targetDistance); //turns distance into abs value
    forwardFlag = false;
  }//Also ensures that while loop works regardless of direction

  //Go forward in a straight direction using PID with MPU6050
  //forward
  while (distanceTravelled < targetDistance - 0.09) //compares average distance travelled by encoders (in pulses that are multiplied by a constant for distance travelled in a pulse)
  {
    variance = enc1_count - orgEnc1 + enc2_count - orgEnc2;
    // find lspeed and rspeed in if statements
    if (forwardFlag == false) //can move this logic and make the drive be in prev comparison. 
    {
      if (variance > 0)
      // Left wheel is leading, Right is lagging
      {
        lSpeed = -180 + (variance); // negative + positive makes value closer to 0
        rSpeed = -180;
      }
      else
      // Left wheel is lagging, Right is leading
      // Variance would be negative and such needs to be subtracted to slow right motor
      {
        lSpeed = -180;
        rSpeed = -180 - (variance);
      }
    }
    else
    {
      if (variance > 0)
      // Left wheel is leading, Right is lagging
      {
        lSpeed = 180 - (variance);
        rSpeed = 180;
      }
      else
      // Left wheel is lagging, Right is leading
      // Variance would be negative and such needs to be added to correct the speed
      {
        lSpeed = 180;
        rSpeed = 180 + (variance);
      }
      
    }
    driveMotor(lSpeed, rSpeed, straightServo);

    updateSense();
    distanceTravelled = fabs(enc1_count - orgEnc1 + enc2_count - orgEnc2)*pulseVal/2; //finds average distance travelled.
    Serial.print("Distance travelled = ");
    Serial.println(distanceTravelled);
  }
}

void turnAngle(double turnAmount) 
// Enter angle to be turned by the EEEBot and the direction
{
  // Variable space
  double turnFactor, lVal, rVal, sVal; // to use for smooth turning; values for driving the motors
  bool leftFlag = true, whileFlag = true; //variable to signify if the bot is going left or right; variable to signify if the loop should be open or close
  if (turnAmount > 0) leftFlag = false;

  updateSense(); //update current z angle
  wantedAngle = angZ + turnAmount; // Wanted zAngle for turn
  do
  {
    turnFactor = 2.5* log(fabs(angZ-wantedAngle)+0.07)+3;
    // will change turn speed in proportion to the difference between (absolute) current and wanted angle
    // (Allows car to slow the turning down as nears the desired turning amount)

    switch(leftFlag)
    {
      //NOTE: wanted angle is less than current when left, greater when going right
      // Used 2 different cases for this reason.
      case true:
        // Determine the proportional values
        lVal = -5.5 * turnFactor;
        rVal = 2 * turnFactor;
        sVal = - 2 * turnFactor;

        // Check if the loop should continue
        if(wantedAngle >= angZ) whileFlag = false;
        break;

      default:
        // Determine the proportional values
        lVal = 2 * turnFactor;
        rVal = -5.5 * turnFactor;
        sVal = 2 * turnFactor;

        // Check if the loop should continue
        if(wantedAngle <= angZ) whileFlag = false;
    }

    driveMotor(180 + lVal, 180 + rVal, straightServo + sVal); //proportional/smooth turning
    updateSense(); //Gets z angle
  }
  while(whileFlag);
}
