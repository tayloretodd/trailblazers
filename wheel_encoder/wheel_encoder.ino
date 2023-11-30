#include <Arduino.h>
#include <math.h>

// testing vars
#define TESTING true

// define wheel encoder pins
#define wheelEncoderPin_forward 1
#define wheelEncoderPin_backward 2

// declare variables and constants
int ticksPerRotation;
float wheelDiameter;
int wheelTicks;
float totalDistance;
bool forwardFlag;
bool backwardFlag;
float unitConversion;
bool usingInches;

// define wheel encoder interrupts
/* if forward wheel encoder is sensed first,
 * this pin (1) will go low first, then the 
 * backward pin will go low after, the cart
 * is moving forward in this case, and
 * distance should be increased */
void ForwardISR()
{
  forwardFlag = backwardFlag ? false : true; // if backward flag is already set, this interrupt was triggered secont
  wheelTicks++;
}

/* if backward  wheel encoder is sensed first,
 * this pin (2) will go low first, then the
 * forward pin will go low after, the cart
 * is moving backward in this case, and
 * distance should be decreased. */
void BackwardISR()
{
  backwardFlag = forwardFlag ? false : true; // if forward flag is already set, this interrupt was triggered second
  wheelTicks--;

  if(wheelTicks < 0) {wheelTicks = 0;}
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  //define pin modes and attach interrupts
  /* interrupt modes will have to be switched
   * once we change to SMD part since this version
   * goes high (I believe, it could also only
   * change poles, I will have to double check) */
  pinMode(wheelEncoderPin_forward, INPUT);
  attachInterrupt(wheelEncoderPin_forward, ForwardISR, FALLING); // could also be low
  pinMode(wheelEncoderPin_backward, INPUT);
  attachInterrupt(wheelEncoderPin_backward, BackwardISR, FALLING); // could also be low

  // hard coding; will need to come back to this and
  // determine how to set these values via the GUI
  ticksPerRotation = 2;
  wheelDiameter = 15.0;
  usingInches = true;
  wheelTicks = 0;   // could be a value if starting from a station
  // end hard coding

  unitConversion = usingInches? 1.0 : 2.54;

  Serial.println("Beginning distance sensing.");
}

void loop() {
  // put your main code here, to run repeatedly:

  /* will interrupts be too close together?
   * maybe just change interrupt code to only set
   * flags? -- requires testing */

  if(forwardFlag) // wheel is moving forward
  {
    // reset flags
    forwardFlag = false;
    backwardFlag = false;

  }

  if(backwardFlag) // wheel is moving backward
  {
    // reset flags
    forwardFlag = false;
    backwardFlag = false;
  }

  totalDistance = (wheelTicks / ticksPerRotation) * (wheelDiameter * PI) * unitConversion;

  if(TESTING)
  {
    // print results to Serial monitor for testing
    Serial.print("Total distance = ");
    Serial.print(totalDistance);

    if(usingInches)
    {
      Serial.println(" in");
    }
    else
    {
      Serial.println(" cm");
    }
  }
  
  // bluetooth transmission code will go in an else statement here
}
