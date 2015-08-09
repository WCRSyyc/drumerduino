/* drummerduino

Play a physical drum with an Arduino controlling the sticks.

Using  Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438

IDEAS:

Could have a profile for each stick + motor, to compensate for any differences

Could implement deliberate motion differences between the sticks, either as
part of the profiles, or in the measures (or both)
- a stick that is beating slower (3/4 time) could also move slower in the upper
  end of the motion range.  Probably want the last 'strike' part to be full speed
  to get the sharp 'tap' when hitting the drum head.
- could also change the motion extent: a slower beating stick could move the
  same speed, but move further.
- more complex logic, but could enhance the visual aspect
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

unsigned const int SERIAL_SPEED = 57600;
//300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
unsigned const long MAX_ULONG = 0xffffffff;// 4294967295

// I2C address for the motor shield
// https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/faq
uint8_t const I2C_ADDR = 0x60; // default motor shield address
uint16_t const MAX_PWM_FREQ = 1600; // default (Hz) (max 4096)

// The motor shield 'ports' that the stick controlling motors are attached to
uint8_t const LEFT_PORT = 1;
uint8_t const RIGHT_PORT = 2;

// Preference / alternative to fixed speed, would be smooth acceleration to reach
// the maximum speed just before impacting the drum head.  That will slow the
// maximum 'drum roll' speed though
uint16_t const STRIKE_SPEED = 255; // maximum

// For the visual aspect of drumming, a long swing of the sticks is probably better.
// For top speed (drum roll), as short of swing as will get to the needed terminal
// velocity is better.

// Initial values taken from tests driving the motors with 5V directly from the
// Arduino board.  When powered from the (motor rated) 12V, speed will go up, and
// specified timings will need to come down.
// All times in microseconds, which without wrap araound compensation will limit
// the maximum run time to a bit over an hour.
unsigned long const SWING_TIME = 60000; // microseconds
unsigned long const BOUNCE_TIME = 20000; //

// Setup a beat sequence/rhythm.  Normally, the beat intervals should be at least
// BOUNCE_TIME + 2 x SWING_TIME + stop time (about BOUNCE_TIME?).  For a drum roll,
// with reduced swing distance, the intervals might be reduced.
unsigned long const leftTrack[] = {0, 200000 };
unsigned long const rightTrack[] = {100000, 20000};
unsigned long const repeatInterval = 400000;
// State constants for the position/movement for each stick
unsigned int const SWING = 0; // moving toward the drum head
unsigned int const BOUNCE = 1; // waiting for the stick to bounce off the drum head
unsigned int const RECOVER = 2; // moving away from the drum head
unsigned int const HOME = 3; // rest state, ready to start swing
// How long to stay in a state before advancing to the next state (without interrupting)
unsigned long const STATE_TIME[] = {MAX_ULONG, SWING_TIME, BOUNCE_TIME, SWING_TIME};

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(I2C_ADDR);
// Specify the motor shield 'ports' each of the stick motors is attached to
Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_PORT);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_PORT);

Adafruit_DCMotor *allMotors[] = {leftMotor, rightMotor};
unsigned const long *allTracks[] = {leftTrack, rightTrack};
unsigned const int trackCount = sizeof(allTracks) / sizeof(unsigned long *);

// Index into the track arrays
unsigned int nextBeat[trackCount];
// The state (of the stick motion) of each each track
unsigned int beatState[trackCount];
// Time point that the current stick state started
unsigned long stateStart[trackCount];
unsigned long measureOffset;

void setup() {
  unsigned int track;

  Serial.begin(SERIAL_SPEED);
  Serial.println("DrummerDuino starting");

  AFMS.begin(MAX_PWM_FREQ);

  measureOffset = micros();
  for (track = 0; track < trackCount; track ++) {
    // Set the motor speeds, and turn each of them on (and off again)
    allMotors[track]->setSpeed(STRIKE_SPEED);
    allMotors[track]->run(FORWARD);
    allMotors[track]->run(RELEASE);

    // Setup a base time reference
    nextBeat[track] = 0;
    beatState[track] = HOME;
    stateStart[track] = measureOffset;
  }
}

void loop() {
  unsigned int track;
  unsigned long now = micros();
  unsigned long measureTime = now - measureOffset;
  for (track = 0; track < trackCount; track++) {
    if (measureTime >= nextBeat[track]) {
      startSwing(track, now);
    } else {
      continueBeat(track, now);
    }
  }
}

void startSwing(unsigned int stick, unsigned long curTime) {
  // Clear (componsate for?) any left over status from the previous swing
  if (beatState[stick] != HOME) {
  //   // interrupting previous}
  //   if (beatState[stick != RECOVER]) {
  //     // ERROR, PREVIOUS STRIKE NOT COMPLETED
  //   }
    beatState[stick] = HOME;
  }
  nextState(stick, curTime);
}

void continueBeat(unsigned int stick, unsigned long curTime) {
  if (curTime < STATE_TIME[beatState[stick]]) {
    // Not time to change the state yet
    return;
  }

  // Ready to change to the next state (for the current stick)
  nextState(stick, curTime);
}

// Send the command(s) to change a stick to the next state in the cycle
void nextState(unsigned int stick, unsigned long curTime) {
  // NOTE: With the current implementation there should be no need to change the
  // speed.  That is set during initialization, and never changes.

  // Advance to the next state in the cycle for the stick
  beatState[stick]++;
  if (beatState[stick] > HOME) {
    beatState[stick] = SWING;
  }
  // Save the time when the new state started, to use for elapsed time in state
  stateStart[stick] = curTime;
  switch (beatState[stick]) {
    case SWING: // 0
      allMotors[stick]->run(FORWARD);
      break;
    case BOUNCE:// 1
      allMotors[stick]->run(RELEASE);
      break;
    case RECOVER:// 2
      allMotors[stick]->run(BACKWARD);
      break;
    case HOME:// 3
      allMotors[stick]->run(RELEASE);
      break;
  }
}
