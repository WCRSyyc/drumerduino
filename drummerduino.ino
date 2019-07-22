/* drummerduino

Play a physical drum with an Arduino controlling the sticks.

Using  Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
controlling electric car door lock mechanisms as actuators for the drum sticks

IDEAS:

Could have a profile for each stick + motor, to compensate for any differences

Could implement deliberate motion differences between the sticks, either as
part of the profiles, or in the measures (or both)
- a stick that is beating slower (3/4 time) could also move slower in the upper
  end of the motion range.  Probably want the last 'strike' part to be full speed
  to get the sharp 'tap' when hitting the drum head.
- could also change the motion extent: a slower beating stick could move the
  same speed, but move further.
- use acceleration as part of the motion profiles
- more complex logic, but could enhance the visual aspect

could have multiple profiles for the stick motions, based on the measures to be
played, or flourishes during longer intervals.

For the visual aspect of drumming, a long swing of the sticks is probably better.
For top speed (drum roll), as short of swing as will get to the needed terminal
velocity is better.

Create a compact notation for drum notes.  Note lengths/speed are typically
whole, half, quarter, eight, sixteenth, thirty-second.  Nice powers of two.  A
byte could represent a note, with a single bit set to specify the length.  That
could be compacted significantly.  3 bits gives 8 values, which would cover everything
from whole down to 1/128.  A null note is also needed, when there is no note at
the start of measure/track.  Add some bits for strength/loudness.  How about
muffled beat, where the normal bounce is prevented?  Or a 'touch' used to deaden
the ring from a previous strike?

motion profile:
- total travel distance
- acceleration curve
- terminal speed (at strike point)
 */

#include <Adafruit_MotorShield.h>

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

uint16_t const STRIKE_SPEED = 255; // maximum

// Initial values taken from tests driving the motors with 5V directly from the
// Arduino board.  When powered from the (motor rated) 12V, speed will go up, and
// specified timing intervals will need to come down.
// All times in microseconds, which without wrap around compensation will limit
// the maximum run time to a bit over an hour.
unsigned long const SWING_TIME = 60000;
unsigned long const BOUNCE_TIME = 20000;
unsigned long const MEASURE_LENGTH = 1000000; // 1 second

// State constants for the position/movement for each stick
unsigned int const SWING = 0; // moving toward the drum head
unsigned int const BOUNCE = 1; // waiting for the stick to bounce off the drum head
unsigned int const RECOVER = 2; // moving away from the drum head
unsigned int const HOME = 3; // rest state, ready to start swing
// How long to stay in each state before advancing to the next state
unsigned long const STATE_TIME[] = {SWING_TIME, BOUNCE_TIME, SWING_TIME, MAX_ULONG};

// Setup a beat sequence/rhythm.  These are the start time points within a
// measure.  Keep the intervals large enough to complete the previous 'beat'
// (at least: BOUNCE_TIME + 2 x SWING_TIME + stop time
unsigned long const leftTrack[] = {0, 200000, 40000};// 1/2, 1,
unsigned long const rightTrack[] = {100000, 30000};// null, 1/4, 3/4

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(I2C_ADDR);
// Specify the motor shield 'ports' each of the stick motors is attached to
Adafruit_DCMotor *leftStick = AFMS.getMotor(LEFT_PORT);
Adafruit_DCMotor *rightStick = AFMS.getMotor(RIGHT_PORT);

Adafruit_DCMotor *allSticks[] = {leftStick, rightStick};
unsigned const long *allTracks[] = {leftTrack, rightTrack};
unsigned const int TRACKS = sizeof(allTracks) / sizeof(unsigned long *);

// Index into the track arrays
unsigned int nextBeat[TRACKS];
// The state (of the stick motion) of each each track
unsigned int beatState[TRACKS];
// (future) time point that the current stick state will end
unsigned long stateEnd[TRACKS];
unsigned int inMeasure[TRACKS];
unsigned long measureBase;
boolean afterMeasure;

void setup() {
  unsigned int track;

  Serial.begin(SERIAL_SPEED);
  Serial.println("DrummerDuino starting");

  AFMS.begin(MAX_PWM_FREQ);

  for (track = 0; track < TRACKS; track ++) {
    // Set the motor speeds, and turn each of them on (and off again)
    allSticks[track]->setSpeed(STRIKE_SPEED);
    allSticks[track]->run(FORWARD);
    allSticks[track]->run(RELEASE);
  }
  afterMeasure = true;
  // Set the base point into the past, so that 'now' will line up at the start
  // of the 'next' measure.  To avoid creeping time shifts, time reference points
  // will offset (and adjust) from this base time point.
  // This should (unsigned) wrap around backward here, then immediately wrap forward
  // again at the step to the start of the first measure
  measureBase = micros() - MEASURE_LENGTH;
  stateEnd[0] = 0;
}

void loop() {
  unsigned int track;
  unsigned long now = micros();

  if (afterMeasure) {
    if (now < stateEnd[0]) {
      // Just waiting for the end of the gap between measures
      return;
    }
    // Reached the end of the gap; start the measure, then fall through to the
    // processing done while a measure is in progress.
    startMeasure();
  }

  for (track = 0; track < TRACKS; track++) {
    if (now >= allTracks[track][nextBeat[track]]) {
      nextState(track);
    }
  }
}

void startMeasure() {
  unsigned int track;
  measureBase += MEASURE_LENGTH;// During a measure, time is referenced to this point
  afterMeasure = false;
  for (track = 0; track < TRACKS; track ++) {
    nextBeat[track] = 0; // point to first not time in the measure, for the track
    beatState[track] = HOME; // always start a measure ready to swing the stick
    // calculate the time point to step to the next state (play note)
    stateEnd[track] = measureBase + allTracks[track][nextBeat[track]];
    inMeasure[track] = 1;
  }
}

// Send the command(s) to change a stick to the next state in the cycle
void nextState(unsigned int track) {
  // Advance the stick to the next state in the cycle, for the current track
  beatState[track]++;
  if (beatState[track] > HOME) {
    beatState[track] = 0; //SWING
  }
  // get the (normal) time point for the end of the new state
  stateEnd[track] += STATE_TIME[beatState[track]];

  // Do the startup processing needed for the new state
  // NOTE: With the current implementation there should be no need to change the
  // speed.  That is set during initialization, and never changes.
  switch (beatState[track]) {
    case SWING:// 0
      allSticks[track]->run(FORWARD);
      break;
    case BOUNCE:// 1
      allSticks[track]->run(RELEASE);
      break;
    case RECOVER:// 2
      allSticks[track]->run(BACKWARD);
      break;
    case HOME:// 3
      allSticks[track]->run(RELEASE);

      // Figure out when the next swing should start (in the current measure)
      nextBeat[track] += 1;// index to the next note start time
      if (nextBeat[track] < sizeof(allTracks[track])) {
        stateEnd[track] = measureBase + allTracks[track][nextBeat[track]];
      } else {
        // final beat of this stick complete for the current measure
        stateEnd[track] = measureBase + MEASURE_LENGTH;
        inMeasure[track] = 0;// This track has no more beats in this measure
        if (tracksInMeasure() <= 0) {
          // no more beats on any track for the current measure
          afterMeasure = true;
        }
      }
      break;
  }
}

// Count the number of tracks that have not played all of the beats needed for
// the current measure.
unsigned int tracksInMeasure() {
  unsigned int activeTracks = 0;
  unsigned int track;
  for (track = 0; track < TRACKS; track++) {
    activeTracks += inMeasure[track];
  }
  return activeTracks;
}
