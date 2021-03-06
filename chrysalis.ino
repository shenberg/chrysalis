#include <bitswap.h>
#include <chipsets.h>
#include <color.h>
#include <colorpalettes.h>
#include <colorutils.h>
#include <controller.h>
#include <cpp_compat.h>
#include <dmx.h>
#include <FastLED.h>
#include <fastled_config.h>
#include <fastled_delay.h>
#include <fastled_progmem.h>
#include <fastpin.h>
#include <fastspi.h>
#include <fastspi_bitbang.h>
#include <fastspi_dma.h>
#include <fastspi_nop.h>
#include <fastspi_ref.h>
#include <fastspi_types.h>
#include <hsv2rgb.h>
#include <led_sysdefs.h>
#include <lib8tion.h>
#include <noise.h>
#include <pixelset.h>
#include <pixeltypes.h>
#include <platforms.h>
#include <power_mgt.h>

#include "FastLed.h"

#include "states.h"

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define NO_INTERACTION_MODE
//#define NO_TOP
#define DATA_PIN    3
#define TOP_CRYSTAL_DATA_PIN 4
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

#define BRIGHTNESS          255
//#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

// enable test pattern by commenting-in
//#define TEST_PATTERN

/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
  */
#define VOLTAGE_UP_PIN 13
#define VOLTAGE_INDICATOR_PIN 6
#define VOLTAGE_READ_PIN A0

#ifdef NO_TOP
#define TOP_CRYSTAL_SIZE CRYSTALS_SIZE
#else
// number of pixels
#define TOP_CRYSTAL_SIZE 9
#endif
// for all regular crystals
constexpr byte pixelCounts[] = {25, 18, 18, 15, 15, 13, 13};

// constexpr doesn't do looks
constexpr int sumHelper(const byte *a, const int N) {
  return a[0] + ((N > 1) ? sumHelper(a+1, N-1) : 0);
}

template<size_t N>
constexpr int sum(const byte (&arr)[N]) {
  return sumHelper(arr, N);
}

#define CRYSTALS_SIZE sum(pixelCounts)
#define NUM_LEDS (CRYSTALS_SIZE + TOP_CRYSTAL_SIZE)
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define CRYSTAL_COUNT ARRAY_SIZE(pixelCounts)
//CRGB leds[NUM_LEDS];
CRGBArray<NUM_LEDS> leds; // crystals except top
#ifdef NO_TOP
#define topCrystal leds
#else
CRGBSet topCrystal = leds(CRYSTALS_SIZE, NUM_LEDS - 1); 
#endif
// in seconds
#define TOP_CRYSTAL_BPM 6
#define BOTTOM_CRYSTAL_BPM 5

// brightness - 0..255 range
#define TOP_CRYSTAL_IDLE_LOW 20
#define TOP_CRYSTAL_IDLE_HI 255

#define BOTTOM_CRYSTAL_LOW 20
#define BOTTOM_CRYSTAL_HI 100  

#define VOLTAGE_THRESHOLD 2.5
// slightly lower threshold to debounce via hysteresis
#define VOLTAGE_RAMPING_THRESHOLD 2

#define RAMP_UP_TIME  5000

// 10 seconds for 1 beat, like a deep breath
#define IDLE_FREQ (65536/10)
#define RAMP_UP_FREQ (65536*5)
// charge loss of entire charge in 2 second
#define MAX_CHARGE (100*256)
#define CHARGE_LOSS_RATE (MAX_CHARGE/10)
#define CHARGE_GAIN_RATE (MAX_CHARGE/20)
#define TRIGGER_THRESHOLD MAX_CHARGE


void drawTopCrystalIdle(byte startPixel, byte length) {
  fill_solid(&leds[startPixel], length, CRGB::Red);
}

void drawRegularCrylstalIdle(byte startPixel, byte length) {
  // TODO: test pattern 1
  //fill_solid(&leds[startPixel], length, CRGB::DarkGreen);
  // TODO: test pattern 2 - different color per crystal
  fill_solid(&leds[startPixel], length, CHSV(startPixel*2, 255, 255));
}

void drawTopCrystalRampUp(byte startPixel, byte length) {
  fill_solid(&leds[startPixel], length, CRGB::White);
}

void drawTouching() {
  FastLED.clear();
}


#define IDLE_MIN_BRIGHTNESS 0
#define IDLE_MAX_BRIGHTNESS 255
#define IDLE_MAX_CHARGE_BRIGHTNESS 90

void drawIdle(uint16_t phase, uint16_t charge) {
  byte start = 0;
  static uint16_t last_phase = phase;
  static byte t = 0;
  if (last_phase > phase) {
    t++;
  }
  last_phase = phase;

#ifdef TEST_PATTERN
  for(byte index = 0; index < CRYSTAL_COUNT; index++) {
    byte length = pixelCounts[index];
    // TODO: clear pixels instead?
    drawRegularCrylstalIdle(start, length);
    start += length;
  }
#endif

  if (charge >= MAX_CHARGE) {
    charge = MAX_CHARGE - 1;
  }
  byte chargeBrightness = (long)charge * 256 / MAX_CHARGE;

  CRGB pulseColor = CRGB::Red;
  fract16 animationPosition = sin16(phase) + 32767; // convert to range 0..65534
  //Serial.println(animationPosition);
  //Serial.println(lerp8by8(IDLE_MIN_BRIGHTNESS, IDLE_MAX_BRIGHTNESS, animationPosition >> 8));
  pulseColor.nscale8(lerp8by8(IDLE_MIN_BRIGHTNESS, IDLE_MAX_BRIGHTNESS, animationPosition >> 8));
  //Serial.println(pulseColor.r);
  CRGB chargeColor(0,255,255);
  chargeColor.nscale8(lerp8by8(0, IDLE_MAX_CHARGE_BRIGHTNESS, chargeBrightness));
  
#ifdef NO_INTERACTION_MODE
  byte hue = t*16 + (phase >> 14);
  leds.fill_solid(CHSV(hue, 255, 255));
  topCrystal.fill_solid(CHSV(hue + 128, 255, 255));
#else
  // draw the top crystal
  //topCrystal.fill_solid(pulseColor + chargeColor);
  topCrystal.fill_solid(ColorFromPalette(PartyColors_p, t, lerp8by8(IDLE_MIN_BRIGHTNESS, IDLE_MAX_BRIGHTNESS, animationPosition >> 8)));
#endif
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set the digital pins as outputs
  pinMode(VOLTAGE_UP_PIN, OUTPUT);
  pinMode(VOLTAGE_INDICATOR_PIN, OUTPUT);

  delay(3000); // 3 second delay for recovery
  
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, CRYSTALS_SIZE).setCorrection(TypicalLEDStrip);
  #ifndef NO_TOP
  FastLED.addLeds<LED_TYPE,TOP_CRYSTAL_DATA_PIN,COLOR_ORDER>(leds, TOP_CRYSTAL_SIZE).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,TOP_CRYSTAL_DATA_PIN,COLOR_ORDER>(&leds[CRYSTALS_SIZE], TOP_CRYSTAL_SIZE).setCorrection(TypicalLEDStrip);
  #endif
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);


  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


float voltage = 0;
#define FILTER_STRENGTH 0.9
//#define FILTER_STRENGTH 0



State currentState;
long transitionTime = 0;
long lastTime = 0; // for delta_t calculation

// signed 7.8 fixed point int
saccum78 charge = 0;
// unsigned 16 bit fraction
fract16 phase = 0;

State transition(uint16_t deltaT) {
  switch(currentState) {
  case IDLE:
    // TODO: && !ramping_down
    if (voltage > VOLTAGE_THRESHOLD) {
      Serial.println("Touch started!");
      return RAMPING_UP;
    }
    if (charge > 0) {
      // note: assuming deltaT never > 255 ms. seems reasonable;
      charge -= lerp16by8(0, CHARGE_LOSS_RATE, deltaT);
      if (charge < 0) {
        charge = 0;
      }
    }
    break;
  case RAMPING_UP:
    // TODO: increase charge, make transition to active according to charge, not time from start
    if (voltage < VOLTAGE_RAMPING_THRESHOLD) {
      Serial.println("Touch ended before trigger");
      return IDLE; // TODO:  ramping down
    } else {
      // note: assuming deltaT never > 255 ms. seems reasonable;
      //Serial.println(charge);
      charge += lerp16by8(0, CHARGE_GAIN_RATE, deltaT);
      if (charge >= TRIGGER_THRESHOLD) {
        charge = TRIGGER_THRESHOLD;
        Serial.println("TRIGGER!");
        //TODO:return after debugging pulse
        return ACTIVE;
      }
    }
    break;
  case ACTIVE:
      Serial.println("activate done");
      charge = 0;
      return IDLE; 
  default:
    Serial.println("crap!");
  }
  return currentState;
}

#define ENERGY_DOWN_LENGTH 1500
#define LIGHT_UP_LENGTH 3000
#define LIT_LENGTH 5000
#define FADE_LENGTH 5000

#define START_FRAME_COUNT 50
void activeAnimation() {

  // animate to known state: full brightness on top, quickly
  for(int i = 0; i < START_FRAME_COUNT; i++) {
    topCrystal.addToRGB(255/(START_FRAME_COUNT-1));
    delay(1000/FRAMES_PER_SECOND);
  }
  // animate moving the energy down
  long startTime = millis();
  long currentTime = startTime;
  topCrystal.fill_solid(CRGB::White);
  // set all pixels to white, while constatly darkening,
  // keep a smaller and smaller set at the bottom constantly white.
  while (currentTime - startTime < ENERGY_DOWN_LENGTH) {
    fadeToBlackBy(topCrystal, TOP_CRYSTAL_SIZE, 7);

    // note: -1 for round-down in division which becomes round up
    byte relativePosition = TOP_CRYSTAL_SIZE - 1 - (currentTime - startTime)*TOP_CRYSTAL_SIZE/ENERGY_DOWN_LENGTH;
    topCrystal(0,relativePosition) = CRGB::White;
    FastLED.show();
    delay(1000/FRAMES_PER_SECOND);
    currentTime = millis();
  }
  
  Serial.println("lighting up");
  startTime = currentTime;
  long dt;
  // TODO: ugh dt
  while ((dt = (currentTime - startTime)) < LIGHT_UP_LENGTH) {
    fadeToBlackBy(leds, NUM_LEDS, 7);
    // for every non-top crystal, draw blob traveling down
    byte pixel = 0;
    for (byte i = 0; i < CRYSTAL_COUNT; i++) {
      // fixed 8.8 fractional position
      accum88 relativePosition = 256*(pixel + pixelCounts[i] - 1) - (dt*(pixelCounts[i]*256) / LIGHT_UP_LENGTH);
      byte upperPixel = relativePosition >> 8;
      if (upperPixel <= pixel + pixelCounts[i] - 1) {
        leds[upperPixel].addToRGB(relativePosition & 0xff);
      }
      if (upperPixel - 1 >= pixel) {
        leds[upperPixel - 1].setRGB(255 - (relativePosition & 0xff), 255 - (relativePosition & 0xff), 255 - (relativePosition & 0xff));
      }

      // go to next crystal
      pixel += pixelCounts[i];
    }
    FastLED.show();
    delay(1000/FRAMES_PER_SECOND);
    currentTime = millis();
  }
  Serial.println("done lighting up");
  // animate crystals lighting up
  // animate lit crystals
  startTime = currentTime;
  while (currentTime - startTime < LIT_LENGTH) {
    rainbowWithGlitter();
    gHue++;
    FastLED.show();
    delay(1000/FRAMES_PER_SECOND);
    currentTime = millis();
  }
  // fade to idle
  startTime = currentTime;
  while (currentTime - startTime < FADE_LENGTH) {
    rainbowWithGlitter();
    gHue++;
    byte relativePosition = max(0, 255 - (currentTime - startTime)*255/ENERGY_DOWN_LENGTH);
    leds.nscale8(relativePosition);
    FastLED.show();
    delay(1000/FRAMES_PER_SECOND);
    currentTime = millis();
  }
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int pot = analogRead(VOLTAGE_READ_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // use 1-tap IIR filter: output = last_out*strength + (1-strength) * input
  // works very well in practice
  voltage = voltage * FILTER_STRENGTH + (1 - FILTER_STRENGTH) * pot * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(voltage);
  //Serial.println(charge);
  digitalWrite(VOLTAGE_INDICATOR_PIN, (voltage > VOLTAGE_THRESHOLD ? HIGH : LOW));
  /*
  // TODO: remove
  if (voltage > VOLTAGE_THRESHOLD) {
    leds.fill_solid(CRGB::White);
  } else {
    leds.fill_solid(CRGB::Green);
  }
  FastLED.show();
  return;*/
  long currentTime = millis();

  uint16_t deltaT = currentTime - lastTime;
  State newState = transition(deltaT);
  if (newState != currentState) {
    transitionTime = currentTime;
    currentState = newState;
  }
  
  switch (currentState) {
  case IDLE:
  case RAMPING_UP:
  {
    // TODO: do fast-math verion of this
    float t = charge/(float)MAX_CHARGE;
    t = t*t; // nonlinear
    //Serial.println((t*RAMP_UP_FREQ + (1.f-t)*IDLE_FREQ) * (deltaT/1000.f));
    phase += (t*RAMP_UP_FREQ + (1.f-t)*IDLE_FREQ) * (deltaT/1000.f);
    //Serial.println(phase);
    drawIdle(phase, charge);
    break;
  }
  case ACTIVE:
    activeAnimation();
    break;
  }

  lastTime = currentTime;

  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  // do regular delay so we don't screw over pc<->arduino comms due to interrupts
  //FastLED.delay(1000/FRAMES_PER_SECOND); 
  delay(1000/FRAMES_PER_SECOND);

  // do some periodic updates
  //EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}



