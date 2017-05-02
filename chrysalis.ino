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

#define DATA_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
  */
#define VOLTAGE_UP_PIN 13
#define VOLTAGE_READ_PIN A0

constexpr byte pixelCounts[] = {25, 18, 18, 15, 15, 13, 13, 9};

constexpr int sumHelper(const byte *a, const int N) {
  return a[0] + ((N > 1) ? sumHelper(a+1, N-1) : 0);
}

template<size_t N>
constexpr int sum(const byte (&arr)[N]) {
  return sumHelper(arr, N);
}

#define NUM_LEDS sum(pixelCounts)
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
CRGB leds[NUM_LEDS];

// in seconds
#define TOP_CRYSTAL_BPM 6
#define BOTTOM_CRYSTAL_BPM 5

// brightness - 0..255 range
#define TOP_CRYSTAL_IDLE_LOW 20
#define TOP_CRYSTAL_IDLE_HI 255

#define BOTTOM_CRYSTAL_LOW 20
#define BOTTOM_CRYSTAL_HI 100  

#define VOLTAGE_THRESHOLD 4
// slightly lower threshold to debounce via hysteresis
#define VOLTAGE_RAMPING_THRESHOLD 3.5

#define RAMP_UP_TIME  5000

void drawTopCrystalIdle(byte startPixel, byte length) {
  fill_solid(&leds[startPixel], length, CRGB::Red);
}

void drawRegularCrylstalIdle(byte startPixel, byte length) {
  fill_solid(&leds[startPixel], length, CRGB::DarkGreen);
}

void drawTopCrystalRampUp(byte startPixel, byte length) {
  fill_solid(&leds[startPixel], length, CRGB::White);
}

void drawIdle() {
  byte start = 0;
  byte index = 0;
  for(index = 0; index < sizeof(pixelCounts) - 1; index++) {
    byte length = pixelCounts[index];
    drawRegularCrylstalIdle(start, length);
    start += length;
  }
  drawTopCrystalIdle(start, pixelCounts[index]);
}


void drawRampUp(int offset) {
  byte start = 0;
  byte index = 0;
  for(index = 0; index < ARRAY_SIZE(pixelCounts ) - 1; index++) {
    byte length = pixelCounts[index];
    drawRegularCrylstalIdle(start, length);
    start += length;
  }
  drawTopCrystalRampUp(start, pixelCounts[index]);  
}

void drawTouching() {
  FastLED.clear();
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set the digital pins as outputs
  pinMode(VOLTAGE_UP_PIN, OUTPUT);

  delay(3000); // 3 second delay for recovery
  
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


float voltage = 0;
#define FILTER_STRENGTH 0.9



State currentState;
long transitionTime = 0;
long lastTime = 0; // for delta_t calculation

State transitionState() {
  switch(currentState) {
  case IDLE:
    // TODO: && !ramping_down
    if (voltage > VOLTAGE_THRESHOLD) {
      Serial.println("Touch started!");

      return RAMPING_UP;
    }
    // TODO: decrease charge according to time delta and given rate
    break;
  case RAMPING_UP:
    // TODO: increase charge, make transition to active according to charge, not time from start
    if (voltage < VOLTAGE_RAMPING_THRESHOLD) {
      // TOOD: charge -= CHARGE_LOSS_RATE * deltaT; if (charge <= 0) {charge = 0; return IDLE;} else {return RAMPING_UP;}
      Serial.println("Touch ended before trigger");
      return IDLE; // TODO:  ramping down
    } else if (millis() - transitionTime > RAMP_UP_TIME) {
      Serial.println("TRIGGER!");
      return ACTIVE;
    }
    // TOOD: charge += CHARGE_GAIN_RATE * deltaT; if (charge >= TRIGGER_THRESHOLD) {return ACTIVE;} else {return RAMPING_UP;}
    break;
  case ACTIVE:
    if (voltage < VOLTAGE_RAMPING_THRESHOLD) {
      Serial.println("Touch ended");
      return IDLE; // TODO: ramping down
    }
    break;
  default:
    Serial.println("crap!");
  }
  return currentState;
}

float charge = 0;
uint16_t phase = 0;


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
  State newState = transitionState();

  long currentTime = millis();
  if (newState != currentState) {
    transitionTime = currentTime;
    currentState = newState;
  }

  switch(currentState) {
    case IDLE:
      //phase += IDLE_FREQ*deltaT;
      drawIdle();
      break;
    case RAMPING_UP:
      //phase += (lerp16(charge, IDLE_FREQ, RAMP_UP_MAX_FREQ)*deltaT;
      drawRampUp(millis() - transitionTime);
      break;
    case ACTIVE:
      //phase =  ??
      drawTouching();
      break;
  }
  lastTime = currentTime;

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
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



