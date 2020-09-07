#include <FastLED.h>

#define LED_DATA 4
#define POT_BRIGHTNESS A0

#define NUM_LEDS 60

#define BTN_PIN A6
#define BRIGHTNESS 255
#define MAX_MODE 8
#define BTN_SAMPLE_PERIOD_MS 20


CRGB leds[NUM_LEDS]; 

uint8_t hue = 0;
uint8_t pulse_width = 3;
typedef enum {
  RED=0, 
  GREEN, 
  BLUE, 
  MAGENTA, 
  RAINBOW, 
  SCROLL, 
  SCROLLBOW,
  OFF
} STATE;
uint8_t num_states = 8;
STATE state = GREEN;

void setup()
{
 FastLED.addLeds<WS2812B,LED_DATA,GRB>(leds,NUM_LEDS);
 //FastLED.setCorrection(TypicalLEDStrip);
 //FastLED.setMaxPowerInVoltsAndMilliamps(5,4000);
 FastLED.setBrightness(255);
 Serial.begin(9600);                             // set the speed at which serial monitor will write at
 FastLED.showColor(CRGB::Black,255);
 delay(1000);                                      // saftey first
}

void loop() {
  FastLED.showColor(CRGB::Black,255);
  
  switch(state) {
    case RED:
      FastLED.showColor(CRGB::Red, BRIGHTNESS);
      break;
    case GREEN:
      FastLED.showColor(CRGB::Green, BRIGHTNESS);
      break;
    case BLUE:
      FastLED.showColor(CRGB::Blue, BRIGHTNESS);
      break;
    case MAGENTA:
      FastLED.showColor(CRGB::Magenta, BRIGHTNESS);
      break;
    case RAINBOW:
      fill_rainbow(leds, NUM_LEDS, 255/(NUM_LEDS-1));
      FastLED.show();
      break;
    case OFF:
      FastLED.showColor(CRGB::Black, 0);
      break;
  }
  Serial.println(state);
  delay(5000);
  uint8_t temp = (uint8_t) state;
  //temp++;
  temp %= num_states;
  state = (STATE)temp;
  
}
