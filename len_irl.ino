#include <avr/pgmspace.h>
#include <Adafruit_NeoPixel.h>
#include "ffft.h"

// Which pin on the Arduino is connected to the NeoPixels
#define LED_PIN     6

// Unused
#define ADC_PIN  12

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT  60

#define NUM_ROWS 6
#define NUM_COLS (LED_COUNT/NUM_ROWS)

// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50 // Set BRIGHTNESS to about 1/5 (max = 255)

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


/* FFT stuff */
int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

byte
  peak[NUM_COLS],      // Peak level of each column; used for falling dots
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data
int
  col[NUM_COLS][10],   // Column levels for the prior 10 frames
  minLvlAvg[NUM_COLS], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[NUM_COLS], // pseudo rolling averages for the prior few frames.
  colDiv[NUM_COLS];    // Used when filtering FFT output to 8 columns

/** These values came from trial-and-error testing on music, speech, and background noise. */
static const uint8_t PROGMEM
  // low-level noise subtracted from each FFT output column
  noise[64] = { 32,24,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // scaling quotients for each FFT output column; music is pretty heavy at the bass end
  eq[64] = {
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first bin
    1,   16 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     12, 186,  38,   2 }, // Weights for 4 bins
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = { 4, 3,
     5, 55, 165, 164 },
  col4data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col5data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col6data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col7data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col8data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  col9data[] = { 5, 36,
     77,  60,  45,  34,  25 },
  // start of the data for each of the columns
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data,
    col8data, col9data };



void setup() {
    strip.begin();           // Init NeoPixel strip object
    strip.show();            // Turn off all pixels
    strip.setBrightness(BRIGHTNESS);

    uint8_t i, j, nBins, binNum, *data;

    memset(peak, 0, sizeof(peak));
    memset(col , 0, sizeof(col));

    for (i = 0; i < NUM_COLS; i++) {
        minLvlAvg[i] = 0;
        maxLvlAvg[i] = 512;
        data         = (uint8_t *)pgm_read_word(&colData[i]);
        nBins        = pgm_read_byte(&data[0]) + 2;
        binNum       = pgm_read_byte(&data[1]);
        for (colDiv[i] = 0, j = 2; j < nBins; j++)
            colDiv[i] += pgm_read_byte(&data[j]);
    }

    /** Unused
    / Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
    ADMUX  = ADC_PIN; // Channel sel, right-adj, use AREF pin
    ADCSRA = _BV(ADEN)  | // ADC enable
              _BV(ADSC)  | // ADC start
              _BV(ADATE) | // Auto trigger
              _BV(ADIE)  | // Interrupt enable
              _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
    ADCSRB = 0;                // Free run mode, no high MUX bit
    DIDR0  = 1 << ADC_PIN; // Turn off digital input for ADC pin
    TIMSK0 = 0;                // Timer0 off

    sei(); // Enable interrupts */
}

void loop() {
    uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
    uint16_t minLvl, maxLvl;
    int      level, y, sum;

    //while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish
    getSamples();


    for (x = 0; x<FFT_N; x++) {
        Serial.print(capture[x]);
        Serial.print(", ");
    }
    Serial.println();
    
    fft_input(capture, bfly_buff);   // Samples -> complex #s
    samplePos = 0;                   // Reset sample counter
    //ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
    fft_execute(bfly_buff);          // Process complex data
    fft_output(bfly_buff, spectrum); // Complex -> spectrum

    // Remove noise and apply EQ levels
    for (x = 0; x < FFT_N/2; x++) {
        L = pgm_read_byte(&noise[x]);
        spectrum[x] = (spectrum[x] <= L) ? 0 :
          (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
    }


    // Downsample spectrum output to NUM_COLS columns:
    for (x = 0; x < NUM_COLS; x++) {
        // Fill background w/colors, then idle parts of columns will erase
        //drawLineV(strip.Color(255, 255, 255), x, NUM_ROWS);

        data   = (uint8_t *)pgm_read_word(&colData[x]);
        nBins  = pgm_read_byte(&data[0]) + 2;
        binNum = pgm_read_byte(&data[1]);
        for (sum = 0, i = 2; i < nBins; i++)
            sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
        col[x][colCount] = sum / colDiv[x];                    // Average
        minLvl = maxLvl = col[x][0];
        for (i = 1; i < 10; i++) { // Get range of prior 10 frames
            if (col[x][i] < minLvl)      minLvl = col[x][i];
            else if (col[x][i] > maxLvl) maxLvl = col[x][i];
        }
        // minLvl and maxLvl indicate the extents of the FFT output, used
        // for vertically scaling the output graph (so it looks interesting
        // regardless of volume level).  If they're too close together though
        // (e.g. at very low volume levels) the graph becomes super coarse
        // and 'jumpy'...so keep some minimum distance between them (this
        // also lets the graph go to zero when no sound is playing)
        if ((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
        minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
        maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)

        // Second fixed-point scale based on dynamic min/max levels:
        level = (long)(NUM_ROWS) * (col[x][colCount] - minLvlAvg[x]) /
            (long)(maxLvlAvg[x] - minLvlAvg[x]);

        // Clip output and convert to byte:
        if (level < 0L)             c = 0;
        else if (level > NUM_ROWS)  c = NUM_ROWS; // Allow dot to go a couple pixels off top
        else                        c = (uint8_t)level;

        if (c > peak[x]) peak[x] = c; // Keep dot on top

        if (peak[x] <= 0) { // Empty column?
            drawLineV(strip.Color(0, 255, 0), x, 0);
            continue;
        } else if(c < NUM_ROWS) {
            drawLineV(strip.Color(0, 255, 0), x, c);
        } else {
            drawLineV(strip.Color(0, 255, 0), x, NUM_ROWS);
        }

        // the 'peak' dot is yellow colored
        y = NUM_ROWS - peak[x];
        drawMatPixel(strip.Color(255, 255, 0), x, y);
    }

    // Every third frame, make the peak pixels drop by 1
    if (++dotCount >= 3) {
        dotCount = 0;
        for (x = 0; x < NUM_COLS; x++) {
            if (peak[x] > 0) peak[x]--;
        }
    }

    if (++colCount >= 10) colCount = 0;
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, color);            //  Set pixel's color (in RAM)
        strip.show();                             //  Update strip to match
        delay(wait);                              //  Pause for a moment
    }
}

// Draws a line lengthwise along the whole strip.
void drawWholeLine(uint32_t color, uint32_t length) {
    for (int i = 0; i < length; ++i)
        strip.setPixelColor(i, color);
    for (int i = length; i < strip.numPixels(); ++i)
        strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
}

// In matrix configuration, draws a vertical line.
void drawLineV(uint32_t color, uint32_t x, uint32_t h) {
    /** NOTE: In our current configuration, cols are numbered backwards
        and every other row is backwards!
      -> . X . . .
         . X . . . <-
      -> . . . . .
      x = 3, h = 2
      pixels 1, 8 need to be turned on
    */
    x = NUM_COLS - 1 - x;
    for (int i = 0; i < h; ++i)
    {
        uint32_t raw_x;
        if (i % 2 == 0)
            raw_x = x + i * NUM_COLS;
        else
            raw_x = (NUM_COLS - 1 - x) + i * NUM_COLS;
        strip.setPixelColor(raw_x, color);
    }
    for (int i = h; i < NUM_ROWS; ++i)
    {
        uint32_t raw_x;
        if (i % 2 == 0)
            raw_x = x + i * NUM_COLS;
        else
            raw_x = (NUM_COLS - 1 - x) + i * NUM_COLS;
        strip.setPixelColor(raw_x, strip.Color(0, 0, 0));
    }
    strip.show();
}

void drawMatPixel(uint32_t color, uint32_t x, uint32_t y)
{
    x = NUM_COLS - 1 - x;
    y = NUM_ROWS - 1 - y;
    uint32_t raw_x;
    if (y % 2 == 0)
        raw_x = x + y * NUM_COLS;
    else
        raw_x = (NUM_COLS - 1 - x) + y * NUM_COLS;
    strip.setPixelColor(raw_x, color);
    strip.show();
}

void getSamples()
{
    static const int16_t noiseThreshold = 4;

    while (samplePos < FFT_N) {
        int16_t sample = analogRead(9);   // 0-255
        capture[samplePos] =
            ((sample > (128-noiseThreshold)) &&
            (sample < (128+noiseThreshold))) ? 0 :
            sample - 128; // Sign-convert for FFT; -256 to +255
            samplePos++;
    }
}

/** Unused */
ISR(ADC_vect) { // Audio-sampling interrupt
    static const int16_t noiseThreshold = 4;
    int16_t              sample         = ADC; // 0-1023

    capture[samplePos] =
        ((sample > (512-noiseThreshold)) &&
            (sample < (512+noiseThreshold))) ? 0 :
        sample - 512; // Sign-convert for FFT; -512 to +511

    if (++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}
