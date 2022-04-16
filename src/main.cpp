#include <EasyButton.h>
#include <Adafruit_I2CDevice.h>
#include <arduinoFFT.h>
#include <Arduino.h>
#include <FastLED_NeoMatrix.h>
#include <driver/i2s.h>
#include <EEPROM.h>
#define LED_PIN 15
#define AUDIO_IN_PIN 13
#define BUTTON_PIN 21
#define I2S_WS 26  // aka LRCL
#define I2S_SD 27  // aka DOUT
#define I2S_SCK 25 // aka BCLK
#define POTNOISEPIN
#define POTSENSITIVEPIN
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define SAMPLES 256         // Must be a power of 2
#define SAMPLING_FREQ 10240 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define NOISE 700           // Used as a crude noise filter, values below this are ignored
#define AMPLITUDE 900       // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define kMatrixWidth 18
#define kMatrixHeight 16
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
#define LAST_VISIBLE_LED 287
#define BLOCK_SIZE 64
#include <EEPROM.h>
#define EEPROM_SIZE 3
#define EEPROM_GAIN 0
#define EEPROM_SQUELCH 1
#define EEPROM_PATTERN 2
const i2s_port_t I2S_PORT = I2S_NUM_0;

byte brightness = 2;
byte displayTime;
bool micType = true;
byte pattern = 0;
byte colorTimer = 0;

byte gain = 10;   // Gain, boosts input level*/
byte squelch = 0; // Squelch, cuts out low level sounds
TaskHandle_t FFT_Task = NULL;
CRGB leds[NUM_LEDS];
FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, kMatrixWidth, kMatrixHeight,
                                                  NEO_MATRIX_TOP + NEO_MATRIX_LEFT +
                                                      NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE);

DEFINE_GRADIENT_PALETTE(outrun_gp){
    0, 141, 0, 100,   // purple
    127, 255, 192, 0, // yellow
    255, 0, 5, 255};  // blue
DEFINE_GRADIENT_PALETTE(redyellow_gp){
    0, 200, 200, 200,    // white
    64, 255, 218, 0,     // yellow
    128, 231, 0, 0,      // red
    192, 255, 218, 0,    // yellow
    255, 200, 200, 200}; // white
CRGBPalette16 outrunPal = outrun_gp;
CRGBPalette16 heatPal = redyellow_gp;

uint16_t micData;             // Analog input for FFT
uint16_t micDataSm;           // Smoothed mic data, as it's a bit twitchy
const uint16_t samples = 512; // This value MUST ALWAYS be a power of 2
unsigned int sampling_period_us;
unsigned long microseconds;
// Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
// Oh, and bins 0,1,2 are no good, so we'll zero them out.
double fftCalc[18];
int fftResult[18];       // Our calculated result table, which we feed to the animations.
double fftResultMax[18]; // A table used for testing to determine how our post-processing is working.
// Table of linearNoise results to be multiplied by squelch in order to reduce squelch across fftResult bins.
int linearNoise[18] = {34, 28, 26, 25, 20, 12, 34, 34, 34, 34, 34, 34, 34, 50, 50, 50, 70, 90};

// Table of multiplication factors so that we can even out the frequency response.
double fftResultPink[18] = {1.70, 1.71, 1.73, 1.78, 1.68, 1.56, 1.55, 1.63, 1.79, 1.62, 1.80, 2.06, 2.47, 3.35, 6.83, 9.55, 15, 20};
// Create FFT object

// Sampling and FFT stuff
int peak[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // The length of these arrays must be >= NUM_BANDS
int oldBarHeights[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int bandValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int barHeights[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t prevFFTValue[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// FFT stuff
double FFT_MajorPeak = 0;
double FFT_Magnitude = 0;
double fftBin[samples];
double vReal[samples];
double vImag[samples];
unsigned long newTime;
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
arduinoFFT FFTmic = arduinoFFT(vReal, vImag, samples, SAMPLING_FREQ);
EasyButton button(BUTTON_PIN);
double fftAdd(int from, int to)
{
    int i = from;
    double result = 0;
    while (i <= to)
    {
        result += fftBin[i++];
    }
    return (result / (to - from + 1));
}
// FFT main code
void FFTcode(void *parameter)
{

    for (;;)
    {
        delay(1); // DO NOT DELETE THIS LINE! It is needed to give the IDLE(0) task enough time and to keep the watchdog happy.
                  // taskYIELD(), yield(), vTaskDelay() and esp_task_wdt_feed() didn't seem to work.

        microseconds = micros();

        for (int i = 0; i < samples; i++)
        {
            int32_t digitalSample = 0;
            int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&digitalSample, portMAX_DELAY); // no timeout
            if (bytes_read > 0)
            {
                micData = abs(digitalSample >> 16);
            }

            // micDataSm = ((micData * 3) + micData)/4;  // We'll be passing smoothed micData to the volume routines as the A/D is a bit twitchy (not used here)
            vReal[i] = micData; // Store Mic Data in an array
            vImag[i] = 0;
            microseconds += sampling_period_us;
        }

        FFTmic.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Weigh data
        FFTmic.Compute(FFT_FORWARD);                        // Compute FFT
        FFTmic.ComplexToMagnitude();                        // Compute magnitudes
        //
        // vReal[3 .. 255] contain useful data, each a 20Hz interval (60Hz - 5120Hz).
        // There could be interesting data at bins 0 to 2, but there are too many artifacts.
        //
        FFTmic.MajorPeak(&FFT_MajorPeak, &FFT_Magnitude); // let the effects know which freq was most dominant

        for (int i = 0; i < samples; i++)
        { // Values for bins 0 and 1 are WAY too large. Might as well start at 3.
            double t = 0.0;
            t = abs(vReal[i]);
            t = t / 16.0; // Reduce magnitude. Want end result to be linear and ~4096 max.
            fftBin[i] = t;
        } // for()

        /* This FFT post processing is a DIY endeavour. What we really need is someone with sound engineering expertise to do a great job here AND most importantly, that the animations look GREAT as a result.
         *
         * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samples = 512 and some overlap.
         * Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of 5120 Hz in bin 255.
         * Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the end. Then detetermine the bins.
         * End frequency = Start frequency * multiplier ^ 16
         * Multiplier = (End frequency/ Start frequency) ^ 1/16
         * Multiplier = 1.320367784
         */

        // Range
        fftCalc[0] = (fftAdd(3, 6));      // 60 - 100
        fftCalc[1] = (fftAdd(6, 9));      // 80 - 120
        fftCalc[2] = (fftAdd(9, 11));     // 100 - 160
        fftCalc[3] = (fftAdd(11, 14));    // 140 - 200
        fftCalc[4] = (fftAdd(14, 17));    // 180 - 260
        fftCalc[5] = (fftAdd(17, 21));    // 240 - 340s
        fftCalc[6] = (fftAdd(21, 27));    // 320 - 440
        fftCalc[7] = (fftAdd(27, 35));    // 420 - 600
        fftCalc[8] = (fftAdd(35, 44));    // 580 - 760
        fftCalc[9] = (fftAdd(44, 48));    // 740 - 980
        fftCalc[10] = (fftAdd(48, 62));   // 960 - 1300
        fftCalc[11] = (fftAdd(62, 79));   // 1280 - 1700
        fftCalc[12] = (fftAdd(79, 99));   // 1680 - 2240
        fftCalc[13] = (fftAdd(99, 120));  // 2220 - 2960
        fftCalc[14] = (fftAdd(120, 141)); // 2940 - 3900
        fftCalc[15] = (fftAdd(141, 166)); // 3880 - 5120
        fftCalc[16] = (fftAdd(166, 200)); // 3880 - 5120
        fftCalc[17] = (fftAdd(200, 255)); // 3880 - 5120
        // Noise supression of fftCalc bins using squelch adjustment for different input types.
        for (int i = 0; i < kMatrixWidth; i++)
        {
            fftCalc[i] = fftCalc[i] - (float)squelch * (float)linearNoise[i] / 4.0 <= 0 ? 0 : fftCalc[i];
            fftCalc[i] = fftCalc[i] * fftResultPink[i];
            fftCalc[i] = fftCalc[i] * gain / 40 + fftCalc[i] / 16.0;
            fftResult[i] = constrain((int)fftCalc[i], 0, 254);
        }

    } // for(;;)
} // FFTcode()
void fftValueToBarHeights_MIC()
{
    for (byte band = 0; band < kMatrixWidth; band++)
    {

        uint8_t fftValue = fftResult[band];
        fftValue = ((prevFFTValue[band] * 3) + fftValue) / 4; // Dirty rolling average between frames to reduce flicker
        barHeights[band] = fftValue / (255 / kMatrixHeight);  // Scale bar height

        if (barHeights[band] > peak[band]) // Move peak up
            peak[band] = min(kMatrixHeight, (int)barHeights[band]);

        prevFFTValue[band] = fftValue; // Save prevFFTValue for averaging late\

    }
}
void setupAudio()
{

    // Attempt to configure INMP441 Microphone
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = SAMPLING_FREQ * 2,                  // 10240 * 2 (20480) Hz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,      // could only get it to work with 32bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // LEFT when pin is tied to ground.
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = 8,                       // number of buffers
        .dma_buf_len = BLOCK_SIZE                 // samples per buffer
    };
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK, // BCLK aka SCK
        .ws_io_num = I2S_WS,   // LRCL aka WS
        .data_out_num = -1,    // not used (only for speakers)
        .data_in_num = I2S_SD  // DOUT aka SD
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);

} // setupAudio()

// function for testmatrix
void DrawOneFrame(byte startHue8, int8_t yHueDelta8, int8_t xHueDelta8)
{
    byte lineStartHue = startHue8;
    for (byte y = 0; y < kMatrixHeight; y++)
    {
        lineStartHue += yHueDelta8;
        byte pixelHue = lineStartHue;
        for (byte x = 0; x < kMatrixWidth; x++)
        {
            pixelHue += xHueDelta8;
            matrix->drawPixel(x, y, CHSV(pixelHue, 255, 255));
        }
    }
}
void TestMatrix()
{
    uint32_t ms = millis();
    int32_t yHueDelta32 = ((int32_t)cos16(ms * (27 / 1)) * (350 / kMatrixWidth));
    int32_t xHueDelta32 = ((int32_t)cos16(ms * (39 / 1)) * (310 / kMatrixHeight));
    DrawOneFrame(ms / 65536, yHueDelta32 / 32768, xHueDelta32 / 32768);
    if (ms < 5000)
    {
        FastLED.setBrightness(scale8(brightness, (ms * 256) / 5000));
    }
    else
    {
        FastLED.setBrightness(brightness);
    }
    FastLED.show();
}
// void sanglanluot()
// {
//     for (uint16_t x = 0; x < kMatrixWidth; x++)
//     {
//         for (uint16_t y = 0; y < kMatrixHeight; y++)
//         {
//             matrix->drawPixel(x, y, CRGB(255, 255, 250));
//             FastLED.show();
//             delay(230);
//         }
//         delay(300);
//     }
//     FastLED.clear();
// }
// void DrawPattern1()
// {
//     FastLED.clear();
//     for (byte band = 0; band < kMatrixWidth; band++)
//     {
//         // rainbowBars
//         for (byte y = 0; y <= barHeights[band]; y++)
//         {
//             matrix->drawPixel(band, y, CHSV(band * 255 / kMatrixWidth, 255, 255));
//         }
//         // whitePeak
//         matrix->drawPixel(band, peak[band], CHSV(0, 0, 255));
//         // Decay peak
//         EVERY_N_MILLISECONDS(250)
//         {
//             for (byte band = 0; band < kMatrixWidth; band++)
//                 if (peak[band] > 0)
//                     peak[band] -= 1;
//         }
//         // Used in some of the patterns
//         EVERY_N_MILLISECONDS(10)
//         {
//             colorTimer++;
//         }
//     }
//     FastLED.show();
// }

// FFT AUX
void FFT_STUFF()
{
    // Reset bandValues[]
    for (int i = 0; i < kMatrixWidth; i++)
    {
        bandValues[i] = 0;
    }
    // Sample the audio pin
    for (int i = 0; i < SAMPLES; i++)
    {
        newTime = micros();
        vReal[i] = analogRead(AUDIO_IN_PIN); // A conversion takes about 9.7uS on an ESP32
        vImag[i] = 0;
        while ((micros() - newTime) < sampling_period_us)
        { /* chill */
        }
    }
    // Compute FFT
    FFT.DCRemoval();
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();
    FFT.MajorPeak(&FFT_MajorPeak, &FFT_Magnitude); // let the effects know which freq was most dominant
    // Analyse FFT results
    for (int i = 2; i < (SAMPLES / 2); i++)
    { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency bin and its value the amplitude.
        if (vReal[i] > NOISE)
        { // Add a crude noise filter
            // 18 bands, 6kHz top band 60hz low band 512 sample rate 10240
            if (i > 0 && i <= 3)
                bandValues[0] += (int)vReal[i];
            if (i > 3 && i <= 6)
                bandValues[1] += (int)vReal[i];
            if (i > 6 && i <= 9)
                bandValues[2] += (int)vReal[i];
            if (i > 9 && i <= 11)
                bandValues[3] += (int)vReal[i];
            if (i > 11 && i <= 14)
                bandValues[4] += (int)vReal[i];
            if (i > 14 && i <= 17)
                bandValues[5] += (int)vReal[i];
            if (i > 17 && i <= 21)
                bandValues[6] += (int)vReal[i];
            if (i > 21 && i <= 25)
                bandValues[7] += (int)vReal[i];
            if (i > 25 && i <= 29)
                bandValues[8] += (int)vReal[i];
            if (i > 29 && i <= 35)
                bandValues[9] += (int)vReal[i];
            if (i > 35 && i <= 42)
                bandValues[10] += (int)vReal[i];
            if (i > 42 && i <= 50)
                bandValues[11] += (int)vReal[i];
            if (i > 50 && i <= 60)
                bandValues[12] += (int)vReal[i];
            if (i > 60 && i <= 71)
                bandValues[13] += (int)vReal[i];
            if (i > 71 && i <= 83)
                bandValues[14] += (int)vReal[i];
            if (i > 83 && i <= 95)
                bandValues[15] += (int)vReal[i];
            if (i > 95 && i <= 111)
                bandValues[16] += (int)vReal[i];
            if (i > 111 && i <= 128)
                bandValues[17] += (int)vReal[i];
        }
    }
}
void fftValueToBarHeights_AUX()
{
    // Process the FFT data into bar heights
    for (byte band = 0; band < kMatrixWidth; band++)
    {
        // Scale the bars for the display
        barHeights[band] = bandValues[band] / AMPLITUDE;
        if (barHeights[band] > kMatrixHeight)
            barHeights[band] = kMatrixHeight;

        // Small amount of averaging between frames
        barHeights[band] = ((oldBarHeights[band] * 1) + barHeights[band] * 1) / 2;

        // Move peak up
        if (barHeights[band] > peak[band])
        {
            peak[band] = min(kMatrixHeight, (int)barHeights[band]);
        }
        // Save oldBarHeights for averaging later
        oldBarHeights[band] = barHeights[band];
    }
}

// drawPattern
void RainbowBars(byte band, byte barHeight)
{
    for (byte y = 0; y <= barHeight; y++)
    {
        // rainbowBars
        matrix->drawPixel(band, y, CHSV(band * 255 / kMatrixWidth, 255, 255));
    }
    // whitePeak
    matrix->drawPixel(band, peak[band], CHSV(0, 0, 255));
}
void changingBars(byte band, byte barHeight)
{

    for (byte y = 0; y <= barHeight; y++)
    {
        // changingBars
        matrix->drawPixel(band, y, CHSV(y * (255 / kMatrixHeight) + colorTimer, 255, 255));
    }
    // whitePeak
    matrix->drawPixel(band, peak[band], CHSV(0, 0, 255));
}
void outrunBars(byte band, byte barHeight)
{
    // no bars for this parttern
    //
    //
    // whitePeak
    matrix->drawPixel(band, peak[band], ColorFromPalette(outrunPal, peak[band] * (255 / kMatrixHeight)));
}
void centerBars(byte band, byte barHeight)
{
    if (barHeight % 2 == 0)
        barHeight--;
    int yStart = ((kMatrixHeight - barHeight) / 2);
    for (byte y = yStart; y <= yStart + barHeight; y++)
    {
        int colorIndex = constrain((y - yStart) * (255 / barHeight), 0, 255);
        matrix->drawPixel(band, y, ColorFromPalette(heatPal, colorIndex));
    }
}
void drawPatterns(byte band)
{
    byte barHeight = barHeights[band];
    switch (pattern % 4)
    {
    case 0:
        RainbowBars(band, barHeight);
        break;
    case 1:
        outrunBars(band, barHeight);
        break;
    case 2:
        changingBars(band, barHeight);
        break;
    case 3:
        centerBars(band, barHeight);
        break;
    }
}

void drawPatternsLoop()
{
    FastLED.clear();
    for (byte band = 0; band < kMatrixWidth; band++)
    {
        drawPatterns(band);
    }
    FastLED.show();
    // Decay peak
    EVERY_N_MILLISECONDS(150)
    {
        for (byte band = 0; band < kMatrixWidth; band++)
            if (peak[band] > 0)
                peak[band] -= 1;
    }
    EVERY_N_MILLISECONDS(10) { colorTimer++; }
    EVERY_N_SECONDS(30)
    {
        // Save values in EEPROM. Will only be commited if values have changed.
        EEPROM.write(EEPROM_GAIN, gain);
        EEPROM.write(EEPROM_SQUELCH, squelch);
        EEPROM.write(EEPROM_PATTERN, pattern);
        EEPROM.commit();
    }
    button.read();
}
void changeInput()
{
    micType = !micType;
    if (micType)
    {
        vTaskResume(FFT_Task);
        while (1)
        {
            fftValueToBarHeights_MIC();
            drawPatternsLoop();
            // Serial.println("dang trong loop mic");
        };
    }
    else
    {
        while (1)
        {
            vTaskSuspend(FFT_Task);
            FFT_STUFF();
            fftValueToBarHeights_AUX();
            drawPatternsLoop();
            // DrawPattern1();
            // button.update();
            // Serial.println("dang trong loop aux");
        }
    }
}
void changeMode()
{
    Serial.println("Button presseddsssssssssssssssssssssssssssssssssssssssssss");
    pattern++;
}

void changeGain()
{
    switch (gain)
    {
    case 0:
        gain = 5;
        break;
    case 5:
        gain = 10;
        break;
    case 10:
        gain = 15;
        break;
    case 15:
        gain = 0;
        break;
    }
    Serial.print("day la gain ");
    Serial.println(gain);
}

// void changeSquelch()
// {
//     switch (squelch)
//     {
//     case 0:
//         squelch = 10;
//         break;
//     case 10:
//         squelch = 20;
//         break;
//     case 20:
//         squelch = 30;
//         break;
//     case 30:
//         squelch = 0;
//         break;
//     }

//     Serial.print("day la squelch ");
//     Serial.println(squelch);
// }

void setup()
{

    //    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
    FastLED.setBrightness(brightness);
    Serial.begin(115200);
    FastLED.clear();
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
    delay(50);
    setupAudio();
    //    Define the FFT Task and lock it to core 0
    xTaskCreatePinnedToCore(
        FFTcode,   // Function to implement the task
        "FFT",     // Name of the task
        10000,     // Stack size in words
        NULL,      // Task input parameter
        1,         // Priority of the task
        &FFT_Task, // Task handle
        0);        // Core where the task should run
    // setup button
    // Initialize the button.
    button.begin();
    button.onPressed(changeMode);
    button.onPressedFor(2000, changeInput);
    button.onSequence(3, 1000, changeGain);
    // button.onSequence(5, 3000, changeSquelch);

    EEPROM.begin(EEPROM_SIZE);
    // It should not normally be possible to set the gain to 255
    // If this has happened, the EEPROM has probably never been written to
    // (new board?) so reset the values to something sane.
    if (EEPROM.read(EEPROM_GAIN) == 255)
    {
        EEPROM.write(EEPROM_GAIN, 0);
        EEPROM.write(EEPROM_SQUELCH, 0);
        EEPROM.write(EEPROM_PATTERN, 0);
        EEPROM.commit();
    }
    // Read saved values from EEPROM
    gain = EEPROM.read(EEPROM_GAIN);
    squelch = EEPROM.read(EEPROM_SQUELCH);
    pattern = EEPROM.read(EEPROM_PATTERN);
}
void loop()
{
    TestMatrix();
    button.read();
    // Serial.println("dang trong loop xin");
}
