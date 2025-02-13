/*
  ESP32 I2S Microphone Sample
  Record audio from I2S microphone and play it back
  upon vibration trigger (SW420 sensor).
*/

#include <Arduino.h>
#include <driver/i2s.h>

// -------------------- CONFIGURATIONS --------------------
#define SAMPLE_RATE       10000          // Adjust as needed
#define CHANNELS          1              // Mono audio
#define BITS_PER_SAMPLE   I2S_BITS_PER_SAMPLE_16BIT
#define RECORD_SECONDS    5              // Record duration
#define PLAYBACK_SECONDS  5              // Playback duration
#define GAIN_FACTOR       30             // Software gain factor

// I2S pins for microphone (I2S_NUM_0)
#define I2S0_WS   21   // LRCK
#define I2S0_SD   33   // DOUT from mic
#define I2S0_SCK  32   // BCLK

// I2S pins for speaker (I2S_NUM_1)
#define I2S1_WS   25   // LRCK
#define I2S1_SD   22   // DIN to speaker
#define I2S1_SCK  26   // BCLK

// User-controlled switch (for recording)
const int switchPin = 5; 

// Vibration sensor pin (for playback)
const int vibrationPin = 4;  

// Calculate total samples for RECORD_SECONDS at SAMPLE_RATE
static const size_t TOTAL_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;

// Dynamically allocated audio buffer for recording
int16_t* recordedData = nullptr;

// Track how many bytes of valid audio are recorded
size_t recordedSize = 0;  

// -------------------- I2S SETUP --------------------
volatile bool vibrationDetected = false; // Flag for vibration

// Configure I2S for microphone (RX)
void setupI2SMic() {
  i2s_config_t i2s_config_mic = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = BITS_PER_SAMPLE,
    .channel_format = (CHANNELS == 1) ? I2S_CHANNEL_FMT_ONLY_LEFT : I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_mic = {
    .bck_io_num = I2S0_SCK,
    .ws_io_num = I2S0_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S0_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config_mic, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config_mic);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, BITS_PER_SAMPLE, 
              (CHANNELS == 1) ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO);
}

// Configure I2S for speaker (TX)
void setupI2SSpeaker() {
  i2s_config_t i2s_config_speaker = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = BITS_PER_SAMPLE,
    .channel_format = (CHANNELS == 1) ? I2S_CHANNEL_FMT_ONLY_LEFT : I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_speaker = {
    .bck_io_num = I2S1_SCK,
    .ws_io_num = I2S1_WS,
    .data_out_num = I2S1_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_1, &i2s_config_speaker, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config_speaker);
  i2s_set_clk(I2S_NUM_1, SAMPLE_RATE, BITS_PER_SAMPLE, 
              (CHANNELS == 1) ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO);
}
void IRAM_ATTR vibrationISR() {
  vibrationDetected = true;  // Set flag when vibration occurs
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Configure pins
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(vibrationPin, INPUT_PULLUP); // SW420 typically uses a simple digital pin
  attachInterrupt(digitalPinToInterrupt(vibrationPin), vibrationISR, FALLING);  // Detect LOW signal

  // Allocate audio buffer dynamically (for 5 seconds of audio)
  recordedData = (int16_t*)malloc(TOTAL_SAMPLES * sizeof(int16_t));
  if (!recordedData) {
    Serial.println("Error: Failed to allocate memory for recordedData.");
    while (1) { /* halt */ }
  }

  // Initialize I2S for microphone (RX) and speaker (TX)
  setupI2SMic();
  setupI2SSpeaker();

  delay(500);
  Serial.println("Initialization Complete.");
}

void loop() {
  // 1. Check if the switch is pressed -> Start recording
  if (digitalRead(switchPin) == LOW) {
    Serial.println("Recording started...");

    size_t totalBytesToRead = TOTAL_SAMPLES * sizeof(int16_t);
    recordedSize = 0; // Reset recorded size
    uint32_t startTime = millis();

    while ((millis() - startTime) < (RECORD_SECONDS * 1000) &&
           recordedSize < totalBytesToRead) 
    {
      // Check if the button has been released
      if (digitalRead(switchPin) == HIGH) {
        Serial.println("Button released, stopping recording...");
        break; // Exit the loop immediately
      }

      size_t bytesRead = 0;
      // Read data from the microphone (non-blocking behavior)
      esp_err_t err = i2s_read(
        I2S_NUM_0,
        (char*)recordedData + recordedSize,
        totalBytesToRead - recordedSize,
        &bytesRead,
        10 // Timeout for 10 ms to avoid long blocking
      );

      if (err == ESP_OK && bytesRead > 0) {
        recordedSize += bytesRead;

        // Apply software gain to new samples only
        size_t samplesRead = bytesRead / sizeof(int16_t);
        for (size_t i = 0; i < samplesRead; ++i) {
          int sampleIndex = (recordedSize / sizeof(int16_t)) - samplesRead + i;
          int32_t amplified = recordedData[sampleIndex] * GAIN_FACTOR;

          // Clip to valid 16-bit range
          if (amplified > 32767)  amplified = 32767;
          if (amplified < -32768) amplified = -32768;

          recordedData[sampleIndex] = (int16_t)amplified;
        }
      }
    }

    Serial.println("Recording finished.");
    delay(500); // Small pause
  }
  // 2. Check vibration sensor -> Playback
  if (vibrationDetected && recordedSize > 0) {
    Serial.println("Vibration detected, starting playback...");

    uint32_t startTime = millis();
    size_t bytesWritten = 0;

    while ((millis() - startTime) < (PLAYBACK_SECONDS * 1000) &&
           digitalRead(vibrationPin) == HIGH)
    {
      i2s_write(
        I2S_NUM_1,
        (const char*)recordedData,
        recordedSize,
        &bytesWritten,
        portMAX_DELAY
      );
    }

    Serial.println("Playback finished or vibration stopped. Waiting...");
    vibrationDetected = false;  // Reset flag after printing
    delay(500); // Small pause
  }
}
