/*
  ESP32 I2S Microphone Sample
  Record audio from I2S microphone and play it back
  upon vibration trigger (SW420 sensor), saving and
  loading from SPIFFS.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include "SPIFFS.h"

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

// Additional pins
const int speakerPowerPin = 13;
const int microphonePowerPin = 14;

// Calculate total samples for RECORD_SECONDS at SAMPLE_RATE
static const size_t TOTAL_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;
// We'll use the same buffer size for loading from SPIFFS
static const size_t MAX_AUDIO_BYTES = TOTAL_SAMPLES * sizeof(int16_t);

// Dynamically allocated audio buffer for both recording and playback
int16_t* recordedData = nullptr;

// Track how many bytes of valid audio are recorded
size_t recordedSize = 0;  

// Vibration ISR flag
volatile bool vibrationDetected = false; 

// -------------------- FUNCTION PROTOTYPES --------------------
void saveAudioToSPIFFS(const char* filename, int16_t* data, size_t sizeInBytes);
bool loadAudioFromSPIFFS(const char* filename, int16_t* buffer, size_t maxBytes, size_t& actualBytesRead);

// -------------------- I2S SETUP --------------------
void setupI2SMic();
void setupI2SSpeaker();
void IRAM_ATTR vibrationISR();

// -------------------- SPIFFS SAVE/LOAD --------------------

// Write raw audio data to SPIFFS
void saveAudioToSPIFFS(const char* filename, int16_t* data, size_t sizeInBytes) {
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Error opening file for writing.");
    return;
  }
  size_t written = file.write((uint8_t*)data, sizeInBytes);
  file.close();

  if (written == sizeInBytes) {
    Serial.println("Audio successfully saved to SPIFFS.");
  } else {
    Serial.println("Failed to write all data to SPIFFS.");
  }
}

// Read raw audio data from SPIFFS into buffer
bool loadAudioFromSPIFFS(const char* filename, int16_t* buffer, size_t maxBytes, size_t &actualBytesRead) {
  File file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading.");
    actualBytesRead = 0;
    return false;
  }

  size_t fileSize = file.size();
  if (fileSize > maxBytes) {
    Serial.println("File is larger than buffer size!");
    file.close();
    actualBytesRead = 0;
    return false;
  }

  // Read the entire file into the buffer
  actualBytesRead = file.read((uint8_t*)buffer, fileSize);
  file.close();

  if (actualBytesRead == 0) {
    Serial.println("No data read from file.");
    return false;
  }

  Serial.printf("Loaded %u bytes from %s.\n", (unsigned)actualBytesRead, filename);
  return true;
}

// -------------------- I2S SETUP FUNCTIONS --------------------
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

// Interrupt for vibration sensor
void IRAM_ATTR vibrationISR() {
  vibrationDetected = true;  // Set flag when vibration occurs
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Configure pins
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(speakerPowerPin, OUTPUT);
  pinMode(microphonePowerPin, OUTPUT);

  pinMode(vibrationPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(vibrationPin), vibrationISR, FALLING);

  // Allocate audio buffer for up to 5 seconds of audio
  recordedData = (int16_t*)malloc(MAX_AUDIO_BYTES);
  if (!recordedData) {
    Serial.println("Error: Failed to allocate memory for recordedData.");
    while (1) { /* halt */ }
  }

  // Initialize I2S for microphone (RX) and speaker (TX)
  setupI2SMic();
  setupI2SSpeaker();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    while (true) { /* halt */ }
  }

  delay(500);
  Serial.println("Initialization Complete.");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // 1. Check if the switch is pressed -> Start recording
  if (digitalRead(switchPin) == LOW) {
    digitalWrite(microphonePowerPin, HIGH);
    Serial.println("Recording started...");

    size_t totalBytesToRead = MAX_AUDIO_BYTES; // 5s worth of 16-bit samples
    recordedSize = 0; // Reset recorded size
    uint32_t startTime = millis();

    while ((millis() - startTime) < (RECORD_SECONDS * 1000) &&
           recordedSize < totalBytesToRead) 
    {
      // Check if the button has been released
      if (digitalRead(switchPin) == HIGH) {
        digitalWrite(microphonePowerPin, LOW);
        Serial.println("Button released, stopping recording...");
        break; // Exit the loop immediately
      }

      size_t bytesRead = 0;
      // Read data from the microphone (with a short timeout)
      esp_err_t err = i2s_read(
        I2S_NUM_0,
        (char*)recordedData + recordedSize,
        totalBytesToRead - recordedSize,
        &bytesRead,
        10 // 10 ms timeout to avoid long blocking
      );

      if (err == ESP_OK && bytesRead > 0) {
        recordedSize += bytesRead;

        // Apply software gain to the *newly* read samples
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
    digitalWrite(microphonePowerPin, LOW);

    // Save the recorded audio to SPIFFS
    if (recordedSize > 0) {
      saveAudioToSPIFFS("/audio_data.bin", recordedData, recordedSize);
    }
    delay(500);
  }

  // 2. Check vibration sensor -> Playback from SPIFFS
  if (vibrationDetected) {
    Serial.println("Vibration detected, starting playback...");

    // Load audio from SPIFFS into recordedData
    size_t fileBytesRead = 0;
    bool loaded = loadAudioFromSPIFFS("/audio_data.bin",
                                      recordedData,
                                      MAX_AUDIO_BYTES,
                                      fileBytesRead);
    vibrationDetected = false;  // reset flag

    if (!loaded || fileBytesRead == 0) {
      Serial.println("No valid audio file to play.");
      return;
    }

    // Turn on speaker module
    digitalWrite(speakerPowerPin, HIGH);
    delay(100);

    // Re-init I2S speaker
    i2s_driver_uninstall(I2S_NUM_1);
    delay(50);
    setupI2SSpeaker();

    // Clear I2S buffers
    i2s_zero_dma_buffer(I2S_NUM_1);
    i2s_set_clk(I2S_NUM_1, SAMPLE_RATE, BITS_PER_SAMPLE, 
                (CHANNELS == 1) ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO);

    uint32_t startTime = millis();

    while ((millis() - startTime) < (PLAYBACK_SECONDS * 1000))
    {
      size_t bytesWritten = 0;
      // Write the entire buffer to I2S
      i2s_write(
        I2S_NUM_1,
        (const char*)recordedData,
        fileBytesRead,
        &bytesWritten,
        portMAX_DELAY
      );
      // If you only want to play once, break
      break;
    }

    Serial.println("Playback finished. Waiting...");
    digitalWrite(speakerPowerPin, LOW);
    delay(500); 
  }
}
