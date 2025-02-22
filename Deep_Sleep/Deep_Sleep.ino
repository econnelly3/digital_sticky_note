/*
  ESP32 I2S Microphone + Speaker
  Deep Sleep with EXT0 and EXT1 Wake-up
  - Wake on vibrationPin (GPIO4) going LOW => Playback
  - Wake on switchPin (GPIO27) going LOW => Record
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include "SPIFFS.h"
#include "esp_sleep.h"

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

// Pins for wake-up
static const int switchPin     = 27; // Button => EXT1 => wake on LOW
static const int vibrationPin  = 4;  // SW420 => EXT0 => wake on LOW

// Additional pins
static const int speakerPowerPin    = 13;
static const int microphonePowerPin = 14;

// Calculate total samples for RECORD_SECONDS at SAMPLE_RATE
static const size_t TOTAL_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;
static const size_t MAX_AUDIO_BYTES = TOTAL_SAMPLES * sizeof(int16_t);

// Audio buffer
int16_t* audioBuffer = nullptr;

// -------------------- FORWARD DECLARATIONS --------------------
void setupI2SMic();
void setupI2SSpeaker();
void recordAudio();
void playbackAudioFromSPIFFS();
void saveAudioToSPIFFS(const char* filename, int16_t* data, size_t sizeInBytes);
bool loadAudioFromSPIFFS(const char* filename, int16_t* buffer, size_t maxBytes, size_t &actualBytesRead);
void enterDeepSleep();

// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== ESP32 STARTUP =====");

  // Allocate memory for audio
  audioBuffer = (int16_t*)malloc(MAX_AUDIO_BYTES);
  if (!audioBuffer) {
    Serial.println("Error: Failed to allocate memory for audio buffer.");
    while (true) { /* halt */ }
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    while (true) { /* halt */ }
  }

  // Configure pins
  // If your hardware is pull-up for the button, pressing it goes LOW:
  pinMode(switchPin,    INPUT_PULLUP); 
  pinMode(vibrationPin, INPUT_PULLUP); 
  pinMode(speakerPowerPin, OUTPUT);
  pinMode(microphonePowerPin, OUTPUT);

  // Determine the reason for wake
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

  // Act based on wake reason
  switch (wakeReason) {
    case ESP_SLEEP_WAKEUP_EXT0: {
      // EXT0 => triggered by vibrationPin => playback
      Serial.println("Wake-up from EXT0 (vibration sensor). Playing audio...");
      setupI2SSpeaker();
      playbackAudioFromSPIFFS();
      break;
    }
    case ESP_SLEEP_WAKEUP_EXT1: {
      // EXT1 => triggered by switchPin => record
      Serial.println("Wake-up from EXT1 (button). Recording audio...");
      setupI2SMic();
      setupI2SSpeaker();
      recordAudio();
      saveAudioToSPIFFS("/audio_data.bin", audioBuffer, MAX_AUDIO_BYTES);
      break;
    }
    default: {
      // Fresh boot or other wake
      Serial.println("Fresh boot or unknown wake-up. No immediate action.");
      break;
    }
  }

  // Enter deep sleep again
  enterDeepSleep();
}

void loop() {
  // Not used; we go to deep sleep from setup().
}

// -------------------------------------------------------------
//               DEEP SLEEP SETUP
// -------------------------------------------------------------
void enterDeepSleep() {
  Serial.println("Setting up wake sources...");

  // EXT0 wake-up on vibrationPin going LOW (level = 0)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)vibrationPin, 0);

  // EXT1 wake-up on switchPin going LOW => "ALL_LOW" mode
  // (We only specify one pin, so if that pin goes LOW, it will wake.)
  uint64_t wakePinMask = (1ULL << switchPin);
  esp_sleep_enable_ext1_wakeup(wakePinMask, ESP_EXT1_WAKEUP_ALL_LOW);

  Serial.println("Entering deep sleep now...");
  delay(100); // small delay to flush Serial
  esp_deep_sleep_start();
}

// -------------------------------------------------------------
//              I2S and AUDIO LOGIC
// -------------------------------------------------------------
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

// Record up to RECORD_SECONDS or until the button is released
void recordAudio() {
  digitalWrite(microphonePowerPin, HIGH);
  digitalWrite(speakerPowerPin, LOW);   // Speaker off while recording
  Serial.println("Recording started...");

  size_t totalBytesToRead = MAX_AUDIO_BYTES;
  size_t recordedBytes = 0;
  uint32_t startTime = millis();

  while ((millis() - startTime) < (RECORD_SECONDS * 1000) &&
         recordedBytes < totalBytesToRead) 
  {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(
      I2S_NUM_0,
      (char*)audioBuffer + recordedBytes,
      totalBytesToRead - recordedBytes,
      &bytesRead,
      50 // short timeout
    );

    if (err == ESP_OK && bytesRead > 0) {
      // Amplify
      size_t samplesRead = bytesRead / sizeof(int16_t);
      for (size_t i = 0; i < samplesRead; i++) {
        int index = (recordedBytes / sizeof(int16_t)) + i;
        int32_t amplified = audioBuffer[index] * GAIN_FACTOR;
        if (amplified > 32767)  amplified = 32767;
        if (amplified < -32768) amplified = -32768;
        audioBuffer[index] = (int16_t)amplified;
      }
      recordedBytes += bytesRead;
    }
    // If the user releases the button => stop recording early
    if (digitalRead(switchPin) == HIGH) {
      break;
    }
  }

  digitalWrite(microphonePowerPin, LOW);
  Serial.print("Recording finished. Bytes recorded: ");
  Serial.println(recordedBytes);
}

// Play from SPIFFS
void playbackAudioFromSPIFFS() {
  digitalWrite(microphonePowerPin, LOW);
  digitalWrite(speakerPowerPin, HIGH);
  Serial.println("Loading audio data and playing...");

  // Re-init speaker
  i2s_driver_uninstall(I2S_NUM_1);
  delay(20);
  setupI2SSpeaker();
  i2s_zero_dma_buffer(I2S_NUM_1);

  // Load audio
  size_t fileBytesRead = 0;
  bool loaded = loadAudioFromSPIFFS("/audio_data.bin",
                                    audioBuffer,
                                    MAX_AUDIO_BYTES,
                                    fileBytesRead);

  if (!loaded || fileBytesRead == 0) {
    Serial.println("Failed to load valid audio from SPIFFS.");
    digitalWrite(speakerPowerPin, LOW);
    return;
  }

  Serial.print("Playing for ");
  Serial.print(PLAYBACK_SECONDS);
  Serial.println(" seconds.");

  uint32_t startTime = millis();
  while ((millis() - startTime) < (PLAYBACK_SECONDS * 1000)) {
    size_t bytesWritten;
    i2s_write(I2S_NUM_1,
              (const char*)audioBuffer,
              fileBytesRead,
              &bytesWritten,
              portMAX_DELAY);
    // If you only want to play once, break after one pass
    break;
  }

  Serial.println("Playback finished.");
  digitalWrite(speakerPowerPin, LOW);
}

// -------------------- SPIFFS IO --------------------
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

  actualBytesRead = file.read((uint8_t*)buffer, fileSize);
  file.close();

  if (actualBytesRead == 0) {
    Serial.println("No data read from file.");
    return false;
  }

  Serial.printf("Loaded %u bytes from %s\n", (unsigned)actualBytesRead, filename);
  return true;
}
