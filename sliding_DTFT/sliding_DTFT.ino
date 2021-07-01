
#include <numeric>
#include <algorithm>
#include <ADC.h>
#include <vector>
#include "/home/joseph/kHz_Stabilizer/kHz_macros.h"

ADC* adc = new ADC();

uint16_t system_response_input_arr[sys_response_window];

int i, j;
int in[2] = { 0 , 0 };
int timeout = 0;

float drive_freqs[3];
uint16_t DACmax[2] = {0, 0};
int DAC_pins[2] = {A21, A22};
int read_pins[2] = {A9, A3};


bool switch_ = false;

uint16_t sine_sweep[6000];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A14, OUTPUT);

  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);

  analogWriteResolution(12);

  Serial.begin(115200);

  post_setup();

  int freqs[6] = { 60, 90, 120, 150, 180, 210 };

  for (j = 0; j < 6; ++j) {
    for (i = 0; i < 1000; ++i)
      sine_sweep[i + 1000 * j] = round(500 * (1 - cos(2 * PI * freqs[j] * i)));
  }
}

void post_setup() {

  while (true) {

    digitalWriteFast(LED_BUILTIN, LOW);

    Serial.clear();

    while (!Serial.available());

    switch (Serial.read()) {

      case FIND_RANGE:
        find_range();
        break;

      case LEARN_SYS_RESPONSE:
        learn_system_response();
        break;

      case STABILIZE:
        stabilize();
        break;

      case TEST_LOOP_TIME:
        test_loop_times();
        break;

      default:
        taper_down();

    }
  }
}

void loop() {

}

elapsedMicros t;

void test_loop_times() {

  delay(20);
  
  digitalWriteFast(LED_BUILTIN, HIGH);

  int t_start = 0;
  std::vector<int> loop_times(5000);
  std::vector<int> shot_number(5000);
  int* shot_ptr = shot_number.data();

  for (int k = 0; k < 5000; ++k) {

    t_start = t;
    digitalWriteFast(A14, HIGH);

    elapsedMicros m;
    while (!Serial.available()) {
      if (m > 5e6) {
        break;
      }
    }

    if (Serial.read() != SYNC_FLAG) {
      loop_times[k] = 0;
      Serial.clear();
    }

    else {

      Serial.readBytes((char*)shot_ptr, 4);
      analogWriteDAC1(0);
      analogWriteDAC0(0);
      loop_times[k] = t - t_start;

    }

    shot_ptr++;
    digitalWriteFast(A14, LOW);
    delay(1);

  }

  write_large_serial_buffer(loop_times, 128);
  write_large_serial_buffer(shot_number, 128);

}

void write_large_serial_buffer(std::vector<int> &buffer, int chunk_size) {

  const int tot_bytes = buffer.size() * sizeof(int);
  int tot_bytes_written = 0;
  byte* curr_pos = (byte*)buffer.data();

  while (tot_bytes_written < tot_bytes) {

    while (!Serial.available());

    if (Serial.read() == CONTINUE) {

      int bytes_written = 0;

      while (bytes_written != chunk_size) {

        bytes_written = Serial.write((const byte*)curr_pos, chunk_size);

        curr_pos += bytes_written;

        tot_bytes_written += bytes_written;

      }

    }

  }

}

void stabilize() {

  digitalWriteFast(LED_BUILTIN, HIGH);

  init_actuator();

  Serial.write((byte)CONTINUE);

  while (true) {

    elapsedMicros m;
    while (!Serial.available()) {
      if (m > 5e6) {
        taper_down();
        return;
      }
    }

    if (Serial.read() != SYNC_FLAG) {
      Serial.clear();
    }

    else {

      Serial.readBytes((char*)in, 4);
      analogWriteDAC1(in[0]);
    }

  }
}

void learn_system_response() {

  digitalWriteFast(LED_BUILTIN, HIGH);

  Serial.readBytes((char*)&DACmax[1], 2);

  generate_system_response_input(system_response_input_arr, DACmax[1], 20,300);

  init_actuator();

  Serial.clear();
  Serial.write((byte) CONTINUE);

  i = 0;

  while (i < sys_response_window) {

    elapsedMillis m;
    while (!Serial.available()) {
      if (m > 1000) {
        taper_down();
        return;
      }
    }

    if (Serial.read() != SYNC_FLAG) {
    }

    else {
      delayMicroseconds(5);
      //analogWriteDAC0(system_response_input_arr[i]);
      analogWriteDAC1(system_response_input_arr[i]);
      ++i;
    }
  }

  taper_down();
}


void find_range() {

  digitalWriteFast(LED_BUILTIN, HIGH);

  for (int i = 0; i < 2; ++i) {

    DACmax[i] = 0;
    int curr = 0;
    timeout = 0;

    while (true) {

      elapsedMillis m;
      while (!Serial.available()) {
        if (m > 2000) {
          taper_down();
          return;
        }
      }

      if (Serial.read() != SYNC_FLAG) {
        Serial.clear();
        Serial.write((byte)CONTINUE);
      }

      else {
        if (Serial.read() == STOP_FLAG) {
          DACmax[i] = curr - 5;
          Serial.write((byte)STOP_FLAG);
          Serial.write((const byte*)&DACmax[i], 2);
          break;
        }
        else {
          analogWrite(DAC_pins[i], curr);
          curr += 5;

          if (curr >= 4095) {
            DACmax[i] = 4095;
            Serial.write((byte)STOP_FLAG);
            Serial.write((const byte*)&DACmax[i], 2);
            break;
          }

          Serial.write((byte)CONTINUE);
        }
      }
    }

  }
  taper_down();
}

inline void taper_down() {

  for (int i = 0; i < 2; ++i) {
    int tapering_val = adc->adc0->analogRead(read_pins[i]);

    while (tapering_val > 0) {
      analogWrite(DAC_pins[i], --tapering_val);
      delayMicroseconds(100);
    }
  }
}

inline void init_actuator() {

  for (i = 0; i < 20; ++i) {
    //analogWriteDAC0(DACmax[0] / 20 * i);
    analogWriteDAC1(DACmax[1] / 20 * i);
    delay(1);
  }

  for (i = 0; i < 20; ++i) {
    //analogWriteDAC0(DACmax[0] - DACmax[0] / 20 * i);
    analogWriteDAC1(DACmax[1] - DACmax[1] / 20 * i);
    delay(1);
  }

  for (i = 0; i < 10; ++i) {
    //analogWriteDAC0(round((double)DACmax[0] / 2 / 20 * i));
    analogWriteDAC1(round((double)DACmax[1] / 2 / 20 * i));
    delay(1);
  }

}
