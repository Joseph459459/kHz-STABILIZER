
#include <numeric>
#include <algorithm>
#include <ADC.h>
#include <vector>
#include "/home/loasis/kHz-STABILIZER/kHz_macros.h"

ADC* adc = new ADC();

int i, j;
int in[2] = { 0 , 0 };
int timeout = 0;

float drive_freqs[3];
uint16_t DACmax[2] = {0, 0};
int DAC_pins[2] = {A21, A22};
int read_pins[2] = {A9, A3};

volatile int trig_count = 0;


bool switch_ = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A14, OUTPUT);

  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);

  analogWriteResolution(12);

  Serial.begin(115200);

  post_setup();

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

      case LEARN_TOT_SYS_RESPONSE:
        learn_total_system_response();
        break;

      case LEARN_LOC_SYS_RESPONSE:
        learn_local_system_response();

      case STABILIZE:
        stabilize();
        break;

      case TEST_LOOP_TIME:
        test_loop_times();
        break;
      case CORRELATE:
        correlate_cams();
        break;

      default:
        taper_down();

    }
  }
}

void loop() {

}

elapsedMicros t;
void trigger_cams_timed() {
  t = 0;
  digitalWriteFast(A14, HIGH);
  delayMicroseconds(100);
  digitalWriteFast(A14, LOW);
}

void test_loop_times() {

  digitalWriteFast(LED_BUILTIN, HIGH);
  detachInterrupt(digitalPinToInterrupt(A18));
  Serial.write((byte)CONTINUE);

  int t_start = 0;
  std::vector<int> loop_times(5000);
  std::vector<int> shot_number(5000);
  int* shot_ptr = shot_number.data();

  delay(500);
  attachInterrupt(digitalPinToInterrupt(A18), trigger_cams, RISING);

  for (int k = 0; k < 5000; ++k) {
    
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
      loop_times[k] = t;

    }

    shot_ ptr++;

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


void correlate_cams() {
  trig_count = 0;
  detachInterrupt(digitalPinToInterrupt(A18));
  delay(500);
  Serial.write((byte)CONTINUE);
  attachInterrupt(digitalPinToInterrupt(A18), trigger_cams, RISING);
  while (trig_count < 5000);
  detachInterrupt(digitalPinToInterrupt(A18));

}

void trigger_cams() {
  digitalWriteFast(A14, HIGH);
  delayMicroseconds(300);
  digitalWriteFast(A14, LOW);
  trig_count++;
}

void stabilize() {

  detachInterrupt(digitalPinToInterrupt(A18));
  digitalWriteFast(LED_BUILTIN, HIGH);

  init_actuator();
  Serial.write((byte)CONTINUE);

  delay(500);
  attachInterrupt(digitalPinToInterrupt(A18), trigger_cams, RISING);
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

void learn_local_system_response() {

  uint16_t* system_response_input_arr = new uint16_t[loc_sys_response_window];

  digitalWriteFast(LED_BUILTIN, HIGH);

  Serial.readBytes((char*)&DACmax[1], 2);
  Serial.readBytes((char*)drive_freqs, 12);

  generate_local_system_response_input(system_response_input_arr, DACmax[1], drive_freqs);

  init_actuator();

  Serial.clear();
  Serial.write((byte) CONTINUE);

  i = 0;

  while (i < loc_sys_response_window) {

    elapsedMillis m;
    while (!Serial.available()) {
      if (m > 1000) {
        delete system_response_input_arr;
        taper_down();
        return;
      }
    }

    if (Serial.read() != SYNC_FLAG) {
    }

    else {
      delayMicroseconds(20);
      //analogWriteDAC0(system_response_input_arr[i]);
      analogWriteDAC1(system_response_input_arr[i]);
      ++i;
    }
  }

  delete system_response_input_arr;
  taper_down();
}

void learn_total_system_response() {

  uint16_t* system_response_input_arr = new uint16_t[tot_sys_response_window];

  digitalWriteFast(LED_BUILTIN, HIGH);

  Serial.readBytes((char*)&DACmax[1], 2);

  generate_total_system_response_input(system_response_input_arr, DACmax[1], 20, 300);

  init_actuator();

  Serial.clear();
  Serial.write((byte) CONTINUE);

  i = 0;

  while (i < tot_sys_response_window) {

    elapsedMillis m;
    while (!Serial.available()) {
      if (m > 1000) {
        delete system_response_input_arr;
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

  delete system_response_input_arr;
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
