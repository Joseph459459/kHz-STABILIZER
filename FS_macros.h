#pragma once
#define num_tones 3
#define window 5000
#define window_2 3000

#define sampling_freq static_cast<float>(1000.0)
#define PI static_cast<float>(3.14159265358979323846)

#define STREAM 0
#define STABILIZE 1
#define SYNC_FLAG 254
#define FIND_RANGE 2
#define SPECTRUM 3
#define LEARN_TF 4
#define STOP_FLAG 5
#define CONTINUE 6

inline void tf_input(uint16_t* tf_input_arr, uint16_t yDACmax) {

	for (int i = 0; i < 1000; ++i) {

		tf_input_arr[i] = yDACmax / 2 / 1000.0 * i;

		tf_input_arr[i + 1000] = yDACmax / 2 * sinf(2 * PI * i / 100) + yDACmax / 2;

		tf_input_arr[i + 2000] = (i % 100) / 50 ? yDACmax : 0;

	}
}