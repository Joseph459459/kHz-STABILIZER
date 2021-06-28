#pragma once
#include <stdio.h>
#include <math.h>

#define _window 5000

#define sampling_freq 1000.0
#define PI 3.14159265358979323846

#define STREAM 0
#define STABILIZE 1
#define FIND_RANGE 2
#define SPECTRUM 3
#define LEARN_SYS_RESPONSE 4
#define CORRELATE 8
#define TEST_LOOP_TIME 9

#define STOP_FLAG 5
#define CONTINUE 6
#define SYNC_FLAG 254

#define bit_depth 4096
#define n_taps 301
#define section_width 1000
#define sys_response_window 3 * section_width + 1


typedef unsigned short uint16_t;

template <typename T>
inline void generate_system_response_input(T* sys_response_input_arr, uint16_t DAC_max, float* freqs) {

	int idx = 0;
	int i;

    sys_response_input_arr[0] = DAC_max / 2;

	++idx;

	for (i = 0; i < section_width; ++i) {

        sys_response_input_arr[i + idx] = round((double)DAC_max / 2 + ((double) DAC_max / 4 + (double) DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[0] / 1000 * i));
	}
	idx += section_width;
	
	for (i = 0; i < section_width; ++i) {
        sys_response_input_arr[i + idx] = round((double)DAC_max / 2 + ((double) DAC_max / 2 - (double) DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[1] / 1000 * i));
	}
	idx += section_width;

	for (i = 0; i < section_width; ++i) {
        sys_response_input_arr[i + idx] = round((double)DAC_max / 2 + ((double) DAC_max / 4 + (double) DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[2] / 1000 * i));
	}
	idx += section_width;

	
}

