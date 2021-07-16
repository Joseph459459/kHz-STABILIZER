#pragma once
#include <stdio.h>
#include <math.h>
#include <vector>

#define fft_window 10000
#define tot_sys_response_window 20000

#define n_taps 301
#define section_width 1000
#define loc_sys_response_window 3 * section_width + 1

#define sampling_freq 1000.0
#define PI 3.14159265358979323846

#define STREAM 0
#define STABILIZE 1
#define FIND_RANGE 2
#define SPECTRUM 3
#define LEARN_LOC_SYS_RESPONSE 4
#define LEARN_TOT_SYS_RESPONSE 10
#define CORRELATE 8
#define TEST_LOOP_TIME 9

#define STOP_FLAG 5
#define CONTINUE 6
#define SYNC_FLAG 254

#define bit_depth 4096


typedef unsigned short uint16_t;

template <typename T>
inline void generate_total_system_response_input(T* sys_response_input_arr, double DAC_max, double lf, double hf) {

    std::vector<double> omega(tot_sys_response_window);

    for (int i = 0; i < tot_sys_response_window; ++i)
        omega[i] = 2 * PI * (lf + (hf - lf) * i / (double) tot_sys_response_window);

    for (int i = 0; i < tot_sys_response_window; ++i)
        sys_response_input_arr[i] = round(DAC_max / 3 * sin(omega[i] * i / 1000) + DAC_max / 2);

}

template <typename T>
inline void generate_local_system_response_input(T* sys_response_input_arr, double DAC_max, float* freqs) {

    int idx = 0;
    int i;

    sys_response_input_arr[0] = round(DAC_max / 2);

    ++idx;

    for (i = 0; i < section_width; ++i) {

        sys_response_input_arr[i + idx] = round(DAC_max / 2 + (DAC_max / 4 + DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[0] / 1000 * i));
    }
    idx += section_width;

    for (i = 0; i < section_width; ++i) {
        sys_response_input_arr[i + idx] = round(DAC_max / 2 + (DAC_max / 2 - DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[1] / 1000 * i));
    }
    idx += section_width;

    for (i = 0; i < section_width; ++i) {
        sys_response_input_arr[i + idx] = round(DAC_max / 2 + (DAC_max / 4 + DAC_max / 4 * i / 1000) * sinf(2 * PI * freqs[2] / 1000 * i));
    }
    idx += section_width;


}

