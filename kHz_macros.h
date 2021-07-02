#pragma once
#include <stdio.h>
#include <math.h>
#include <vector>

#define fft_window 10000
#define sys_response_window 10000

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


typedef unsigned short uint16_t;

template <typename T>
inline void generate_system_response_input(T* sys_response_input_arr, uint16_t DAC_max, double lf, double hf) {

    std::vector<double> omega(sys_response_window);

    for (int i = 0; i < sys_response_window; ++i)
        omega[i] = 2 * PI * (lf + (hf - lf) * i / (double) sys_response_window);

    for (int i = 0; i < sys_response_window; ++i)
        sys_response_input_arr[i] = round(DAC_max / 3.0 * sin(omega[i] * i / 1000.0) + DAC_max / 2);

}

