#pragma once

#include "kHz_macros.h"
#include <vector>
#include <gsl/gsl_poly.h>

#define max_tones 5
#define sq(x) ((x)*(x))

struct complex {
	float real = 0;
	float imag = 0;
};

struct axes_cell {
	
	void set_actuator_constants(std::array<double, 4> fit_params) {
		A = fit_params[0];
		B = fit_params[1];
		C = fit_params[2];
		D = fit_params[3];
	}

	float* noise;

	complex Chi[max_tones];
	float w[max_tones];
	int N[max_tones];
	float old_centroid[max_tones] = { 0 };

	int n = 0;
	int num_tones;
	float differential;
	int maxN;
	float A;
	float B;
	float C;
	float D;
	double solved_cubic[3];
	int num_sols;
	float noise_element = 0;
	float target[3];
	float m;
	float b;
	int DAC_cmds[3] = { 0 };
	float SET_POINT;
	
#define moving_avg_window 20

	float running_avg[moving_avg_window];
	float error = 0;

	int Wrap(int N, int L, int H) {
		H = H - L + 1; return (N - L + (N < L) * H) % H + L;
	}

	int next_DAC(float in) {
		
		noise_element = in - target[0];

		differential = 0;

		for (int j = 0; j < num_tones; ++j) {

			float Chi_real = Chi[j].real;

			Chi[j].real = noise_element * cosf(w[j] * (1 - N[j])) - old_centroid[j] * cosf(w[j]) +
				Chi[j].real * cosf(w[j]) - Chi[j].imag * sinf(w[j]);

			Chi[j].imag = noise_element * sinf(w[j] * (1 - N[j])) - old_centroid[j] * sinf(w[j]) +
				Chi_real * sinf(w[j]) + Chi[j].imag * cosf(w[j]);

			float mag = 2.0 / N[j] * sqrtf(sq(Chi[j].real) + sq(Chi[j].imag));
			float phase = atan2f(Chi[j].imag, Chi[j].real);

			differential += mag * (cosf(w[j] * (N[j]) + phase) - cosf(w[j] * (N[j] - 1) + phase));
		}

		running_avg[n % moving_avg_window] = in / static_cast<float>(moving_avg_window);

		if (n > maxN + moving_avg_window) {
			error = std::accumulate(running_avg, running_avg + moving_avg_window, 0.f) - SET_POINT;
		}

		//target[0] = 2 * SET_POINT - (in + differential);
		target[0] -= differential;
		target[0] -= error/static_cast<float>(moving_avg_window);
		  
		//DAC_cmds[0]  = round((target[0] * (1 + tf_params[3]) - tf_params[3] * target[1] - tf_params[2] * DAC_cmds[1] - tf_params[1])
		//	/ (tf_params[0] - tf_params[2]));

		DAC_cmds[0] = (target[0] - B - (C + 2 * D) * DAC_cmds[1] + D * DAC_cmds[2]) / (A - C - D);

		DAC_cmds[0] = std::max(DAC_cmds[0], 0);
		DAC_cmds[0] = std::min(DAC_cmds[0], 4095);
		
		return DAC_cmds[0];

	}

	void post_step() {

		DAC_cmds[2] = DAC_cmds[1];
		DAC_cmds[1] = DAC_cmds[0];
		target[2] = target[1];
		target[1] = target[0];

		++n;

		for (int j = 0; j < num_tones; ++j) {
			old_centroid[j] = noise[Wrap(n + 1 - N[j], 0, maxN)];
		}

		noise[Wrap(n,0,maxN)] = noise_element;

	}
};

std::vector<axes_cell> axes(2);
