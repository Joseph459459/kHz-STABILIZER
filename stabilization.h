#pragma once

#include "FS_macros.h"
#include <vector>
#include <gsl/gsl_poly.h>

#define max_tones 5
#define sq(x) ((x)*(x))

struct complex {
	float real = 0;
	float imag = 0;
};

struct axes_cell {

	float* noise;

	complex Chi[max_tones];
	float w[max_tones];
	int N[max_tones];
	float old_centroid[max_tones] = { 0 };

	int n = 0;
	int num_tones;
	float differential;
	int maxN;
	float tf_params[5];
	double solved_cubic[3];
	int num_sols;
	float noise_element = 0;
	float target[3];
	float m;
	float b;
	int DAC_cmds[3] = { 0 };
	float SET_POINT;
	int temp_cmds[3];


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

		target[0] = 2 * SET_POINT - (in + differential);




		//num_sols = gsl_poly_solve_cubic(tf_params[3] / tf_params[4],
		//	(tf_params[3] - tf_params[0]) / tf_params[4],
		//	(tf_params[1] + (target[0] - 2 * target[1] + target[2]) - tf_params[0] * (DAC_cmds[2] - DAC_cmds[1])) / tf_params[4],
		//	solved_cubic, solved_cubic + 1, solved_cubic + 2);
		//if (num_sols == 1) {
		//	DAC_cmds[0] = round(solved_cubic[0]) + DAC_cmds[1];
		//}
		//else {
		//	// FIND ROOT CLOSEST TO THE LINEAR APPROXIMATE COMMAND
		//	for (int i = 0; i < 3; ++i) {
		//		temp_cmds[i] = round(solved_cubic[i]) + DAC_cmds[1] - (target[0] - b) / m;
		//		int* cmd_ptr = std::min_element(temp_cmds, temp_cmds + 3);
		//		DAC_cmds[0] = round(solved_cubic[cmd_ptr - temp_cmds]) + DAC_cmds[1];
		//	}
		//}


		qDebug() << DAC_cmds[0];
		qDebug() << target[0];
		qDebug() << noise_element;
		qDebug() << in;

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

		noise[Wrap(n, 0, maxN)] = noise_element;

	}
};

std::vector<axes_cell> axes(2);