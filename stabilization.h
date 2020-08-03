#pragma once

#include "FS_macros.h"
#include <vector>
#include <gsl/gsl_poly.h>

#define max_tones 5
#define sq(x) ((x)*(x))

struct complex {
	float mag = 0;
	float phase = 0;
};
struct axes_cell {

	float* noisy_signal;
	complex Chi[max_tones];
	float prev[max_tones] = { 0 };
	float w[max_tones];
	float N[max_tones];
	int prev_idx[max_tones] = { 0 };
	
	float differential;

	int maxN;
	int DAC_cmds[3] = { 0 };
	int num_tones;
	float tf_params[5];
	unsigned int n = 0;
	float centroid_prev[2];
	double solved_cubic[3];
	int num_sols;
	float target[3];
	float noise_element = 0;
	float m;
	float b;
	float SET_POINT;
	int temp_cmds[3];

	int next_DAC(float in) {
		
		noise_element = (in - SET_POINT) + (SET_POINT - target[0]);

		differential = 0;

		for (int j = 0; j < num_tones; ++j) {

			Chi[j].mag =
				2 / N[j] * sqrtf(sq(Chi[j].mag) +
					Chi[j].mag * (2 * (cosf(w[j] * N[j] + Chi[j].phase) * noise_element -
						cosf(Chi[j].phase) * prev[j])) + sq(prev[j]) + sq(noise_element) -
					2 * noise_element * prev[j] * cosf(w[j] * N[j]));

			Chi[j].phase = atan2f(
				Chi[j].mag * sinf(w[j] + Chi[j].phase) -
				prev[j] * sinf(w[j]) - noise_element * sinf(w[j] * (N[j] - 1)),

				Chi[j].mag * cosf(w[j] + Chi[j].phase) -
				prev[j] * cosf(w[j]) + noise_element * cosf(w[j] * (N[j] - 1)));

			
			differential +=

				Chi[j].mag * cosf(w[j] * (N[j] + 1) + Chi[j].phase) -
				Chi[j].mag * cosf(w[j] * (N[j] - 0) + Chi[j].phase);

		}


		if (n < 600) {

			qDebug() << Chi[0].mag;
			qDebug() << Chi[0].phase;
			qDebug() << differential;
			qDebug() << in;

			return DAC_cmds[0];
		}

		//target[0] = 2 * SET_POINT - (in + differential);
		target[0] = SET_POINT - (noise_element + differential);

		num_sols = gsl_poly_solve_cubic(tf_params[3] / tf_params[4],
			(tf_params[3] - tf_params[0]) / tf_params[4],
			(tf_params[1] + (target[0] - 2*target[1] + target[2]) - tf_params[0] * (DAC_cmds[2] - DAC_cmds[1])) / tf_params[4],
			solved_cubic, solved_cubic + 1, solved_cubic + 2);

		if (num_sols == 1) {
			DAC_cmds[0] = round(solved_cubic[0]) + DAC_cmds[1];
		}
			
		else {
			
			// FIND ROOT CLOSEST TO THE LINEAR APPROXIMATE COMMAND

			for (int i = 0; i < 3; ++i)		

			temp_cmds[i] = round(solved_cubic[i]) + DAC_cmds[1] - (target[0] - b) / m;

			int* cmd_ptr = std::min_element(temp_cmds, temp_cmds + 3);

			DAC_cmds[0] = round(solved_cubic[cmd_ptr - temp_cmds]) + DAC_cmds[1];

		}


		qDebug() << noise_element + differential;
		qDebug() << target[0];
		qDebug() << noise_element;
		qDebug() << in;

		return DAC_cmds[0];

	}

	void post_step() {


		noisy_signal[n % maxN] = noise_element;

		++n;

		DAC_cmds[2] = DAC_cmds[1];
		DAC_cmds[1] = DAC_cmds[0];
		target[2] = target[1];
		target[1] = target[0];

		for (int j = 0; j < num_tones; ++j) {

			if (n >= N[j]) {
				prev[j] = noisy_signal[prev_idx[j]];
				prev_idx[j] = (prev_idx[j] + 1) % maxN;
			}

		}

	}


};

std::vector<axes_cell> axes(2);