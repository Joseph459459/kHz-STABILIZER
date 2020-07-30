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

	float* signal;
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
	int n;
	float centroid_prev[2];
	double solved_cubic[3];
	int num_sols;
	float for_second_deriv;

	int next_DAC(const float in) {

		differential = 0;
		for_second_deriv = -2 * signal[n % maxN] + signal[(n - 1) % maxN];

		for (int j = 0; j < num_tones; ++j) {

			Chi[j].mag =
				2 / N[j] * sqrtf(sq(Chi[j].mag) +
					Chi[j].mag * (2 * (cosf(w[j] * N[j] + Chi[j].phase) * in -
						cosf(Chi[j].phase) * prev[j])) + sq(prev[j]) + sq(in) -
					2 * in * prev[j] * cosf(w[j] * N[j]));

			Chi[j].phase = atan2f(
				Chi[j].mag * sinf(w[j] + Chi[j].phase) -
				prev[j] * sinf(w[j]) - in * sinf(w[j] * (N[j] - 1)),

				Chi[j].mag * cosf(w[j] + Chi[j].phase) -
				prev[j] * cosf(w[j]) + in * cosf(w[j] * (N[j] - 1)));

			
			differential +=

				Chi[j].mag * cosf(w[j] * N[j] + Chi[j].phase) -
				Chi[j].mag * cosf(w[j] * (N[j] - 1) + Chi[j].phase);

		}


		num_sols = gsl_poly_solve_cubic(tf_params[3] / tf_params[4],
			(tf_params[3] - tf_params[0]) / tf_params[4],
			(tf_params[1] + (in + for_second_deriv) - tf_params[0] * (DAC_cmds[2] - DAC_cmds[1])) / tf_params[4],
			solved_cubic, solved_cubic + 1, solved_cubic + 2);

		if (num_sols == 1)
			DAC_cmds[0] = round(solved_cubic[0]) + DAC_cmds[1];

		else
			DAC_cmds[0] = round(solved_cubic[2]) + DAC_cmds[1];


		return DAC_cmds[0];

	}

	void post_step(const float in) {

		differential = 0;

		signal[n % maxN] = in;

		++n;

		DAC_cmds[2] = DAC_cmds[1];
		DAC_cmds[1] = DAC_cmds[0];

		for (int j = 0; j < num_tones; ++j) {

			if (n >= N[j]) {
				prev[j] = signal[prev_idx[j]];
				prev_idx[j] = (prev_idx[j] + 1) % maxN;
			}

		}



	}


};

std::vector<axes_cell> axes(2);