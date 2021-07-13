#pragma once

#include "kHz_macros.h"
#include <vector>
#include <array>
#include <numeric>
#include <algorithm>


#define max_tones 5
#define sq(x) ((x)*(x))

struct complex {
	float real = 0;
	float imag = 0;
};

#define moving_avg_window 20

struct SDTFT_algo {
	
    SDTFT_algo(int i){};

    SDTFT_algo(std::array<double, 4> fit_params,
                    std::vector<float> tones,
                    std::vector<int> N, int DAC_max,
                    float SET_POINT) {

        A = fit_params[0];
		B = fit_params[1];
        C = fit_params[2];
        D = fit_params[3];

        num_tones = tones.size();

        for (int i = 0; i < tones.size(); ++i){
            omega[i] = 2 * PI * tones[i] / 1000;
            this->N[i] = N[i];
        }

        for (int i = 0; i < 3; ++i){
            DAC_cmds[i] = round((float)DAC_max / 2);
            target[i] = SET_POINT;
        }

        maxN = *std::max_element(N.begin(),N.end());
        noise = new float[maxN + 1]();

        this->SET_POINT = SET_POINT;
        this->DAC_max = DAC_max;

        for (int i = 0; i < moving_avg_window; ++i){
            running_avg[i] = SET_POINT;
        }

	}

	float* noise;

    // the complex fourier component
	complex Chi[max_tones];

    // the normalized frequencies
    float omega[max_tones];

    // the lengths of the individual filters
	int N[max_tones];

    // centroid at the END of the filter (to be removed next sample)
	float old_centroid[max_tones] = { 0 };

	int n = 0;
	int num_tones;
	float differential;
	int maxN;

    // parameters for system response calculation
	float A;
	float B;
	float C;
	float D;

	float noise_element = 0;
	float target[3];
	int DAC_cmds[3] = { 0 };
    int DAC_max = 0;

    // the target centroid - should be 1/2 of the max DAC range
	float SET_POINT;
	
	float running_avg[moving_avg_window];
	float error = 0;

    int wrap(int N, int L, int H) {
		H = H - L + 1; return (N - L + (N < L) * H) % H + L;
	}

    int next_DAC_cmd(float in) {
		
		noise_element = in - target[0];

		differential = 0;

        // Computes the Sliding Discrete-time Fourier Transform for each target frequency

		for (int j = 0; j < num_tones; ++j) {

            float Chi_real_curr = Chi[j].real;

            Chi[j].real = noise_element * cosf(omega[j] * (1 - N[j])) - old_centroid[j] * cosf(omega[j]) +
                Chi[j].real * cosf(omega[j]) - Chi[j].imag * sinf(omega[j]);

            Chi[j].imag = noise_element * sinf(omega[j] * (1 - N[j])) - old_centroid[j] * sinf(omega[j]) +
                Chi_real_curr * sinf(omega[j]) + Chi[j].imag * cosf(omega[j]);

			float mag = 2.0 / N[j] * sqrtf(sq(Chi[j].real) + sq(Chi[j].imag));
			float phase = atan2f(Chi[j].imag, Chi[j].real);

            differential += mag * (cosf(omega[j] * (N[j]) + phase) - cosf(omega[j] * (N[j] - 1) + phase));
		}

        // Computes moving average to perform proportional feedback

		running_avg[n % moving_avg_window] = in / static_cast<float>(moving_avg_window);


        error = std::accumulate(running_avg, running_avg + moving_avg_window, 0.f) - SET_POINT;


		target[0] -= differential;
        target[0] -= error/static_cast<float>(moving_avg_window);
		  
        DAC_cmds[0] = (target[0] - B + (C + 2 * D) * DAC_cmds[1] - D * DAC_cmds[2]) / (A + C + D);

        if (std::max(DAC_cmds[0],0) == 0){
            target[0] = B - (C + 2*D)*DAC_cmds[1] + D*DAC_cmds[2];
            DAC_cmds[0] = 0;
            return 0;
        }
        if (std::min(DAC_cmds[0], DAC_max) == DAC_max){
            target[0] = (A+C+D)*DAC_max + B - (C + 2*D)*DAC_cmds[1] + D*DAC_cmds[2];
            DAC_cmds[0] = DAC_max;
            return DAC_max;
        }

		return DAC_cmds[0];

	}

    void prepare_next_cmd() {

		DAC_cmds[2] = DAC_cmds[1];
		DAC_cmds[1] = DAC_cmds[0];
		target[2] = target[1];
		target[1] = target[0];

		++n;

		for (int j = 0; j < num_tones; ++j) {
            old_centroid[j] = noise[wrap(n + 1 - N[j], 0, maxN)];
		}

        noise[wrap(n,0,maxN)] = noise_element;

	}
};
