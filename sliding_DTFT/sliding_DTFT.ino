
#include <numeric>
#include <algorithm>
#include <ADC.h>
#include "FS_macros.h"

//#define _DEBUG_ON
#ifndef _DEBUG_ON
#else 
#define _DEBUG_
#endif

ADC* adc = new ADC();

struct complex {
	float mag;
	float phase;
};
struct dir_cell {

	complex Chi[num_tones];
	float prev[num_tones] = { 0 };
	float w[num_tones];
	float N[num_tones];
	int prev_idx[num_tones] = { 0 };
	int maxN;


	void assign(float tones[num_tones], float n[num_tones]) {

		for (int j = 0; j < num_tones; ++j) {

			w[j] = 2 * PI * tones[j] / sampling_freq;
			N[j] = n[j];
			Chi[j].mag = 0;
			Chi[j].phase = 0;

		}
	}
};

dir_cell x_;
dir_cell y_;

int i, j;
float differential[2] = { 0 , 0 };
float in[2] = { 0 , 0 };
int timeout = 0;
unsigned int n = 0;
float* xsignal;
float* ysignal;
int xDACmax = 0;
uint16_t yDACmax = 0;

#ifdef _DEBUG_
float sine[2000];
#endif 


void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);

	adc->setAveraging(16);
	adc->setResolution(12);

	analogWriteResolution(12);

	Serial.begin(115200);

	postsetup();
}

void postsetup() {
#ifdef _DEBUG_
	while (!Serial.available());

	float xtones[3] = { 20, 40, 60 };
	float xN[3] = { 60, 60, 60 };
	x_.assign(xtones, xN);

	for (int k = 0; k < 2000; ++k) {
		sine[k] = sinf(2 * PI * 60 / sampling_freq * k);
	}
	x_.maxN = *std::max_element(xN, xN + num_tones);
	xsignal = new float[x_.maxN]();


#else 
	digitalWriteFast(LED_BUILTIN, LOW);
	Serial.clear();
	while (!Serial.available());

	switch (Serial.read()) {

	case FIND_RANGE:
		find_range();

	case LEARN_TF:
		learn_tf();

	case STABILIZE:

		float xtones[num_tones];
		float ytones[num_tones];
		float xN[num_tones];
		float yN[num_tones];

		for (int i = 0; i < num_tones; ++i) {
			Serial.readBytes((char*)&xtones[i], 4);
			Serial.readBytes((char*)&xN[i], 4);
			Serial.readBytes((char*)&ytones[i], 4);
			Serial.readBytes((char*)&yN[i], 4);
		}
		x_.assign(xtones, xN);
		x_.maxN = *std::max_element(xN, xN + num_tones);
		xsignal = new float[x_.maxN]();
		y_.assign(ytones, yN);
		y_.maxN = *std::max_element(yN, yN + num_tones);
		ysignal = new float[y_.maxN]();
		stabilize();


	default:
		taper_down();

#endif
	}

}

void loop() {

}

void stabilize() {


	while (true) {
		while (!Serial.available()) {
			if (++timeout > 20000)
				taper_down();
		};
		timeout = 0;
		if (Serial.read() != SYNC_FLAG) {
			Serial.clear();
		}
		else {

			Serial.readBytes((char*)in, 8);

			for (j = 0; j < num_tones; ++j) {

				x_.Chi[j].mag =
					2 / x_.N[j] * sqrtf(sq(x_.Chi[j].mag) +
						x_.Chi[j].mag * (2 * (cosf(x_.w[j] * x_.N[j] + x_.Chi[j].phase) * in[0] -
							cosf(x_.Chi[j].phase) * x_.prev[j])) + sq(x_.prev[j]) + sq(in[0]) -
						2 * in[0] * x_.prev[j] * cosf(x_.w[j] * x_.N[j]));

				x_.Chi[j].phase = atan2f(
					x_.Chi[j].mag * sinf(x_.w[j] + x_.Chi[j].phase) -
					x_.prev[j] * sinf(x_.w[j]) - in[0] * sinf(x_.w[j] * (x_.N[j] - 1)),

					x_.Chi[j].mag * cosf(x_.w[j] + x_.Chi[j].phase) -
					x_.prev[j] * cosf(x_.w[j]) + in[0] * cosf(x_.w[j] * (x_.N[j] - 1)));

				differential[0] = differential[0] +

					x_.Chi[j].mag * cosf(x_.w[j] * x_.N[j] + x_.Chi[j].phase) -
					x_.Chi[j].mag * cosf(x_.w[j] * (x_.N[j] - 1) + x_.Chi[j].phase);

			}

			analogWriteDAC0(0);
			analogWriteDAC1(0);

			differential[0] = 0;

			xsignal[n % x_.maxN] = in[0];
			++n;

			for (int i = 0; i < num_tones; ++i) {

				if (n >= x_.N[j]) {
					x_.prev[j] = xsignal[x_.prev_idx[j]];
					x_.prev_idx[j] = (x_.prev_idx[j] + 1) % x_.maxN;
				}

			}

		}
	}
}

void learn_tf() {
	
	uint16_t* tf_input_arr = new uint16_t[window_2]();

	tf_input(tf_input_arr,yDACmax);


	Serial.write((byte)CONTINUE);
	
	
	i = 0;

	while (i < window_2) {

		// 0.144 micros per loop iteration
		while (!Serial.available()) {
			if (++timeout > 6e6)
				taper_down();
		}
		timeout = 0;

		if (Serial.read() != SYNC_FLAG) {
			Serial.clear();
		}
		else {
			delayMicroseconds(25);
			analogWriteDAC0(tf_input_arr[i]);
			analogWriteDAC1(tf_input_arr[i]);

			++i;
		}
	}

	delete tf_input_arr;

}

void find_range() {

	digitalWriteFast(LED_BUILTIN, HIGH);

	int currx = 0;
	int curry = 0;

	while (true) {

		// 0.144 micros per loop iteration
		while (!Serial.available()) {
			if (++timeout > 6e6)
				taper_down();
		}
		timeout = 0;

		if (Serial.read() != SYNC_FLAG) {
			Serial.clear();
			Serial.write((byte)CONTINUE);
		}

		else {
			if (Serial.read() == STOP_FLAG) {
				yDACmax = curry - 5;
				Serial.write((byte)STOP_FLAG);
				Serial.write((const byte*)&yDACmax, 2);
				break;
			}
			else {
				analogWriteDAC1(curry);
				curry += 5;

				if (curry >= 4095) {
					yDACmax = 4095;
					Serial.write((byte)STOP_FLAG);
					Serial.write((const byte*)&yDACmax, 2);
					break;
				}

				Serial.write((byte)CONTINUE);
			}
		}
	}

	taper_down();
}

void taper_down() {

	delete xsignal;
	delete ysignal;

	int tapering_val_y = adc->adc0->analogRead(A9);
	int tapering_val_x = adc->adc1->analogRead(A3);

	while (tapering_val_y > 0) {
		analogWriteDAC1(--tapering_val_y);
		delayMicroseconds(100);
	}
	postsetup();
}



