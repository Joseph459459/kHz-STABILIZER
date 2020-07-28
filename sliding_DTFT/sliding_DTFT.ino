
#include <numeric>
#include <algorithm>
#include <ADC.h>
#include <vector>

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
struct axes_cell {

	float* signal;
	complex Chi[num_tones];
	float prev[num_tones] = { 0 };
	float w[num_tones];
	float N[num_tones];
	int prev_idx[num_tones] = { 0 };
	int maxN;
	int DAC_cmds[3] = { 0 };

	void assign(float tones[num_tones], float n[num_tones]) {

		for (int j = 0; j < num_tones; ++j) {

			w[j] = 2 * PI * tones[j] / sampling_freq;
			N[j] = n[j];
			Chi[j].mag = 0;
			Chi[j].phase = 0;

		}
	}
};

std::vector<axes_cell> axes(2);

int i, j;
float differential[2] = { 0 , 0 };
float in[2] = { 0 , 0 };
float tf_params[2] = { 0 , 0 };
int timeout = 0;
unsigned int n = 0;
float drive_freqs[3];

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
	ax.assign(xtones, xN);

	for (int k = 0; k < 2000; ++k) {
		sine[k] = sinf(2 * PI * 60 / sampling_freq * k);
	}
	ax.maxN = *std::maaxelement(xN, xN + num_tones);
	ax.signal = new float[ax.maxN]();


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

		Serial.readBytes((char*) &tf_params, 8);


		axes[0].assign(xtones, xN);
		axes[0].maxN = *std::max_element(xN, xN + num_tones);
		axes[0].signal = new float[axes[0].maxN]();
		axes[1].assign(ytones, yN);
		axes[1].maxN = *std::max_element(yN, yN + num_tones);
		axes[1].signal = new float[axes[1].maxN]();

		stabilize();


	default:
		taper_down();

#endif
	}

}

void loop() {

}

int ax_idx = 0;

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

			for (axes_cell ax : axes) {

				for (j = 0; j < num_tones; ++j) {

					ax.Chi[j].mag =
						2 / ax.N[j] * sqrtf(sq(ax.Chi[j].mag) +
							ax.Chi[j].mag * (2 * (cosf(ax.w[j] * ax.N[j] + ax.Chi[j].phase) * in[ax_idx] -
								cosf(ax.Chi[j].phase) * ax.prev[j])) + sq(ax.prev[j]) + sq(in[ax_idx]) -
							2 * in[ax_idx] * ax.prev[j] * cosf(ax.w[j] * ax.N[j]));

					ax.Chi[j].phase = atan2f(
						ax.Chi[j].mag * sinf(ax.w[j] + ax.Chi[j].phase) -
						ax.prev[j] * sinf(ax.w[j]) - in[ax_idx] * sinf(ax.w[j] * (ax.N[j] - 1)),

						ax.Chi[j].mag * cosf(ax.w[j] + ax.Chi[j].phase) -
						ax.prev[j] * cosf(ax.w[j]) + in[ax_idx] * cosf(ax.w[j] * (ax.N[j] - 1)));

					differential[ax_idx] = differential[ax_idx] +

						ax.Chi[j].mag * cosf(ax.w[j] * ax.N[j] + ax.Chi[j].phase) -
						ax.Chi[j].mag * cosf(ax.w[j] * (ax.N[j] - 1) + ax.Chi[j].phase);

				}

				ax.DAC_cmds[0] = round(1 / tf_params[ax_idx] * ((in[ax_idx] + differential[ax_idx]) - 2 * ax.signal[n % ax.maxN - 1] + 2 * ax.signal[n % ax.maxN - 2])
					+ 2 * ax.DAC_cmds[1] - ax.DAC_cmds[2]);
			
				ax_idx++;
			}


			analogWriteDAC0(axes[0].DAC_cmds[0]);

			analogWriteDAC1(axes[1].DAC_cmds[0]);


			ax_idx = 0;
			for (axes_cell ax : axes) {
				
				differential[ax_idx] = 0;

				ax.signal[n % ax.maxN] = in[ax_idx];
				++n;

				for (j = 0; j < num_tones; ++j) {

					if (n >= ax.N[j]) {
						ax.prev[j] = ax.signal[ax.prev_idx[j]];
						ax.prev_idx[j] = (ax.prev_idx[j] + 1) % ax.maxN;
					}

				}

				ax_idx++;
			}

			ax_idx = 0;
		}
	}
}

void learn_tf() {
	

	Serial.readBytes((char*)drive_freqs, 24);

	uint16_t* tf_input_arr = new uint16_t[tf_window]();

	tf_input_(tf_input_arr,yDACmax,drive_freqs);

	init_actuator();

	Serial.write((char*) &drive_freqs,24);
	
	i = 0;

	while (i < tf_window) {

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

	yDACmax = 0;

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

	delete axes[0].signal;
	delete axes[1].signal;

	int tapering_val_y = adc->adc0->analogRead(A9);
	int tapering_val_x = adc->adc1->analogRead(A3);

	while (tapering_val_y > 0) {
		analogWriteDAC1(--tapering_val_y);
		delayMicroseconds(100);
	}
	postsetup();
}

void init_actuator() {

	for (int i = 0; i < 20; ++i) {
		analogWriteDAC0(4095.0 / 20 * i);
		analogWriteDAC1(4095.0 / 20 * i);
		delay(1);
	}

	for (int i = 0; i < 20; ++i) {
		analogWriteDAC0(4095 - 4095.0 / 20 * i);
		analogWriteDAC1(4095 - 4095.0 / 20 * i);
		delay(1);
	}

	for (int i = 0; i < 10; ++i) {
		analogWriteDAC0(2048.0 / 20 * i);
		analogWriteDAC1(2048.0 / 20 * i);
		delay(1);
	}

}

