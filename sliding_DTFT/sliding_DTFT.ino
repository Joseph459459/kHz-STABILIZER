
#include <numeric>
#include <algorithm>
#include <ADC.h>
#include <vector>

#include "FS_macros.h"



ADC* adc = new ADC();


int i, j;
int in[2] = { 0 , 0 };
int timeout = 0;
float drive_freqs[3];


int xDACmax = 0;
uint16_t yDACmax = 0;



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

	digitalWriteFast(LED_BUILTIN, LOW);
	Serial.clear();
	while (!Serial.available());

	switch (Serial.read()) {

	case FIND_RANGE:
		find_range();

	case LEARN_TF:
		learn_tf();

	case STABILIZE:
		stabilize();

	default:
		taper_down();

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


			analogWriteDAC0(in[0]);

			analogWriteDAC1(in[1]);


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

