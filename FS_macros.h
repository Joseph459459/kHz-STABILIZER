#pragma once
#define num_tones 3
#define window 5000
#define window_2 1000

#define sampling_freq static_cast<float>(1000.0)
#define PI static_cast<float>(3.14159265358979323846)

#define STREAM 0
#define STABILIZE 1
#define SYNC_FLAG 254
#define FIND_RANGE 2
#define SPECTRUM 3
#define LEARN_TF 4
#define STOP_FLAG 5
#define CONTINUE 6

template <typename T>
inline void tf_input(T* tf_input_arr, uint16_t yDACmax) {

	tf_input_turnaroundspeed(tf_input_arr, yDACmax);

}

template <typename T>
inline void tf_input_turnaroundspeed(T* tf_input_arr, uint16_t yDACmax) {
	int i, j;
	int idx = 0;
	double slope[10];
	for (i = 1; i < 11; ++i) {
		slope[i-1] = 2048.0 / 60 * i;
	}

	for (i = 0; i < round(2048 / slope[0]); ++i) {

		tf_input_arr[idx + i] = round(slope[0] * i);
	}
	idx += round(2048 / slope[0]);

	for (i = 0; i < round(2048 / slope[0]); ++i) {

		tf_input_arr[idx + i] = round(2048.0 - slope[0] * i);
	}
	idx += round(2048 / slope[0]);

	for (j = 0; j < 10; ++j) {
		
		for (i = 0; i < round(2048 / slope[0]); ++i) {

			tf_input_arr[idx + i] = round(slope[0] * i);
		}

		idx += round(2048 / slope[0]);

		for (i = 0; i < round(2048 / slope[j]);++i) {

			tf_input_arr[idx + i] = round(2048.0 - slope[j] * i);
		}

		idx += round(2048 / slope[j]);
	}


}

template <typename T>
inline void tf_input_refandsinewaves(T* tf_input_arr, uint16_t yDACmax) {

	int idx = 0;

	for (int j = 0; j < 5; ++j) {
		for (int i = 0; i < 22; ++i) {
			tf_input_arr[idx + i] = round(4095.0 / 22  * i);
			tf_input_arr[idx + i + 22] = round(4095 - 4095.0 / 22 * i);
		}
		idx += 44;
	}

	for (int i = 0; i < 4000; ++i) {
		
		tf_input_arr[idx + i] = round(2000.0 / 3 * (sin(2 * PI / 50 * i) + sin(2 * PI / 16.6 * i + 1) + sin(2 * PI / 20 * i - 2)) + 2048);
	}

	for (int i = 0; i < 100; ++i) {
		tf_input_arr[idx + i] = round(tf_input_arr[idx + i] * i / 100);

	}

}

template <typename T>
inline void tf_input_littlejags(T* tf_input_arr, uint16_t yDACmax) {

	int i = 0;
	int idx = 0;

	int turns[5] = { 2000,2500,3000,3500,4095 };
	int amps[6] = { 250,500,750,1000,1500,2000 };
	double slope = 2048 / 30;

	for (int dest : turns) {

		for (int amp : amps) {

			for (i = 0; i < round(dest / slope); ++i) {
				tf_input_arr[idx + i] = round(slope * i);
			}

			idx += i;

			for (i = 0; i < round(amp / slope); ++i) {
				tf_input_arr[idx + i] = round(dest - slope * i);
			}
			
			idx += i;

			for (i = 0; i < round((4095 - (dest - amp))/slope); ++i) {
				tf_input_arr[idx + i] = round(dest - amp + slope * i);
			}
			
			idx += i;

			for (i = 0; i < round(4095 / slope); ++i) {
				tf_input_arr[idx + i] = round(4095 - slope * i);

			}

			idx += i;

		}
	}
}

template <typename T>
inline void tf_input_littlejagsgoingup(T* tf_input_arr, uint16_t yDACmax) {

	int len = 100;
	int ampl = 400;
	int drop = ampl / 2;
	for (int j = 0; j < 15; ++j){
		for (int i = 0; i < len/2; ++i) {

			tf_input_arr[i + len * j] = drop*j + ampl / ((double)len/2) * i;
			tf_input_arr[i + len * j + len/2] = (drop * j + ampl) - (drop) / ((double) len/2) * i;

		}
	}

	for (int i = 0; i < 150; ++i) {

		tf_input_arr[1500 + i] = 3000 - 3000 / 150 * i;
	}


}

template <typename T>
inline void tf_input_fullamplitude_timestretching_once(T* tf_input_arr, uint16_t yDACmax) {

#define numpeaks 20


	int i, j;
	int len[20];

	for (int i = 0; i < numpeaks; ++i) {
		len[i] = 10 * i;
	}


	double amps[numpeaks] = { 0 };

	for (i = 1; i < numpeaks + 1; ++i) {
		amps[i - 1] = 4095.0;
	}
	int idx = 0;
	for (j = 0; j < numpeaks; ++j) {

		for (i = 0; i < len[j] / 2; ++i) {
			tf_input_arr[i + idx] = round(amps[j] / (len[j] / 2.0) * i);
			tf_input_arr[i + idx + len[j] / 2] = round(amps[j] - amps[j] / (len[j] / 2.0) * i);
		}
		idx += len[j];
	}


}

template <typename T>
inline void tf_input_lowerreferencecurves(T* tf_input_arr, uint16_t yDACmax) {

	int i, j;
	const int len = 80;

	#define numpeaks 20

	double amps[numpeaks] = { 0 };
	
	amps[0] = 4095.0;

	for (i = 2; i < numpeaks+1; ++i) {
		amps[i-1] = 4095.0 - 4095.0 / numpeaks * (i-2);
	}

	for (j = 0; j < numpeaks; ++j) {	
		for (i = 0; i < len/2; ++i) {
			tf_input_arr[i + j * len] = round(amps[j] / (len / 2.0) * i);
			tf_input_arr[i + j * len + len / 2] = round(amps[j] - amps[j] / (len / 2.0) * i);
		}
	}
}

template <typename T>
inline void tf_input_refcurves_halfwaycurves(T* tf_input_arr, uint16_t yDACmax) {

#define numpeaks 6

	double amps[15] = { 0 };
	for (int i = 1; i < numpeaks+1; ++i) {
		amps[i - 1] = i * 4095 / (double) numpeaks;
	}

	int len = 40;

	for (int j = 0; j < numpeaks; ++j) {
		
		for (int i = 0; i < len/2; ++i) {

			tf_input_arr[i + len * j] = round(amps[j] / (len/2) * i);
			tf_input_arr[i + len * j + len / 2] = round(amps[j] - amps[j] / (len / 2) * i);

		}
	}

	int idx = len * numpeaks;

	for (int i = 0; i < 100; ++i) {

		tf_input_arr[idx + i] = round(4095 / 100.0 * i);
	}

	idx += 100;

	for (int j = 0; j < numpeaks; ++j) {

		for (int i = 0; i < len / 2; ++i) {

			tf_input_arr[idx + i + len * j] = round(4095.0 - amps[j] / (len / 2) * i);
			tf_input_arr[idx + i + len * j + len / 2] = round(4095.0 - (amps[j] - amps[j] / (len / 2) * i));

		}
	}

	idx += len * numpeaks;

	tf_input_arr[idx] = 4095;
	idx++;



	for (int i = 0; i < 100; ++i) {
		tf_input_arr[idx + i] = round(4095.0 - 4095 / 100 * i);
	}

	idx += 100;

	for (int i = 0; i < 100; ++i) {
		tf_input_arr[idx + i] = round(2048.0 / 100 * i);
	}

	idx += 100;


	for (int j = 0; j < numpeaks; ++j) {

		for (int i = 0; i < len / 2; ++i) {
			tf_input_arr[idx + i + len * j] = round(2047.5 - ((amps[j] / 2) / (len / 2) * i));
			tf_input_arr[idx + i + len * j + len / 2] = round((2047.5 - amps[j]/2) + (amps[j] / 2) / (len / 2) * i);
		}

	}

	idx += len * numpeaks;

	for (int j = 0; j < numpeaks; ++j) {

		for (int i = 0; i < len / 2; ++i) {
			tf_input_arr[idx + i + len * j] = round((amps[j] / 2) / (len / 2) * i + 2047.5);
			tf_input_arr[idx + i + len * j + len / 2] = round((2047.5 + amps[j] / 2) - (amps[j] / 2) / (len / 2) * i);
		}

	}




}

template <typename T>
inline void tf_input_halfwaycurves(T* tf_input_arr, uint16_t yDACmax) {

	int length = 80;
	int idx = 0;
	for (int i = 0; i < 100; ++i) {

		tf_input_arr[i] = round(2047.5 / 100 * i);
	}

	idx += 100;
	int reps = 5;

	for (int j = 0;j < reps;++j) {
		for (int i = 0; i < length / 2; ++i) {

			tf_input_arr[idx + i + j*length] = round(2047.5 + 2047.5 / (length/2) * i);
			tf_input_arr[idx + i + j*length + length / 2] = round(4095.0 - 2047.5 / (length/2) * i);
		}
	}

	idx += reps * length;
	for (int j = 0; j < reps; ++j) {
		for (int i = 0; i < length / 2; ++i) {

			tf_input_arr[idx + i + j * length] = round(2047.5 - 2047.5 / (length/2) * i);
			tf_input_arr[idx + i + j * length + length / 2] = round(2047.5 / (length/2) * i);
		}
	}
}

template <typename T>
inline void tf_input_allrefcurves_timestretching(T* tf_input_arr, uint16_t yDACmax) {

	double base_slope = 4096 / 10;

	double slopes[7] = { base_slope, base_slope / 2, base_slope / 3, base_slope / 4, 
		base_slope / 5, base_slope / 6, base_slope / 7 };

	int peaks[8] = { 4096 / 8, 2 * 4096 / 8, 3 * 4096 / 8, 4 * 4096 / 8, 5 * 4096 / 8, 6 * 4096 / 8, 7 * 4096 / 8, 4095 };
	int idx = 0;

	for (int slope : slopes) {

		for (int peak : peaks) {

			for (int i = 0; i < (int)(peak / slope); ++i) {

				tf_input_arr[idx + i] = floor(slope * i);
				tf_input_arr[idx + (int)(peak / slope) + i] = floor(peak - slope * i);

			}

			idx += 2*(int)(peak / slope);
		}
	}

	for (int i = 0; i < 20; ++i) {
		tf_input_arr[idx + i] = 4095 / 20 * i;
	}
	idx += 20;

	for (int slope : slopes) {

		for (int peak : peaks) {

			for (int i = 0; i < (int)(peak / slope); ++i) {

				tf_input_arr[idx + i] = floor(4095 - slope * i);
				tf_input_arr[idx + (int)(peak / slope) + i] = floor(4095 - peak + slope * i);

			}

			idx += 2 * (int)(peak / slope);
		}
	}
}

template <typename T>
inline void tf_input_singlejaggedtimestretch_lowerreferences(T* tf_input_arr, uint16_t yDACmax) {

	int freqs[10] = {18,28,38,48,58,68,78,88,98,108};
	int idx = 0;
	int i = 0;

	//for (int freq : freqs) {

	//	for (i = 0; i < 80; ++i) {
	//		tf_input_arr[idx + i] = floor(2048 / 80.0 * i);
	//	}

	//	idx += 80;

	//	for (i = 0; i < freq / 2; ++i) {
	//		tf_input_arr[idx + i] = floor(2048 - 2048 / (freq / 2.0) * i);
	//	}

	//	idx += freq/2;
	//}


	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


	for (int freq : freqs) {
		
		for (i = 0; i < freq / 2; ++i) {
			tf_input_arr[idx + i] = floor(2000 / (freq / 2.0) * i);
		}
		idx += freq / 2;

		for (i = 0; i < freq / 4; ++i) {
			tf_input_arr[idx + i] = floor(2000 - 2000 / (freq / 2.0) * i);
		}
		idx += freq / 4;

		for (i = 0; i < freq / 2; ++i) {
			tf_input_arr[idx + i] = floor(1000 + 2000 / (freq / 2.0) * i);
		}
		idx += freq / 2;

		for (i = 0; i < 3 * freq / 4; ++i) {
			tf_input_arr[idx + i] = floor(3000 - 2000 / (freq / 2.0) * i);
		}
		idx += 3 * freq / 4;
	}

	for (int freq : freqs) {

	for (i = 0; i < freq / 2; ++i) {

		tf_input_arr[idx + i] = floor(4096 / (freq / 2.0) * i);

		tf_input_arr[idx + i + freq / 2] = floor(4096 - 4096 / (freq / 2.0) * i);
	}

	idx += freq;
}




	}
