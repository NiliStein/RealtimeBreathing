#ifndef UTILITIES_H
#define UTILITIES_H
#pragma once
#include <librealsense2/rsutil.h>

#define PI 3.14159265358979323846

#define LOG_TITLES_4_STICKERS(D2units)"Frame_index,Color idx,Depth idx,Color timestamp,Depth timestamp,System color timestamp,System depth timestamp\
,System timestamp,left (x y z) cm,right (x y z) cm,mid2  (x y z) cm,mid3  (x y z) cm,left (x y)" + D2units + ",right (x y) " + D2units + ",mid2 (x y) " + D2units + \
",mid3 (x y) " + D2units + ",left - mid2 2D distance " + D2units + ",left - mid3 2D distance " + D2units + ",left - right 2D distance " + D2units + \
",right - mid2 2D distance " + D2units + ",right - mid3 2D distance " + D2units + ",mid2 - mid3 2D distance " + D2units + ",left - mid2 3D distance (cm)\
,left - mid3 3D distance (cm),left - right 3D distance (cm),right - mid2 3D distance (cm),right - mid3 3D distance (cm),mid2 - mid3 3D distance (cm)\
,2D average distance,3D average distance,FPS,realSamplesNum,frequency,BPM\n"

#define LOG_TITLES_5_STICKERS(D2units) "Frame_index,Color idx,Depth idx,Color timestamp,Depth timestamp,System color timestamp,System depth timestamp\
,System timestamp,left (x y z) cm,right (x y z) cm,mid1(x y z) cm,mid2  (x y z) cm,mid3  (x y z) cm,left (x y)" + D2units + ",right (x y) " + D2units + \
",mid1 (x y) " + D2units + ",mid2 (x y) " + D2units + ",mid3 (x y) " + D2units + ",left - mid1 2D distance " + D2units + ",left - mid2 2D distance " + D2units + \
",left - mid3 2D distance " + D2units + ",left - right 2D distance " + D2units + ",right - mid1 2D distance " + D2units + ",right - mid2 2D distance " + \
D2units + ",right - mid3 2D distance " + D2units + ",mid1 - mid2 2D distance " + D2units + ",mid1 - mid3 2D distance " + D2units + ",mid2 - mid3 2D distance " + \
 D2units + ",left - mid1 3D distance (cm),left - mid2 3D distance (cm),left - mid3 3D distance (cm),left - right 3D distance (cm),right - mid1 3D distance (cm),\
right - mid2 3D distance (cm),right - mid3 3D distance (cm),mid1 - mid2 3D distance (cm),mid1 - mid3 3D distance (cm),mid2 - mid3 3D distance (cm),2D average \
distance,3D average distance,FPS,realSamplesNum,frequency,BPM\n"


// for logging
std::ofstream logFile;

void init_logFile(const char* filename, int num_of_stickers, std::string D2units) {
	std::string name_prefix;
	if (filename) {
		//std::string file_name = filename;
		name_prefix = "file_log"; //file_name.substr(file_name.find("\\"), file_name.find("\."));
	}
	else {
		name_prefix = "live_camera_log";
	}
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
	std::string log_name = name_prefix + "_" + oss.str() + ".csv";
	logFile.open(log_name);
	(num_of_stickers == 4) ? logFile << LOG_TITLES_4_STICKERS(D2units) : logFile << LOG_TITLES_5_STICKERS(D2units);
}

// fill out with n values, evenly spaced between a and b, including a and b
void linespace(double a, double b, int n, std::vector<double>* out) {
	double step = (b - a) / (n - 1);
	for (int i = 0; i < n - 1; i++) {
		out->push_back(a);
		a += step;           // could recode to better handle rounding errors
	}
	out->push_back(b);
}

static float distance2D(float x, float y, float a, float b) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2));
}

static float distance3D(float x, float y, float z, float a, float b, float c) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2) + pow(z - c, 2));
}

void get_3d_coordinates(const rs2::depth_frame& depth_frame, float x, float y, cv::Vec3f& output) {
	float pixel[2] = { x, y };
	float point[3]; // From point (in 3D)
	auto dist = depth_frame.get_distance(pixel[0], pixel[1]);

	rs2_intrinsics depth_intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

	rs2_deproject_pixel_to_point(point, &depth_intr, pixel, dist);

	//convert to cm
	output[0] = float(point[0]) * 100.0;
	output[1] = float(point[1]) * 100.0;
	output[2] = float(point[2]) * 100.0;

}

bool check_illegal_3D_coordinates(const BreathingFrameData* breathing_data) {
	bool illegal_3d_coordinates = false;
	//check for 0,-0,0 3d coordinates.
	for (std::pair<stickers, cv::Vec3f*> sticker_elem : breathing_data->stickers_map_cm) {
		stickers s = sticker_elem.first;
		cv::Vec3f* d3_coor = sticker_elem.second;
		if ((*d3_coor)[0] == 0.0 && (*d3_coor)[1] == -0 && (*d3_coor)[2] == 0) {
			illegal_3d_coordinates = true;
			break;
		}
	}
	return illegal_3d_coordinates;
}

void FFT(short int dir, long m, double *x, double *y)
{
	long n, i, i1, j, k, i2, l, l1, l2;
	double c1, c2, tx, ty, t1, t2, u1, u2, z;
	/* Calculate the number of points */
	n = 1;
	for (i = 0; i < m; i++)
		n *= 2;
	/* Do the bit reversal */
	i2 = n >> 1;
	j = 0;


	for (i = 0; i < n - 1; i++) {
		if (i < j) {
			tx = x[i];
			ty = y[i];
			x[i] = x[j];
			y[i] = y[j];
			x[j] = tx;
			y[j] = ty;
		}
		k = i2;
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}

	/* Compute the FFT */
	c1 = -1.0;
	c2 = 0.0;
	l2 = 1;
	for (l = 0; l < m; l++) {
		l1 = l2;
		l2 <<= 1;
		u1 = 1.0;
		u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (i = j; i < n; i += l2) {
				i1 = i + l1;
				t1 = u1 * x[i1] - u2 * y[i1];
				t2 = u1 * y[i1] + u2 * x[i1];
				x[i1] = x[i] - t1;
				y[i1] = y[i] - t2;
				x[i] += t1;
				y[i] += t2;
			}
			z = u1 * c1 - u2 * c2;
			u2 = u1 * c2 + u2 * c1;
			u1 = z;
		}
		c2 = sqrt((1.0 - c1) / 2.0);
		if (dir == 1)
			c2 = -c2;
		c1 = sqrt((1.0 + c1) / 2.0);
	}

	/* Scaling for forward transform */
	if (dir == 1) {
		for (i = 0; i < n; i++) {
			x[i] /= n;
			y[i] /= n;
		}
	}
}

/*
 * low pass filter. currently not in use.
 * in - samples to be filtered
 * dt - time delta between two samples
 * fc - cutoff frequency
 * size - number of samples
 * returns filtered samples in param out
 */
void LPF(double* in, double dt, double fc, double* out, int size) {
	double RC = 1 / (2 * PI * fc);
	double alpha = dt / (RC + dt);
	out[0] = (alpha * in[0]);
	for (int i = 1; i < size; i++) {
		out[i] = (alpha *  in[i] + (1 - alpha) * out[i - 1]);
	}
}

/*
 * high pass filter. currently not in use.
 * in - samples to be filtered
 * dt - time delta between two samples
 * fc - cutoff frequency
 * size - number of samples
 * returns filtered samples in param out
 */
void HPF(double* in, double dt, double fc, double* out, int size) {
	double RC = 1 / (2 * PI * fc);
	double alpha = dt / (RC + dt);
	out[0] = in[0];
	for (int i = 1; i < size; i++) {
		out[i] = (alpha *  out[i - 1] + alpha * (in[i] - in[i - 1]));
	}
}


/*
* To be used in D mode
* returns most dominant frequency, calculated for average distance in frames received in the last 15 seconds
* the avg distance is calculated only for distances set to true in user_cfg.dists_included
*/
long double calc_frequency_fft(std::vector<cv::Point2d>* samples, std::vector<cv::Point2d>* out_frequencies = NULL) {
	if (samples->size() < 5) {
		logFile << '\n';
		return 0;
	}
	int realSamplesNum = samples->size();	// N - number of samples (frames)
	int realSamplesLog = log2(realSamplesNum);
	// fft requires that the number of samples is a power of 2. add padding if needed
	const int paddedSamplesNum = (realSamplesNum == pow(2, realSamplesLog)) ? realSamplesNum : pow(2, realSamplesLog + 1);	
	int dir = 1;
	long m = log2(paddedSamplesNum);
	double* X = new double[paddedSamplesNum];
	double* Y = new double[paddedSamplesNum];
	// insert real samples in first #realSamplesNum slots
	for (int s = 0; s < realSamplesNum; s++) { 
		X[s] = samples->at(s).y;
		Y[s] = 0;	// no imaginary part in samples
	}
	// add padding after real samples
	for (int s = realSamplesNum; s < paddedSamplesNum; s++) { 
		X[s] = 0;
		Y[s] = 0;
	}
	
	double t0 = samples->at(0).x;	// t0, t1 - start, end time(seconds)
	double t1 = samples->at(realSamplesNum - 1).x;
	if (t1 == t0) return 0;
	double fps = realSamplesNum / (t1 - t0);	// FPS - frames per second

	FFT(dir, m, X, Y);

	// following comented code return the average of -top- frequencies found by fourier
	/*const int top = 100;
	int top_max_idx[top] = { 0 };
	double min_bpm = 7.0;
	int mini = ceil((min_bpm / 60.0)*((paddedSamplesNum - 2.0) / fps));	// assume BPM >= min_bpm
	for (int j = 0; j < top; j++) {
		int max_idx = 0;
		double max_val = 0;
		for (int i = mini; i < realSamplesNum / 2; i++) { //frequency 0 always most dominant. ignore first coef.
			double val = abs(X[i])*abs(X[i]) + abs(Y[i])*abs(Y[i]);
			if (val > max_val) {
				max_val = val;
				max_idx = i;
			}
		}
		top_max_idx[j] = max_idx;
		X[max_idx] = 0;
	}
	double avg_max_idx = 0;
	for (int i = 0; i < top; i++)  avg_max_idx += top_max_idx[i];
	avg_max_idx /= top;
	long double f = fps / (paddedSamplesNum - 2.0) * top_max_idx[0];
	long double f_avg = fps / (paddedSamplesNum - 2.0) * avg_max_idx;
	*/

	int max_idx = 0;
	double max_val = 0;
	double min_bpm = 5.0;	// TODO: decide minimal BPM
	int mini = ceil((min_bpm / 60.0)*((paddedSamplesNum - 2.0) / fps));	// assume BPM >= min_bpm
	for (int i = 0; i < realSamplesNum / 2; i++) { //frequency 0 always most dominant. ignore first coef.
		double val = abs(X[i])*abs(X[i]) + abs(Y[i])*abs(Y[i]);	
		if (out_frequencies != NULL) {
			double frequency = fps / (paddedSamplesNum - 2.0) * i;
			//out_frequencies->push_back(cv::Point2d(frequency, abs(X[i])));
			out_frequencies->push_back(cv::Point2d(frequency, val));
		}
		if (i >= mini && val > max_val) {
			max_val = val;
			max_idx = i;
		}
	}

	delete X;
	delete Y;
	
	logFile << fps << ',';
	logFile << realSamplesNum << ',';
	long double f = fps / (paddedSamplesNum - 2.0) * max_idx;
	logFile << std::fixed <<std::setprecision(6) << f << ',';
	logFile << std::fixed << std::setprecision(6) << f*60.0 << '\n';
	return f;
}


/* 
 * calc_frequency_differently finds maximum points in given samples and caculates average time elapsed between two peaks
 * the method uses two variables that mey need tuning:
 * neighborhood - a maximum point will only count as a peak if its value is greater then all of its neighbors (needed to screen adjacant samples describing the same peak)
 * threshold percentage -  a point will only count as a peak if its value is greate than max_threshold = max_dist - threshold_precentage * length (different peaks have
 * different values. this threshold is needed to screen low local maximum points)
 * return frequency
 */
long double calc_frequency_differently(std::vector<cv::Point2d>* samples, bool cm_units) {
	if (samples->size() < 40) {
		logFile << "\n";
		return 0;
	}
	int neighborhood = (cm_units) ? 3 : 5;
	double threshold_precentage = (cm_units) ? 27.0 / 100.0 : 45.0 / 100.0;
	
	double max_dist = samples->at(0).y;
	double min_dist = samples->at(0).y;
	double length, max_threshold;
	std::vector<std::tuple<double, double, int>> peaks;	//timestamp, avg distance, index in samples
	for (int i = 0; i < samples->size(); i++) {
		double curr_dist = samples->at(i).y;
		if (curr_dist > max_dist) max_dist = curr_dist;
		if (curr_dist < min_dist) min_dist = curr_dist;
	}
	length = max_dist - min_dist;
	max_threshold = max_dist - threshold_precentage * length;
	for (int i = neighborhood; i < samples->size() - neighborhood; i++) {
		double curr_dist = samples->at(i).y;
		if (curr_dist < max_threshold) continue;
		bool is_max_in_neighborhood = true;
		for (int j = 1; j <= neighborhood; j++) {
			double prev_neighbor = samples->at(i - j).y;
			double next_neighbor = samples->at(i + j).y;
			if (curr_dist < prev_neighbor || curr_dist < next_neighbor) {
				is_max_in_neighborhood = false;
				break;
			}
		}
		if (is_max_in_neighborhood) {
			peaks.push_back(std::tuple<double, double, int>(samples->at(i).x, curr_dist, i));
		}
	}
	
	if (peaks.size() < 2) {
		logFile << "\n";
		return 0;
	}
	//get avg time lapse between picks
	double time_lapses = 0.0;
	for (int i = 0; i < peaks.size() - 1; i++) {
		time_lapses += std::get<0>(peaks.at(i + 1)) - std::get<0>(peaks.at(i));
	}
	long double breath_cycle = time_lapses / (peaks.size() - 1.0);
	//logFile << "\nLength: " << length;
	//logFile << "\nmax_dist: " << max_dist;
	//logFile << "\nmin_dist: " << min_dist;
	//for (int i = 0; i < peaks.size(); i++) {
		//logFile << "\npick " << i << ": " << std::get<1>(peaks[i]) << " , " << std::get<0>(peaks[i]);
	//}
	//logFile << "\nNumber of picks: " << peaks.size();
	//logFile << "\nTotal time_lapses: " << time_lapses;
	//logFile << "\nbreath cycle: " << breath_cycle;
	
	long double bpm = 60.0 / breath_cycle;
	long double f = bpm / 60.0;
	
	logFile << "-" << ","; // FPS coloumn
	logFile << samples->size() << ","; // realSamplesNum coloumn
	logFile << f << ",";
	logFile << bpm << "\n";
	return f;

}

void normalize_distances(std::vector<cv::Point2d>* samples) {
	if (samples->size() < 3) return;
	double max_dist = samples->at(0).y;
	double min_dist = samples->at(0).y;
	for (int i = 1; i < samples->size(); i++) {
		double curr_dist = samples->at(i).y;
		if (curr_dist > max_dist) max_dist = curr_dist;
		if (curr_dist < min_dist) min_dist = curr_dist;
	}
	for (int i = 0; i < samples->size(); i++) {
		double temp_t = samples->at(i).x;
		double temp_d = samples->at(i).y;
		double norm_d = 2 * (temp_d - min_dist) / (max_dist - min_dist) - 1;
		samples->at(i) = cv::Point2d(temp_t, norm_d);
	}
}

/*
 *	parse samples from a text file in the following format:
 *	new line for each sample. samples consist of time and distance separated by space (" "). time before distance.
 *	for exaple:
 *	0 0.0
 *	0.125 0.3090169943749474
 *	0.25 0.5877852522924731
 *	samples are normalized and inserted to given vector
 */
void get_samples_from_file(std::vector<cv::Point2d>* samples) {
	std::ifstream samples_file("alons_samples.txt");
	std::string line;

	while (std::getline(samples_file, line)) {
		std::string first = line.substr(0, line.find(" "));
		std::string second = line.substr(line.find(" "), line.length());

		double time = std::stod(first);
		double dist = std::stod(second);
		samples->push_back(cv::Point2d(time, dist));
	}
	normalize_distances(samples);
}

// insert samples relevant for current window of size 250 (from start_index to 250 + start_index) to out_window_samples
void simulate_running_window(int start_index, std::vector<cv::Point2d>* samples, std::vector<cv::Point2d>* out_window_samples) {
	int end_index = (start_index + 250 > samples->size()) ? samples->size() : start_index + 250;
	for (unsigned int i = start_index; i < end_index; i++) {
		double avg_dist = samples->at(i).y;
		double t = samples->at(i).x;
		out_window_samples->push_back(cv::Point2d(t, avg_dist));
	}
	
}

#endif