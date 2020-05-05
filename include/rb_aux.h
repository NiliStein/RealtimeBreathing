#ifndef RBAUX_H
#define RBAUX_H
#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <CvPlot/cvplot.h>
//for logging purposes
#include <fstream>


#define NUM_OF_LAST_FRAMES 256 
#define CONFIG_FILEPATH "config.txt"
#define GET_FREQUENCY_BY_FFT true	//if false, use get_frequency_differently


enum sticker_color {
	YELLOW,
	BLUE,
	GREEN,
	RED
};

enum dimension {
	D2,
	D3
};

enum graph_mode {
	DISTANCES,
	LOCATION,
	FOURIER,
	NOGRAPH
};

enum stickers {
	left,
	mid1,
	mid2,
	mid3,
	right,
	sdummy // needed for enum iteration
};

enum distances {
	left_mid1,
	left_mid2,
	left_mid3,
	left_right,
	right_mid1,
	right_mid2,
	right_mid3,
	mid1_mid2,
	mid1_mid3,
	mid2_mid3,
	ddummy // needed for enum iteration
};

/* 
	Configuration details extracted from config.txt
	@ mode: indeicates the kind of graph to be presented.
		distances - tracking a given set of distances. bpm will be calculated using the average of said set.
		location - tracking the location of a given set of stickers. (TODO: no bpm calculated for this mode?)
	@ stickers_included: set of stickers to include. 
		for mode distances, TODO: any sticker required for an included distance? or it wont be used, tbd 
		for mode location, all stickers of which the location is requested.
	@ dists_included: set of distances to be tracked
		only used in mode distances.
*/
class Config {
public:
	dimension dimension;
	graph_mode mode;
	std::map<stickers, bool> stickers_included;
	std::map<distances, bool> dists_included;
	sticker_color color;

	//ctor:
	Config(const char* config_filepath, int* res);
};

/*	BreathingFrameData class
	Stores the processed data of a frame.
	@ circles : saves the centroids of the circles found representing the stickers.
		Each circle has x,y coordinates and depth, indexes of each are in the corresponding order.
		First circle is the frame center.
	@ left, right, mid1-3 : Pointers to the coordinates of the corresponding sticker, as following:
		left ---- mid1 ---- right
		  -					 -
		    -	  mid2	   -
			  -			 -
			    - mid3 -
		Values can be invalid if stickers do not exist. Careful!
	@ ***_cm : coordinates in cm.
	@ dAB : distance in pixels between A and B, while A,B are initials of the names of the stickers.
	@ dAB_depth : 3D distance in cm between A and B, while A,B are initials of the names of the stickers.
	@ average_2d_dist, average_3d_dist : average distances between stickers, in 2D and in 3D.
	@ ***_timestamp : timestamp of each frame.
*/
class BreathingFrameData {
public:
	std::vector<cv::Vec3f> circles; //(x,y,depth)
	cv::Vec3f *left, *right, *mid1, *mid2, *mid3; //(x,y,depth)
	//TODO: adding alternative coordinates in cm, choose one after deciding which is more accurate (dist by pixels or by cm(given by rs2_deproject))
	cv::Vec3f left_cm, right_cm, mid1_cm, mid2_cm, mid3_cm; //(x,y,depth)
	std::map<stickers, cv::Vec3f*> stickers_map_cm;
	float dLM1, dLM2, dLM3, dLR, dRM1, dRM2, dRM3, dM1M2, dM1M3, dM2M3;
	std::map<distances, float*> distances_map_2d;
	float dLM1_depth, dLM2_depth, dLM3_depth, dLR_depth, dRM1_depth, dRM2_depth, dRM3_depth, dM1M2_depth, dM1M3_depth, dM2M3_depth;
	std::map<distances, float*> distances_map_3d;
	//float dLR_depth, dML_depth, dMR_depth, dMD_depth, dDL_depth, dDR_depth;
	float average_2d_dist;
	float average_3d_dist;
	double color_timestamp;
	double depth_timestamp;
	double system_timestamp;
	unsigned long long frame_idx, color_idx, depth_idx;
	double system_color_timestamp, system_depth_timestamp;
	
	//ctor:
	BreathingFrameData() :
		left(NULL), right(NULL), mid1(NULL), mid2(NULL), mid3(NULL),
		stickers_map_cm({ {stickers::left, &left_cm}, {stickers::mid1, &mid1_cm}, {stickers::mid2, &mid2_cm}, {stickers::mid3, &mid3_cm}, {stickers::right, &right_cm} }),
		dLM1(0.0), dLM2(0.0), dLM3(0.0), dLR(0.0), dRM1(0.0), dRM2(0.0), dRM3(0.0), dM1M2(0.0), dM1M3(0.0), dM2M3(0.0),
		distances_map_2d({ {distances::left_mid1, &dLM1}, {distances::left_mid2, &dLM2}, {distances::left_mid3, &dLM3}, {distances::left_right, &dLR},
			{distances::right_mid1, &dRM1}, {distances::right_mid2, &dRM2}, {distances::right_mid3, &dRM3}, {distances::mid1_mid2, &dM1M2},
			{distances::mid1_mid3, &dM1M3}, {distances::mid2_mid3, &dM2M3} }),
		dLM1_depth(0.0), dLM2_depth(0.0), dLM3_depth(0.0), dLR_depth(0.0), dRM1_depth(0.0), dRM2_depth(0.0), dRM3_depth(0.0), dM1M2_depth(0.0), dM1M3_depth(0.0), dM2M3_depth(0.0),
		distances_map_3d({ {distances::left_mid1, &dLM1_depth}, {distances::left_mid2, &dLM2_depth}, {distances::left_mid3, &dLM3_depth}, {distances::left_right, &dLR_depth},
			{distances::right_mid1, &dRM1_depth}, {distances::right_mid2, &dRM2_depth}, {distances::right_mid3, &dRM3_depth}, {distances::mid1_mid2, &dM1M2_depth}, 
			{distances::mid1_mid3, &dM1M3_depth}, {distances::mid2_mid3, &dM2M3_depth} }),
		//dLR_depth(0.0), dML_depth(0.0), dMR_depth(0.0), dMD_depth(0.0), dDL_depth(0.0), dDR_depth(0.0),
		average_2d_dist(0.0), average_3d_dist(0.0),
		color_timestamp(0.0), depth_timestamp(0.0)
	{}

	//METHODS://

	/* Updates the locations of the stickers and validates the pointers to them. */
	void UpdateStickersLoactions();

	/* Calculates 2D distances between all stickers and their average. */
	void CalculateDistances2D(Config* user_cfg);
	/* Calculates 3D distances between all stickers and their average. */
	void CalculateDistances3D(Config* user_cfg);

	/* Gets the description of the frame in the following format:
		TODO: update format */
	void GetDescription();
	/* for a log with precision 2*/
};


/* FrameManager class.
	Manages all frames and the memory required for them.
*/
class FrameManager {
public:
	clock_t manager_start_time;
	Config* user_cfg;

	//ctor
	FrameManager(Config* user_cfg, unsigned int n_frames = NUM_OF_LAST_FRAMES, const char * frame_disk_path = NULL);

	//dtor
	~FrameManager();
	
	/* reset frame manager for another run (switch between files or between live camera to file and vice versa */
	void reset();

	int get_frames_array_size();

	/**
	 * Processes a video color frame
	 *
	 * @param frame - a video frame from the camera
	 *
	 */
	void process_frame(const rs2::video_frame& color_frame, const rs2::depth_frame& depth_frame);

	/* Turn interval activity on/off: */
	void activateInterval();
	void deactivateInterval();
	/**
	* To be used in L mode (for plotting locations of stickers)
	* TODO: for now, return only z coordinate (depth)
	* returns system_timestamp and according depth of sticker s for every frame received in the last 15 seconds
	* if called in L mode, no points are pushed to vector out
	*/
	void get_locations(stickers s, std::vector<cv::Point2d> *out);

	/**
	* To be used in D mode (for plotting avg distance of stickers)
	* To be used in F mode for calculating fft
	* returns system_timestamp and according avg distance of every frame received in the last 15 seconds
	* the avg distance is calculated only for distances set to true in user_cfg.dists_included
	* if called in L mode, no points are pushed to vector out
	*/
	void get_dists(std::vector<cv::Point2d> *out);

	/**
	* To be used in N mode (for calculating frequency and BPM without generating any graphs)
	* returns frequency calculated using get_frequency_fft if GET_FREQUENCY_BY_FFT. using get_frequency_differently otherwise.
	* the avg distances used for crequency calculation are the  distances set to true in user_cfg.dists_included
	*/
	long double no_graph();

	

private:
	/**
	 * Cleans all allocated resources
	 */
	void cleanup();

	/**
	 * Add frame_data to collection of frame datas.
	 * NOTE: only last n_frames saved so the oldest frame_data will be deleted 
	 */
	void add_frame_data(BreathingFrameData * frame_data);

	int frame_idx = 1;
	double first_timestamp = NULL;
	unsigned int _n_frames;
	unsigned int _oldest_frame_index;
	BreathingFrameData** _frame_data_arr;
	const char* _frame_disk_path;
	bool interval_active;
	int frames_dumped_in_row = 0; //reinitialized after cleanup

};

/*	GraphPlot class.
	Used for graph plotting.
	@ window : pointer to the graph window (created only once)
	@ axes : axes created for the graph. More information in the documentation of CvPlot.
	@ time_begin : beginning time of the plotting.
	@ mode : the mode of the graph as configured in the config file. 
	@ dimension : the dimension defined in the config file.
	@ first_plot : boolen indicating if the graph is being plotted from the beginning or not.
*/
class GraphPlot {
private:
	CvPlot::Window* window;
	CvPlot::Axes axes;
	clock_t time_begin;
	graph_mode _mode;
	dimension _dimension;
	bool first_plot;

	void _plotFourier(std::vector<cv::Point2d>& points);
	void _plotDists(std::vector<cv::Point2d>& points);
	void _plotLoc(std::vector<cv::Point2d>& points, const char * lineSpec);
	void _plotNoGraph(std::vector<cv::Point2d>& points);
	void _init_plot_window();

public:

	//ctor:
	GraphPlot(graph_mode mode, dimension dimension, clock_t start_time);
	
	void reset(clock_t start_time);

	void plot(std::vector<cv::Point2d>& points, const char * lineSpec = NULL);

};

#endif