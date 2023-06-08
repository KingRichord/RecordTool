#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#ifndef EYEWHEEL_PARAMETERS_H
#define EYEWHEEL_PARAMETERS_H

struct ProcessorConfig {
	int pyramid_levels;
	int patch_size;
	int fast_threshold;
	int max_iteration;
	double track_precision;
	double ransac_threshold;
	
	int max_features_num;
	int min_distance;
	bool flag_equalize;
	
	int img_rate;
	int pub_frequency;
};
enum StateOrder
{
	O_P = 0,
	O_R = 3,
};
extern double VEL_N_wheel;
extern double GYR_N_wheel;
extern double TD_WHEEL;


extern double kl;
extern double kr;
extern double b;


extern ProcessorConfig processor_config;
extern std::string config_file;

extern std::string cam_distortion_model;
extern cv::Vec2i cam_resolution;
extern cv::Vec4d cam_intrinsics;
extern cv::Vec4d cam_distortion_coeffs;
extern Eigen::Isometry3d T_body_cam;
extern Eigen::Matrix3d K;
extern float square_size;
extern int board_col;
extern int board_row;
bool loadParameters(std::string config_file);
void set_part_Config(std::vector<double> &camera_matrix, std::vector<double> &coeffs,
                     Eigen::Isometry3d &T_body_cam_, int image_height, int image_width);
#endif //EYEWHEEL_PARAMETERS_H
