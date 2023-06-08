#include <string>
#include "Parameters.h"

ProcessorConfig processor_config{};
std::string config_file;
std::string cam_distortion_model;
cv::Vec2i cam_resolution;
cv::Vec4d cam_intrinsics;
cv::Vec4d cam_distortion_coeffs;
Eigen::Isometry3d T_body_cam;
Eigen::Matrix3d K;
float square_size = 0.024; // unit:  m


int board_col = 8;
int board_row = 5;

double TD_WHEEL;
double VEL_N_wheel;
double GYR_N_wheel;

double kl = 0.00047820240382508;
double kr = 0.00047768621928995;
double b = 1.52439;

void set_part_Config(std::vector<double> &camera_matrix, std::vector<double> &coeffs,
                             Eigen::Isometry3d &T_body_cam_, int image_height, int image_width) {
	
	
	processor_config.fast_threshold = 30;
	processor_config.patch_size = 21;
	processor_config.pyramid_levels = 2;
	processor_config.max_iteration = 10;
	processor_config.track_precision = 0.01;
	processor_config.ransac_threshold = 1;
	
	processor_config.max_features_num = 250;
	processor_config.min_distance = 10;
	processor_config.flag_equalize = false;
	
	processor_config.pub_frequency = 10;
	processor_config.img_rate = 20;
	
	cam_distortion_model = "radtan";
	// 内参和畸变矩阵配置
	cv::Mat m_k = (cv::Mat_<float>(3, 3) << camera_matrix[0],         0.,          camera_matrix[2],
			0.,                 camera_matrix[1],    camera_matrix[3],
			0.,                        0.,               1.          );
	std::cout << "m_k" <<std::endl;
	std::cout << m_k <<std::endl;
	cv::Mat m_d = (cv::Mat_<float>(5, 1) << coeffs[0], coeffs[1], coeffs[2], coeffs[3], 0.);
	std::cout << "m_d" <<std::endl;
	std::cout << m_d <<std::endl;
	
	
	cam_resolution[0] = image_width;
	cam_resolution[1] = image_height;
	
	cam_intrinsics[0] = camera_matrix[0];
	cam_intrinsics[1] = camera_matrix[1];
	cam_intrinsics[2] = camera_matrix[2];
	cam_intrinsics[3] = camera_matrix[3];
	K << cam_intrinsics[0],0,                   cam_intrinsics[2],
			0,                cam_intrinsics[1],cam_intrinsics[3],
			0.,               0.,                      1.;
	cam_distortion_coeffs[0] = coeffs[0];
	cam_distortion_coeffs[1] = coeffs[1];
	cam_distortion_coeffs[2] = coeffs[2];
	cam_distortion_coeffs[3] = coeffs[3];
	T_body_cam = T_body_cam_;
	std::cout <<"T_body_cam"<<std::endl;
	std::cout <<T_body_cam.matrix() <<std::endl;
	VEL_N_wheel = 0.01;
	GYR_N_wheel = 0.01;
	TD_WHEEL = 0.0;
}
bool loadParameters(std::string config_file) {
	config_file = config_file;
	cv::FileStorage fs_settings(config_file, cv::FileStorage::READ);
	if (!fs_settings.isOpened()) {
		std::cout << "config_file error: cannot open " << config_file << std::endl;
		return false;
	}
	processor_config.fast_threshold = fs_settings["fast_threshold"];
	processor_config.patch_size = fs_settings["patch_size"];
	processor_config.pyramid_levels = fs_settings["pyramid_levels"];
	processor_config.max_iteration = fs_settings["max_iteration"];
	processor_config.track_precision = fs_settings["track_precision"];
	processor_config.ransac_threshold = fs_settings["ransac_threshold"];
	
	processor_config.max_features_num = fs_settings["max_features_num"];
	processor_config.min_distance = fs_settings["min_distance"];
	processor_config.flag_equalize = static_cast<int>(fs_settings["flag_equalize"]) != 0;
	
	processor_config.pub_frequency = fs_settings["pub_frequency"];
	processor_config.img_rate = fs_settings["img_rate"];
	
	fs_settings["distortion_model"] >> cam_distortion_model;
	cam_resolution[0] = fs_settings["resolution_width"];
	cam_resolution[1] = fs_settings["resolution_height"];
	cv::FileNode n_instrin = fs_settings["intrinsics"];
	cam_intrinsics[0] = static_cast<double>(n_instrin["fx"]);
	cam_intrinsics[1] = static_cast<double>(n_instrin["fy"]);
	cam_intrinsics[2] = static_cast<double>(n_instrin["cx"]);
	cam_intrinsics[3] = static_cast<double>(n_instrin["cy"]);
	
	K << cam_intrinsics[0],0,                cam_intrinsics[2],
	     0,                cam_intrinsics[1],cam_intrinsics[3],
		 0.,               0.,                      1.;
	cv::FileNode n_distort = fs_settings["distortion_coeffs"];
	cam_distortion_coeffs[0] = static_cast<double>(n_distort["k1"]);
	cam_distortion_coeffs[1] = static_cast<double>(n_distort["k2"]);
	cam_distortion_coeffs[2] = static_cast<double>(n_distort["p1"]);
	cam_distortion_coeffs[3] = static_cast<double>(n_distort["p2"]);
	
	// 相机和车体之间的外参矩阵
	cv::Mat T_body_cam_mat;
	fs_settings["T_body_cam"] >> T_body_cam_mat;
	Eigen::Matrix4d  T_body_cam_;
	cv::cv2eigen(T_body_cam_mat,T_body_cam_);
	T_body_cam =T_body_cam_;
	std::cout <<"T_body_cam"<<std::endl;
	std::cout <<T_body_cam.matrix() <<std::endl;
	
	VEL_N_wheel = fs_settings["wheel_velocity_noise_sigma"];
	GYR_N_wheel = fs_settings["wheel_gyro_noise_sigma"];
	TD_WHEEL = fs_settings["wheel_cam_td"];
	return true;
}