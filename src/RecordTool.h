#ifndef _IRCAMERAPOSE_H_
#define _IRCAMERAPOSE_H_
// 库
#include <robokit/core/rbk_core.h>
#include <robokit/utils/logger/logger.h>
#include <robokit/utils/error/error.h>
// Eigen
#include <Eigen/Geometry>
// OpenCV
#include <opencv2/core.hpp>
// 消息类
#include "message_odometer.pb.h"
// 记录时间
#include "tic_toc.h"
// 可视化消息传输
#include "communication/talker.h"
// 外参标定核心
#include "Parameters.h"
// 图像信息格式
struct ImageTransmissionData
{
	long time_stamp;
	cv::Mat img;
};
// 里程计信息格式
struct OdomInf {
	long time_stamp;  //纳秒级时间戳
	Eigen::Isometry3d T_odom_body;
};


using namespace rbk::protocol;
using namespace rbk;

class RecordTool : public NPluginInterface {
public:
	RecordTool();
	
	~RecordTool();
	
	void run();
	
	void loadFromConfigFile();
	
	void setSubscriberCallBack();
	
	void MessageLocalizationCallBack(google::protobuf::Message *msg); // 读取定位信息
	// void modelChangedSubscriber(google::protobuf::Message *msg);        // 更新模型
	// 更新模型
	void modelChangedSubscriber(std::vector<std::string> mtv);
	bool init();                                                       // 初始化模型
	
	std::atomic<bool> slam_status{false};       // 是否开启slam系统
	std::atomic<bool> vo_status{false};	        // 视觉里程计功能开启

private:
	// 将外参转换成矩阵
	Eigen::Isometry3d transformToRobot(double &yaw, double &pitch, double &roll,
	                                   double &x, double &y, double &z);
	std::atomic<bool> m_model_changed{false};	// 模型是不是更改的标志位
	
	bool m_use_for_localization{false};       	// 是不是用于定位的相机
	bool m_cam_ex_param_calibrated_{false}; 	// 相机外参是否标定完成
	// 相机内参
	double m_fx_{}, m_fy_{}, m_cx_{}, m_cy_{}, m_k1_{}, m_k2_{}, m_k3_{}, m_k4_{}, m_k5_{}, m_k6_{}, m_p1_{}, m_p2_{};
	// 相机外参
	double m_x_{}, m_y_{}, m_z_{}, m_yaw_{}, m_pitch_{}, m_roll_{};
	
	long last_time_stamp{-1};                       // 上一时刻相机数据到达的时间
	bool cam_odom_computed_finshed{false};
	std::mutex m_odom_lock_;                                // 里程计消息锁
	std::queue<OdomInf> odom_buf_;                          // 时间窗口中的里程计信息
	std::shared_ptr<Talker> m_talker_;
	std::thread process;
	bool fist_init{true};
	std::string path_to_save_data_;
	
};

RBK_INHERIT_PROVIDER(RecordTool, NPluginInterface, "1.0.0");
#endif
