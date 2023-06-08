#include "RecordTool.h"
RBK_INHERIT_SOURCE(RecordTool)
RecordTool::RecordTool() {
	m_model_changed = true; // 模型是不是发生了改变
	m_talker_ = std::make_shared<Talker>();
	LogInfo("============正在建立连接===================")
	std::string ip = "tcp://192.168.192.5:8002";	LogInfo(ip)
	
	m_talker_->Init(ip);
	process.detach();
}
RecordTool::~RecordTool()
{
	while (process.joinable())
		process.join();
	// estimator_.reset();
	m_talker_.reset();
}
void RecordTool::run() {
	while (true) {
		if (m_model_changed) {
			m_model_changed = false;
			LogWarn("相机模型发现变化")
			SLEEP(10);
			init();
		}
		if (vo_status == false) {
			LogWarn("开启 VO 失败!")
			SLEEP(10);
			continue; //跳出也要保证时间间隔
		}
		ImageTransmissionData tmp{};
		if (rbk::utils::globaldata::GlobalData::instance()->get("infrared_camera_image", tmp)) {
			if (tmp.img.empty()) continue;
			if (tmp.time_stamp <= last_time_stamp) continue;
			last_time_stamp = tmp.time_stamp;
			double timestamp = (double )tmp.time_stamp/1000000000.;
			cv::Mat cur_img=  tmp.img.clone();
			// estimator_->inputImage( timestamp,cur_img);
		} else {
			LogError("Global variable: infrared_camera_Image, not found, please confirm the version of MutiDcamera!")
		}
		SLEEP(40);
	}
}

void RecordTool::loadFromConfigFile() {}

void RecordTool::setSubscriberCallBack() {
	// setTopicCallBack<rbk::protocol::Message_2D_CamInfo>(&CameraOdometer::modelChangedSubscriber, this);
	setTopicCallBack<rbk::protocol::Message_Odometer>(&RecordTool::MessageLocalizationCallBack, this);
	setModelChangeCallBack(std::bind(&RecordTool::modelChangedSubscriber, this, std::placeholders::_1));
	setInitialized(true);
}

// 调用机器人的定位信息,构造全局大SEERTAG地图
void RecordTool::MessageLocalizationCallBack(google::protobuf::Message *msg) {
	Message_Odometer data;
	data.CopyFrom(*msg);
	Eigen::Vector3d vel{0.,0.,0.};
	Eigen::Vector3d gyr{0.,0.,0.};
	double timestamp = (double)data.header().data_nsec()/1000000000.;
	vel = Eigen::Vector3d(data.vel_x(), 0., 0.);
	gyr=  Eigen::Vector3d(0., 0.,data.vel_rotate());
	// estimator_->inputWheel(timestamp,vel,gyr);
}
void RecordTool::modelChangedSubscriber(std::vector<std::string> mtv) {\
	if (std::find(mtv.begin(), mtv.end(), "camera") != mtv.end()) {
		LogWarn("Detect changes in camera model")
		m_model_changed = true;
	}
}

bool RecordTool::init() {
	bool status{false};// 是否获取正确的配置标志位
	// 一旦相机配置和检测配置发生变更，即触发该模块重新读取数据进行的重新初始化
	MODEL_PARAM_START(camera)
			std::string m_modeSelect = MODEL_PARAM_READ(std::string, modeSelect);
			LogWarn("m_modeSelect  ==== :  "<< m_modeSelect);
			if(m_modeSelect == "useForLocalization")
			{
				m_use_for_localization = MODEL_PARAM_READ(bool,
				                              modeSelect.useForLocalization.useForLocalization);
				LogWarn("m_use_for_localization::"<< m_use_for_localization)
				if(m_use_for_localization)
				{
					std::string m_type = MODEL_PARAM_READ(std::string, brand);
					if(m_type == "Hik-MV-TCP")
					{
						m_fx_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.fx)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.fx) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.fx) : 0);
						m_fy_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.fy)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.fy) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.fy) : 0);
						m_cx_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.cx)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.cx) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.cx) : 0);
						m_cy_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.cy)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.cy) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.cy) : 0);
						m_k1_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k1)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k1) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k1) : 0);
						m_k2_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k2)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k2) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k2) : 0);
						m_k3_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k3)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k3) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k3) : 0);
						m_k4_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k4)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k4) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k4) : 0);
						m_k5_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k5)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k5) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k5) : 0);
						m_k6_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.k6)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.k6) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.k6) : 0);
						m_p1_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.p1)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.p1) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.p1) : 0);
						m_p2_ = MODEL_PARAM_READ(double,brand.Hik-MV-TCP.p2)+(MODEL_CALIB_PARAM_EXIST(brand.Hik-MV-TCP.p2) ? MODEL_CALIB_PARAM_READ(double, brand.Hik-MV-TCP.p2) : 0);
					}
					if (m_type == "USBCamera-USB") {
						m_fx_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.fx)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.fx) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.fx) : 0);
						m_fy_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.fy)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.fy) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.fy) : 0);
						m_cx_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.cx)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.cx) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.cx) : 0);
						m_cy_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.cy)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.cy) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.cy) : 0);
						m_k1_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k1)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k1) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k1) : 0);
						m_k2_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k2)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k2) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k2) : 0);
						m_k3_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k3)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k3) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k3) : 0);
						m_k4_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k4)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k4) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k4) : 0);
						m_k5_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k5)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k5) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k5) : 0);
						m_k6_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.k6)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.k6) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.k6) : 0);
						m_p1_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.p1)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.p1) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.p1) : 0);
						m_p2_ = MODEL_PARAM_READ(double,brand.USBCamera-USB.p2)+(MODEL_CALIB_PARAM_EXIST(brand.USBCamera-USB.p2) ? MODEL_CALIB_PARAM_READ(double, brand.USBCamera-USB.p2) : 0);
					}
					if (m_type == "DaHeng-TCP") {
						m_fx_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.fx)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.fx) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.fx) : 0);
						m_fy_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.fy)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.fy) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.fy) : 0);
						m_cx_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.cx)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.cx) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.cx) : 0);
						m_cy_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.cy)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.cy) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.cy) : 0);
						m_k1_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k1)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k1) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k1) : 0);
						m_k2_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k2)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k2) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k2) : 0);
						m_k3_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k3)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k3) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k3) : 0);
						m_k4_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k4)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k4) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k4) : 0);
						m_k5_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k5)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k5) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k5) : 0);
						m_k6_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k6)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.k6) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.k6) : 0);
						m_p1_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.p1)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.p1) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.p1) : 0);
						m_p2_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.p2)+(MODEL_CALIB_PARAM_EXIST(brand.DaHeng-TCP.p2) ? MODEL_CALIB_PARAM_READ(double, brand.DaHeng-TCP.p2) : 0);
					}
					if (m_type == "Huaray-TCP") {
						m_fx_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.fx)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.fx) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.fx) : 0);
						m_fy_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.fy)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.fy) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.fy) : 0);
						m_cx_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.cx)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.cx) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.cx) : 0);
						m_cy_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.cy)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.cy) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.cy) : 0);
						m_k1_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k1)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k1) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k1) : 0);
						m_k2_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k2)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k2) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k2) : 0);
						m_k3_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k3)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k3) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k3) : 0);
						m_k4_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k4)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k4) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k4) : 0);
						m_k5_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k5)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k5) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k5) : 0);
						m_k6_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.k6)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.k6) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.k6) : 0);
						m_p1_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.p1)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.p1) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.p1) : 0);
						m_p2_ = MODEL_PARAM_READ(double,brand.DaHeng-TCP.p2)+(MODEL_CALIB_PARAM_EXIST(brand.Huaray-TCP.p2) ? MODEL_CALIB_PARAM_READ(double, brand.Huaray-TCP.p2) : 0);
					}
					// 获取相机的外参信息
					m_yaw_   = MODEL_PARAM_READ(double,basic.yaw)  +(MODEL_CALIB_PARAM_EXIST(basic.yaw)   ? MODEL_CALIB_PARAM_READ(double, basic.yaw) : 0);
					m_pitch_ = MODEL_PARAM_READ(double,basic.pitch)+(MODEL_CALIB_PARAM_EXIST(basic.pitch) ? MODEL_CALIB_PARAM_READ(double, basic.pitch) : 0);
					m_roll_  = MODEL_PARAM_READ(double,basic.roll) +(MODEL_CALIB_PARAM_EXIST(basic.roll)  ? MODEL_CALIB_PARAM_READ(double, basic.roll) : 0);
					m_x_     = MODEL_PARAM_READ(double,basic.x)+(MODEL_CALIB_PARAM_EXIST(basic.x) ? MODEL_CALIB_PARAM_READ(double, basic.x) : 0);
					m_y_     = MODEL_PARAM_READ(double,basic.y)+(MODEL_CALIB_PARAM_EXIST(basic.y) ? MODEL_CALIB_PARAM_READ(double, basic.y) : 0);
					m_z_     = MODEL_PARAM_READ(double,basic.z)+(MODEL_CALIB_PARAM_EXIST(basic.z) ? MODEL_CALIB_PARAM_READ(double, basic.z) : 0);
					// 确定是不是外参标定完成
					if(
							MODEL_CALIB_PARAM_EXIST(basic.yaw)   ||
							MODEL_CALIB_PARAM_EXIST(basic.pitch) ||
							MODEL_CALIB_PARAM_EXIST(basic.roll)  ||
							MODEL_CALIB_PARAM_EXIST(basic.x)     ||
							MODEL_CALIB_PARAM_EXIST(basic.y)     ||
							MODEL_CALIB_PARAM_EXIST(basic.z))
					{
						m_cam_ex_param_calibrated_ = true;  // 已经标定完成
						LogWarn("Camera -- external parameter -- calibration completed--")
					}
					else
					{
						m_cam_ex_param_calibrated_ = true; // 未标定
						LogWarn("Camera -- external reference --! Not calibrated--")
					}
					LogWarn("camerea module, camera initialization completed")
					status = true; break;
				}else status = false ;
			} else status = false;
	MODEL_PARAM_END(camera)
	if (!status) {
		return false;
	}
	std::vector<double> camera_matrix;
	camera_matrix.reserve(4);
	camera_matrix.emplace_back(m_fx_);
	camera_matrix.emplace_back(m_fy_);
	camera_matrix.emplace_back(m_cx_);
	camera_matrix.emplace_back(m_cy_);
	std::vector<double> coeffs;
	coeffs.reserve(8);
	coeffs.emplace_back(m_k1_);
	coeffs.emplace_back(m_k2_);
	coeffs.emplace_back(m_p1_);
	coeffs.emplace_back(m_p2_);
	coeffs.emplace_back(m_k3_);
	coeffs.emplace_back(m_k4_);
	coeffs.emplace_back(m_k5_);
	coeffs.emplace_back(m_k6_);
	Eigen::Isometry3d T_body_cam = transformToRobot
			(m_yaw_, m_pitch_, m_roll_, m_x_, m_y_, m_z_);
	int image_width = 640;
	int image_height = 480;
	set_part_Config(camera_matrix, coeffs, T_body_cam, image_height, image_width);
	vo_status = true;
	return true;
}
Eigen::Isometry3d RecordTool::transformToRobot(double &yaw, double &pitch, double &roll,
                                                   double &x, double &y, double &z) {
	Eigen::Matrix3d rotation_matrix =
			Eigen::AngleAxisd(yaw   * M_PI / 180.0,  Eigen::Vector3d::UnitZ()).toRotationMatrix()
	      * Eigen::AngleAxisd(pitch * M_PI / 180.0,  Eigen::Vector3d::UnitY()).toRotationMatrix()
	      * Eigen::AngleAxisd(roll  * M_PI / 180.0,  Eigen::Vector3d::UnitX()).toRotationMatrix();
	rotation_matrix.normalized();
	Eigen::Isometry3d pose;
	pose.setIdentity();
	pose.translate(Eigen::Vector3d(x, y, z));
	pose.rotate(rotation_matrix);
	return pose;
}