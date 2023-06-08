#ifndef ROBOKIT_TALKER_H
#define ROBOKIT_TALKER_H
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include "robokit/utils/json/json.h"
#include "ConvertImage.h"

class Talker {
public:
	enum MsgTypeIdx {
		Image = 0,
		Points = 1,
		KeyFrame = 2,
		POSE_GRAPH = 3,
		Pose = 4,
		Odom =5
	};
	Talker();
	~Talker();
	void Init(const std::string &ip);
	void PubKeyFrame(const vector< Eigen::Isometry3d> &frames);
	void PubPoints(const std::vector<Eigen::Vector3d>& points);
	void PubPoseGraph(const vector< Eigen::Isometry3d> &PoseGraph);
	void PubPose(const Eigen::Isometry3d &Pose);
	void PubImage(cv::Mat &imput);
private:
	void Pub(nlohmann::json &metadata);
	std::unique_ptr<ImagemConverter> converter_;
};
#endif
