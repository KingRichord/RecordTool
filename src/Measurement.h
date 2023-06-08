#pragma once

#include <memory>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

enum class MeasureType {
    kUnknown,
    kWheel,
    kMonoImage,
    kSimMonoImage,
    kGNSS
};
struct Measurement {
    virtual ~Measurement() { }

    double timestamp;
    MeasureType type;
};

struct WheelData : public Measurement {
    WheelData() { 
        type = MeasureType::kWheel;
    }
    ~WheelData() override = default;
    
    double left;
    double right;
};
using WheelDataConstPtr = std::shared_ptr<const WheelData>;
using WheelDataPtr = std::shared_ptr<WheelData>;

struct MonoImageData : public Measurement {
    MonoImageData() {
        type = MeasureType::kMonoImage;
    }
    virtual ~MonoImageData() override { }
    cv::Mat image;
};
using MonoImageDataConstPtr = std::shared_ptr<const MonoImageData>;
using MonoImageDataPtr = std::shared_ptr<MonoImageData>;

struct GpsData : public Measurement {
    GpsData() {
        type = MeasureType::kGNSS;
    }
    ~GpsData() override = default;

    Eigen::Vector3d lon_lat_hei;
    Eigen::Matrix3d cov;
};
using GpsDataConstPtr = std::shared_ptr<const GpsData>;
using GpsDataPtr = std::shared_ptr<GpsData>;
