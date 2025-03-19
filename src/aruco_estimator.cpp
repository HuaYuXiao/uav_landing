#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <aruco/fractaldetector.h>
#include <aruco/cvdrawingutils.h>

class ArucoEstimator {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cam_pos_pub_;

    aruco::FractalDetector fractal_detector_;
    aruco::CameraParameters cam_params_;
    std::string camera_topic_prefix, camera_parameters_file;
    double marker_size_;
    bool ena_disturbances_;
    double brightness_;
    double haze_coeff_;
    int rain_value_;

    void loadCameraParams();

public:
    ArucoEstimator();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void start() { ros::spin(); }
};

ArucoEstimator::ArucoEstimator() : it_(nh_) {
    // Load parameters
    nh_.param<std::string>("camera_topic_prefix", camera_topic_prefix, "/monocular/");
    nh_.param<std::string>("camera_parameters_file", camera_parameters_file, "calib.yaml");
    nh_.param<double>("marker_size", marker_size_, 1);
    nh_.param<bool>("ena_disturbances", ena_disturbances_, true);
    nh_.param<double>("brightness", brightness_, 1);
    nh_.param<double>("haze_coeff", haze_coeff_, 0);
    nh_.param<int>("rain_value", rain_value_, 0);

    // Initialize detector
    loadCameraParams();
    fractal_detector_.setConfiguration(aruco::FractalMarkerSet::FRACTAL_5L_6);
    fractal_detector_.setParams(cam_params_, marker_size_);

    // Setup ROS communications
    image_sub_ = it_.subscribe(camera_topic_prefix + "image_raw", 1, 
                             &ArucoEstimator::imageCallback, this);
    image_pub_ = it_.advertise(camera_topic_prefix + "image_info", 1);
    cam_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/positioning/aruco/", 10);

    ROS_INFO("Aruco Estimator initialized");
}

void ArucoEstimator::loadCameraParams() {
    std::string config_path = ros::package::getPath("uav_landing") + 
                            "/config/camera_calibration/" + camera_parameters_file;
    std::cout << config_path << std::endl;
    cam_params_.readFromXMLFile(config_path);
    if (!cam_params_.isValid()) {
        ROS_ERROR("Invalid camera parameters!");
        throw std::runtime_error("Invalid camera parameters");
    }
}

void ArucoEstimator::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;

    // Process frame
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (fractal_detector_.detect(gray)) {
        ROS_INFO_ONCE("Marker detected");
        fractal_detector_.drawMarkers(frame);

        if (fractal_detector_.poseEstimation()) {
            cv::Mat tvec = fractal_detector_.getTvec();
            cv::Mat rvec = fractal_detector_.getRvec();
            fractal_detector_.draw3d(frame);

            // Convert rotation vector to matrix
            cv::Mat rot_mat;
            cv::Rodrigues(rvec, rot_mat);

            // Convert to quaternion
            tf2::Matrix3x3 tf_rot(
                rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
            );
            tf2::Quaternion tf_quat;
            tf_rot.getRotation(tf_quat);

            // Create and publish pose message
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "monocular_link";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = tvec.at<double>(0);
            pose.pose.position.y = -tvec.at<double>(1);
            pose.pose.position.z = tvec.at<double>(2);
            pose.pose.orientation.x = tf_quat.x();
            pose.pose.orientation.y = tf_quat.y();
            pose.pose.orientation.z = tf_quat.z();
            pose.pose.orientation.w = tf_quat.w();
            cam_pos_pub_.publish(pose);

            // Draw position text
            std::string pos_text = cv::format("X: %.4f Y: %.4f Z: %.4f", 
                                            tvec.at<double>(0), 
                                            -tvec.at<double>(1), 
                                            tvec.at<double>(2));
            cv::putText(frame, pos_text, cv::Point(20, 90), 
                       cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 3);
            cv::putText(frame, pos_text, cv::Point(20, 90), 
                       cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1);
        }
    }
    else {
        cv::putText(frame, "NOT FOUND", cv::Point(20, 30), 
                   cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 3);
        cv::putText(frame, "NOT FOUND", cv::Point(20, 30), 
                   cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1);
        ROS_WARN_ONCE("Marker not detected");
    }

    // Publish processed image
    image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_estimator");
    ArucoEstimator estimator;
    estimator.start();
    return 0;
}
