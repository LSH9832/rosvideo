#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "argparse/argparser.h"

using namespace std;

double fps=30;
int bitrate=2000;
bool compressed=true;
bool ros_source=false;
bool ros_address=false;
bool show=false;
bool pub_compressed=true;
string source;
string dist;
ros::Subscriber sub;
ros::Publisher pub;
cv::VideoWriter writer;
cv::VideoCapture cap;
cv::Mat receive_img;
cv::Mat img_to_process;
cv::Mat pub_img;


argsutil::argparser get_args(int argc, char* argv[], bool show=false) {
    auto args = argsutil::argparser("");
    args.add_option<string>("-c", "--cfg", "config file path", "./config/source2ros.yaml");
    args.add_option<bool>("-s", "--show", "show image with opencv", (bool)show);
    args.add_help_option();
    args.parse(argc, argv);
    return args;
}

bool endsWith(const std::string& str, const std::string& suffix) {  
    if (str.length() >= suffix.length())
        return (0 == str.compare(str.length() - suffix.length(), suffix.length(), suffix));  
    else
        return false;
} 


bool startsWith(const std::string& str, const std::string& prefix) {  
    if (str.length() >= prefix.length())
        return (0 == str.compare(0, prefix.length(), prefix));  
    else
        return false;
}


cv::VideoWriter getVideoWriter(
    cv::Size imgsz,
    string address="rtmp://127.0.0.1/live/test", 
    double fps=30., 
    int bitrate=500
) {
    if (startsWith(address, "rtmp://") || startsWith(address, "rtsp://")) {
        string stream_type = startsWith(address, "rtmp://")?"rtmpsink":"rtspsink";
        string dist = "appsrc ! videoconvert ! x264enc speed-preset=ultrafast bitrate=" + 
                    to_string(bitrate) + string(" tune=zerolatency ! flvmux ! ") + 
                    stream_type + "location=" + address;
        
        cv::VideoWriter w(
            dist,
            cv::CAP_GSTREAMER,
            fps,
            imgsz
        );
        return w;
    } else
        return cv::VideoWriter(address, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, imgsz);
}


void process_image() {
    img_to_process = receive_img.clone();
    /**
     * add your code here
     */
    pub_img = img_to_process;
}


int show_img() {
    int key = -1;
    if (show) {
        cv::imshow(dist, pub_img);
        key = cv::waitKey(1);
    }
    return key;
}

void rosCompressed2pub(const sensor_msgs::CompressedImage::ConstPtr& data) {
    // cv_bridge::CvImage bridge(std_msgs::Header(), "bgr8", image);
    receive_img = cv::imdecode(data->data, cv::IMREAD_COLOR);
    if (!ros_address && !writer.isOpened()) writer = getVideoWriter(receive_img.size(), dist, fps, bitrate);
    process_image();
    
    if (ros_address) {
        if (pub_compressed) pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toCompressedImageMsg());
        else pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg());
    }
    else writer.write(pub_img);

    if (show_img()==27) exit(0);
}

void rosImage2pub(const sensor_msgs::Image::ConstPtr& data) {
    receive_img = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8)->image;
    if (!ros_address && !writer.isOpened()) writer = getVideoWriter(receive_img.size(), dist, fps, bitrate);
    process_image();

    if (ros_address) {
        if (pub_compressed) pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toCompressedImageMsg());
        else pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg());
    }
    else writer.write(pub_img);

    if (show_img()==27) exit(0);
}


void ros2rtmp(int argc, char* argv[]) {
    ros::init(argc, argv, "ros2rtmp");
    ros::NodeHandle nd;
    if (compressed)
        sub = nd.subscribe(source, 10, rosCompressed2pub);
    else
        sub = nd.subscribe(source, 10, rosImage2pub);
    ros::spin();
}

void ros2ros(int argc, char* argv[]) {
    ros::init(argc, argv, "ros2ros");
    ros::NodeHandle nd;
    if (compressed) {
        pub = nd.advertise<sensor_msgs::CompressedImage>(dist, 10);
        sub = nd.subscribe(source, 10, rosCompressed2pub);
    }
    else {
        pub = nd.advertise<sensor_msgs::Image>(dist, 10);
        sub = nd.subscribe(source, 10, rosImage2pub);
    }   
    ros::spin();
}

void stream2rtmp() {
    cap.open(source);
    cv::Size cap_size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    writer = getVideoWriter(cap_size, dist, fps, bitrate);

    while (cap.isOpened()) {
        cap.read(receive_img);
        if (receive_img.empty()) {
            cap.release();
            cap.open(source);
            continue;
        }

        process_image();

        writer.write(pub_img);
        if (show_img() == 27) break;
    }

    writer.release();
    cap.release();
    cv::destroyAllWindows();
}


void stream2ros(int argc, char* argv[]) {
    ros::init(argc, argv, "rtmp2ros");
    ros::NodeHandle nd;
    if (compressed) 
        pub = nd.advertise<sensor_msgs::CompressedImage>(dist, 10);
    else 
        pub = nd.advertise<sensor_msgs::Image>(dist, 10);
    ros::Rate rate(fps);  // 10HZ
    cap.open(source);

    while (cap.isOpened()) {
        cap.read(receive_img);
        if (receive_img.empty()) {
            cap.release();
            cap.open(source);
            continue;
        }

        process_image();

        if (pub_compressed) pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toCompressedImageMsg());
        else pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg());
        rate.sleep();
        if (show_img() == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();



}

int main(int argc, char* argv[]) {

    auto args = get_args(argc, argv);
    YAML::Node cfg = YAML::LoadFile(args.get_option_string("--cfg"));
    args = get_args(argc, argv, cfg["show"].as<bool>());

    show = args.get_option_bool("--show");
    ros_source = cfg["ros_source"].as<bool>();
    compressed = cfg["compressed"].as<bool>();
    source = cfg["source"].as<string>();
    dist = cfg["address"].as<string>();     // ros topic name or rtmp address
    ros_address = cfg["ros_address"].as<bool>();
    fps = cfg["fps"].as<double>();
    bitrate = cfg["bitrate"].as<int>();

    cout << "start pushing frames from " << source << " to " << dist << endl;
    
    if (!ros_address) {
        cout << "Bitrate: " << bitrate << "kbps" << endl;
        cout << "FPS: " << fps << endl;
    }

    if (ros_source) {
        if (ros_address) ros2ros(argc, argv);
        else ros2rtmp(argc, argv);
    }
    else {
        if (ros_address) stream2ros(argc, argv);
        else stream2rtmp();
    }

    return 0;
}
