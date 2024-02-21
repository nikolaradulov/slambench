#include<SLAMBenchAPI.h>

#include<io/sensor/CameraSensor.h>
#include<io/sensor/CameraSensorFinder.h>
#include<io/SLAMFrame.h>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<iostream>
#include<fstream>
#include<thread>
#include<chrono>
#include<opencv2/opencv.hpp>

std::ofstream outfile;
bool im_file_initialized=false;
Eigen::Matrix4f ident_matrix = Eigen::Matrix4f::Identity();

static slambench::outputs::Output *pose_out;
static slambench::io::CameraSensor * grey_sensor = nullptr;
static slambench::TimeStamp last_frame_timestamp;
static slambench::outputs::Output *grey_frame_output;
static Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> image;

static slambench::io::CameraSensor *rgb_sensor;

static slambench::TimeStamp latest_frame;
static std::vector<unsigned char> image_raw;
static std::vector<unsigned char> grey_raw;

static sb_uint2 greyInputSize;
static std::pair<int,int> *indices;
static cv::Mat *imRGB = nullptr;
static int begin;
static int end;

std::string im_file_name;
std::string default_file ="";
static int event_frames=0;


void static im_compute_metrics(const cv::Mat image)
{   
    cv::Mat current_image;

    // Check if the image has more than one channel and convert to grayscale if so
    if (image.channels() > 1) {
        cv::cvtColor(image, current_image, cv::COLOR_BGR2GRAY);
    } else {
        current_image = image;
    }

    /* ---------- Compute Image Quality ------------ */
    cv::Mat laplacian, absLaplacian;
    imRGB = new cv::Mat (rgb_sensor->Height, rgb_sensor->Width,CV_8UC3);
    // Sharpness: Variance of Laplacian
    cv::Laplacian(current_image, laplacian, CV_64F);
    cv::convertScaleAbs(laplacian, absLaplacian);
    cv::Scalar mu, sigma;
    cv::meanStdDev(absLaplacian, mu, sigma);
    double sharpness = sigma.val[0] * sigma.val[0];

    // Brightness: Measure the brightness level
    double brightness = cv::mean(current_image)[0];

    // Contrast: Standard deviation of pixel intensities
    cv::meanStdDev(current_image, mu, sigma);
    double contrast = sigma.val[0];
    /* --------------------------------------------- */
    std::cout << sharpness << "  " << brightness << "  " << contrast << std::endl;

    std::ifstream ifile(im_file_name);
    bool file_exists = ifile.good();
    ifile.close();

    if (!file_exists || !im_file_initialized) {
        std::ofstream ofile(im_file_name, std::ofstream::out);
        im_file_initialized = true;
        ofile.close();
    }

    std::ofstream ofile(im_file_name, std::ofstream::out | std::ofstream::app);
    if (ofile.is_open()) {
        ofile << sharpness << " " << brightness << " " << contrast << std::endl;
        ofile.close();
    } else {
        std::cerr << "Failed to open the file for writing." << std::endl;
    }

}



bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings){
    slam_settings->addParameter(TypedParameter<std::string>("img", "img-metrics", "File for image metrics",&im_file_name, &default_file));
    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings){
    
    slambench::io::CameraSensorFinder sensor_finder;
    grey_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "grey"}});
	assert(grey_sensor!=nullptr && "Failed, did not find event sensor");
    rgb_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "rgb"}});
    

    // initialise the pose output and bind it to the slam configuration output
    pose_out = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_out);

    // to add output for the frame later
    grey_frame_output = new slambench::outputs::Output("Grey Frame", slambench::values::VT_FRAME);
    grey_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(grey_frame_output);

    greyInputSize   = make_sb_uint2(grey_sensor->Width, grey_sensor->Height);
    grey_raw.resize(grey_sensor->Height * grey_sensor->Width);
   
    return true;
}

bool sb_update_frame(SLAMBenchLibraryHelper * lib, slambench::io::SLAMFrame* s){
    // check that the right sensor is associated with the frame
    // only do stuff is so 
    if(s->FrameSensor==grey_sensor){
        memcpy(grey_raw.data(), s->GetData(), s->GetSize());
        latest_frame = s->Timestamp;
        cv::Mat image_grey = cv::Mat(grey_sensor->Height, grey_sensor->Width, CV_8UC1, grey_raw.data());
		im_compute_metrics(image_grey);
        s->FreeData();
        return false;
            
    }else if(s->FrameSensor == rgb_sensor and imRGB) {
        printf("Checkpoint 1.5\n");
        memcpy(imRGB->data, s->GetData(), s->GetSize());
        printf("memcpy good\n");
        cv::Mat image_grey = cv::Mat(rgb_sensor->Height, rgb_sensor->Width, CV_8UC3, imRGB->data);
        im_compute_metrics(image_grey);
        printf("quality good\n");
        last_frame_timestamp = s->Timestamp;
        s->FreeData();
        printf("checkpoint 2\n");
        return true;
    }
      
    return false;	
}

bool sb_process_once(SLAMBenchLibraryHelper * slam_settings){
    // nothing to do for a fake
    
    return true;
}

bool sb_clean_slam_system(){
   
    delete pose_out;
    delete grey_sensor;
    outfile.close();
    return true;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *slam_settings, const slambench::TimeStamp *timestamp){
    // pass the identy matrix as pose , should be 0,0,0
    if(pose_out->IsActive()) {
        std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());
	    pose_out->AddPoint(*timestamp, new slambench::values::PoseValue(ident_matrix));
    }   
   
    if(grey_frame_output->IsActive()) {
		std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());

		grey_frame_output->AddPoint(latest_frame, new slambench::values::FrameValue(greyInputSize.x, greyInputSize.y, slambench::io::pixelformat::G_I_8, (void*) &(grey_raw.at(0))));
	}

   
    return true;
}

bool sb_relocalize(){
    return true;
}