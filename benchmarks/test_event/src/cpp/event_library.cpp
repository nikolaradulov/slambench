#include<SLAMBenchAPI.h>
#include<io/sensor/EventCameraSensor.h>
#include<io/sensor/CameraSensor.h>
#include<io/sensor/CameraSensorFinder.h>
#include<io/SLAMFrame.h>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<io/Event.h>
#include<iostream>
#include<fstream>
#include<thread>
#include<chrono>

std::ofstream outfile;

Eigen::Matrix4f ident_matrix = Eigen::Matrix4f::Identity();

static slambench::outputs::Output *pose_out;
static slambench::io::EventCameraSensor * event_sensor = nullptr;
static slambench::io::CameraSensor * grey_sensor = nullptr;
static slambench::TimeStamp last_frame_timestamp;
static slambench::outputs::Output *event_frame_output;
static slambench::outputs::Output *grey_frame_output;
static Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> image;

static slambench::TimeStamp latest_frame;
static std::vector<unsigned char> image_raw;
static std::vector<unsigned char> grey_raw;
static sb_uint2 eventInputSize;
static sb_uint2 greyInputSize;
static std::pair<int,int> *indices;

static int begin;
static int end;

static bool event=false;
static bool grey=false;
static int event_frames=0;

void select_frame(SLAMBenchLibraryHelper * lib){
    // get the mutex
    std::lock_guard<std::mutex> lock(lib->GetMutex());
    int cur_begin_idx = lib->GetBeginIndex();
    int cur_end_idx = lib->GetEndIndex();
    if(cur_end_idx==lib->events_->size()-1){
        std::cout<<lib->events_->size()<<'\n';
        lib->SetBeginIndex(-1000);
        // lib->begin_idx_= cur_end_idx+1;
        lib->SetEndIndex(-1000);
        return;
    }
    //select next frame based on time
    auto current_ts = (*(lib->events_))[cur_end_idx+1].ts;
    int i;
    for(i = cur_end_idx+1; i < lib->events_->size(); ++i) {
            auto delta = (*(lib->events_))[i].ts - current_ts;
            if (delta > std::chrono::milliseconds{20}) break;
    }

    lib->SetBeginIndex(cur_end_idx+1);
    // lib->begin_idx_= cur_end_idx+1;
    // 1000 events per frame
    lib->SetEndIndex(i-1);
    // std::cout<<"next is "<<cur_end_idx+1<<' '<<i-1<<"\n\n";
}

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings){
    slam_settings->event_ = true;
    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings){
    outfile.open("text_event_out.txt");

    // retrieve one event sensor
    event_sensor = (slambench::io::EventCameraSensor*)slam_settings->get_sensors().GetSensor(slambench::io::EventCameraSensor::kEventCameraType);
    assert(event_sensor!=nullptr && "Failed, did not find event sensor");
    
    slambench::io::CameraSensorFinder sensor_finder;
    grey_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "grey"}});
	assert(grey_sensor!=nullptr && "Failed, did not find event sensor");

    // initialise the pose output and bind it to the slam configuration output
    pose_out = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_out);

    event_frame_output = new slambench::outputs::Output("Event Frame", slambench::values::VT_FRAME);
  	event_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(event_frame_output);

    // to add output for the frame later
    grey_frame_output = new slambench::outputs::Output("Grey Frame", slambench::values::VT_FRAME);
    grey_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(grey_frame_output);
    
    

    
    eventInputSize   = make_sb_uint2(event_sensor->Width, event_sensor->Height);
    // Initialize Vertices

    greyInputSize   = make_sb_uint2(grey_sensor->Width, grey_sensor->Height);
    grey_raw.resize(grey_sensor->Height * grey_sensor->Width);
    // std::cout<<"------ width "<<event_sensor->Width<<"------ height"<<event_sensor->Height<<"------\n";
    //set the first frame
    select_frame(slam_settings);
    return true;
}

bool sb_update_frame(SLAMBenchLibraryHelper * lib, slambench::io::SLAMFrame* s){
    // check that the right sensor is associated with the frame
    // only do stuff is so 
    if(s->FrameSensor==event_sensor){
        // then print the data ? or file it
        // could/should get something to hold the data for further use after the frame is freed
        // instead for now we can just read straight from the data and see
        // maybe take note of the timestampt too
        last_frame_timestamp=s->Timestamp;
        //outfile<<"CURRENT FRAME STAMP: "<<last_frame_timestamp<<"\n";
        indices = static_cast<std::pair<int, int> *> (s->GetData());
        begin = indices->first;
        end = indices->second;
        event_frames++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        // free the data from the frame so you don;t run out of memory
        outfile<<"EXIT FRAME\n";
        select_frame(lib);
        // /exit(1);
        // std::cout<<"got frame with "<<begin<<' '<<end<<' '<<last_frame_timestamp<<'\n';
        event=true;
        // if(grey&&event){
        //     grey=false;
        //     event =false;
        //     return true;
        // }

        // s->FreeData();
        return true;
    }
   
    //std::cout<<"There are others\n\n\n";
    if(s->FrameSensor==grey_sensor){
        memcpy(grey_raw.data(), s->GetData(), s->GetSize());
        latest_frame = s->Timestamp;
        // s->FreeData();
        grey=true;
        // forces the algorithm to need one of each to work
        // if(grey&&event){
        //     grey=false;
        //     event =false;
        //     return true;
        // }
        return true;
            
    }
    // std::cout<<"Mistery frame received "<<s->FrameSensor->GetType()<<" \n";    
    return false;	
}

bool sb_process_once(SLAMBenchLibraryHelper * slam_settings){
    // nothing to do for a fake
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // std::cout<<"Done process once\n";
    //used both grey and event so we need new pair
    
    return true;
}

bool sb_clean_slam_system(){
    delete event_sensor;
    delete pose_out;
    delete event_frame_output;
    delete grey_sensor;
    std::cout<<"FINISHED: sb_clean_slam_system; There were "<<event_frames<<" event frames \n";
    outfile.close();
    return true;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *slam_settings, const slambench::TimeStamp *timestamp){
    // pass the identy matrix as pose , should be 0,0,0
    if(pose_out->IsActive()) {
        std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());
	    pose_out->AddPoint(*timestamp, new slambench::values::PoseValue(ident_matrix));
    }
    // std::cout<<"FINISHED: sb_update_outputs\n";
    // we need to output the events. Could do a matrix that stays constantly in memory and then we use frames
    
    // for (const auto& value : image_raw) {
    //     if (value !=0)
    //     	std::cout << static_cast<int>(value) << " ";
    // }
    // std::cout << std::endl<<std::endl<<std::endl<<std::endl;
        
    
   
    if(grey_frame_output->IsActive()) {
		std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());

		grey_frame_output->AddPoint(latest_frame, new slambench::values::FrameValue(greyInputSize.x, greyInputSize.y, slambench::io::pixelformat::G_I_8, (void*) &(grey_raw.at(0))));
	}

    // now we just replace the frame value here and everything should be dandy
    // if(event_frame_output->IsActive()) {
	// 	std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());
    //     // Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> rotated = (image.transpose().rowwise().reverse());
	// 	event_frame_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(eventInputSize.y, eventInputSize.x,  slambench::io::pixelformat::G_I_8, (void*) (image.data())));
	// }

    
    if(event_frame_output->IsActive()) {
		std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());
        // Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> rotated = (image.transpose().rowwise().reverse());
		// std::cout<<"Showing frame with indices "<<indices->first<<' '<<indices->second<<' '<<last_frame_timestamp<<'\n';
        event_frame_output->AddPoint(last_frame_timestamp, new slambench::values::EventFrameValue(eventInputSize.y, eventInputSize.x,  slam_settings->events_, begin, end));
	}

    // if that implementation works we then employ something simillar and creat a special output class for events, where we have the matrix
    // globally stored and then it's just updated as events come in 
    // std::cout<<"Done update outputs\n";
    return true;
}

bool sb_relocalize(){
    return true;
}