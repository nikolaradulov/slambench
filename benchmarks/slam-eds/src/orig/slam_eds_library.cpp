#include<SLAMBenchAPI.h>
#include<io/sensor/EventCameraSensor.h>
#include<io/sensor/CameraSensor.h>
#include<io/sensor/CameraSensorFinder.h>
#include<io/SLAMFrame.h>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<io/Event.h>
#include<iostream>
/** EDS Library **/
#include <eds/EDS.h>
#include <memory>

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

static int begin;
static int end;

/** Configuration **/
EDSConfiguration eds_config;
/** Indexes **/
uint64_t ef_idx, frame_idx;
int64_t kf_idx;

/** Initialized flag **/
bool initialized, first_track;

/** Is Lost flag **/
bool is_lost;

float rescale_factor;
std::vector<float> all_scale;

/** Image frame in opencv format **/
cv::Mat img_frame;

/** Image frame in opencv format splitted in channels **/
cv::Mat img_rgb[3];

/** Buffer of events **/
std::vector<::base::samples::Event> events;

/** Local Depth map **/
std::shared_ptr<::eds::mapping::IDepthMap2d> depthmap;

/** All Frames and Keyframes History **/
std::vector<dso::FrameShell*> all_frame_history;
std::vector<dso::FrameShell*> all_keyframes_history;

/** Keyframes window **/
std::vector<dso::FrameHessian*> frame_hessians;

/** DSO Calibration **/
std::shared_ptr<dso::CalibHessian> calib;

/** DSO Undistort parameters **/
std::shared_ptr<::dso::Undistort> undistort;

/**  Initializer **/
std::shared_ptr<::dso::CoarseInitializer> initializer;

/** Event-to-Image tracker (EDS)**/
std::shared_ptr<::eds::tracking::Tracker> event_tracker;
std::shared_ptr<::eds::tracking::KeyFrame> key_frame;
std::shared_ptr<::eds::tracking::EventFrame> event_frame;

/** Image-to-Image Tracker (DSO) **/
std::shared_ptr<::dso::CoarseTracker> image_tracker;
dso::Vec5 last_coarse_RMSE;

/** Mapping (Point selection strategy)**/
float* selection_map;
std::shared_ptr<::dso::PixelSelector> pixel_selector;
std::shared_ptr<::dso::CoarseDistanceMap> coarse_distance_map;

// ::dso ~ eds::dso
/** Bundle Adjustment using DSO energy functional **/
float currentMinActDist;
std::vector<dso::PointFrameResidual*> active_residuals;
::dso::IndexThreadReduce<dso::Vec10> thread_reduce;
std::shared_ptr<::dso::EnergyFunctional> bundles;
std::vector<float> all_res_vec;

/** KeyFrame camera pose w.r.t World **/
base::samples::RigidBodyState pose_w_kf;

/** EventFrame camera pose w.r.t Keyframe **/
base::samples::RigidBodyState pose_kf_ef;

/** EventFrame camera pose w.r.t World **/
base::samples::RigidBodyState pose_w_ef;

/** Globa Map **/
std::map<int, base::samples::Pointcloud> global_map;

static std::vector<base::samples::event> ef_events;
//=================== [HELPER FUNCTIONS] ===================

dso::SE3 initialize(dso::ImageAndExposure* image, int id, const int &snapped_threshold)
{
    // =========================== add into allFrameHistory =========================
    dso::FrameHessian* fh = new dso::FrameHessian();
    dso::FrameShell* shell = new dso::FrameShell();
    shell->camToWorld = dso::SE3();
    shell->aff_g2l = dso::AffLight(0,0);
    shell->marginalizedAt = shell->id = all_frame_history.size();
    shell->timestamp = image->timestamp;
    shell->incoming_id = id;
    fh->shell = shell;
    all_frame_history.push_back(shell);

    // =========================== make Images / derivatives etc. =========================
    fh->ab_exposure = image->exposure_time;
    fh->makeImages(image->image, calib.get());

    /** Output image while trying to initialize **/
    outputImmaturePtsFrameViz(fh, ::base::Time::fromSeconds(fh->shell->timestamp));

    if(initializer->frameID<0) // first frame set. fh is kept by coarseInitializer.
    {
        initializer->setFirst(calib.get(), fh);
    }
    else if(initializer->trackFrame(fh, snapped_threshold))	// if SNAPPED
    {
        /** Initialize: this set the initialized flag to true and push the first KF **/
        {
            std::cout<<"[EDS_TASK] START INITIALIZE FROM DSO INITIALIZER"<<std::endl;

            /** add first keyframe: this is what EDS calls add frame to Hessian struct **/
            dso::FrameHessian* firstFrame = initializer->firstFrame;
            firstFrame->idx = frame_hessians.size();
            frame_hessians.push_back(firstFrame);
            firstFrame->frameID = all_keyframes_history.size();
            all_keyframes_history.push_back(firstFrame->shell);

            /** this is needed for bundles adjustment (energy functional) **/
            bundles->insertFrame(firstFrame, calib.get());
            setPrecalcValues();

            /** Reserve memory for points in first keyframe **/
            firstFrame->pointHessians.reserve(dso::wG[0]*dso::hG[0]*0.2f); //max 20% of all pixles
            firstFrame->pointHessiansMarginalized.reserve(dso::wG[0]*dso::hG[0]*0.2f); //max 20% of all pixels
            firstFrame->pointHessiansOut.reserve(dso::wG[0]*dso::hG[0]*0.2f);// max 20% of all pixels

            float sumID=1e-5, numID=1e-5;
            for(int i=0;i<initializer->numPoints[0];i++)
            {
                sumID += initializer->points[0][i].iR;
                numID++;
            }
            float rescaleFactor = 1 / (sumID / numID);

            /** randomly sub-select the points I need **/
            float keepPercentage = dso::setting_desiredPointDensity / initializer->numPoints[0];

            if(!dso::setting_debugout_runquiet)
                printf("[EDS_TASK] Initialization: keep %.1f%% (need %d, have %d)!\n", 100*keepPercentage,
                        (int)(dso::setting_desiredPointDensity), initializer->numPoints[0] );

            for(int i=0;i<initializer->numPoints[0];i++)
            {
                if(rand()/(float)RAND_MAX > keepPercentage) continue;

                dso::Pnt* point = initializer->points[0]+i;
                dso::ImmaturePoint* pt = new dso::ImmaturePoint(point->u+0.5f,point->v+0.5f,
                                            firstFrame,point->my_type, calib.get(),
                                            &(img_rgb[0].at<unsigned char>(0)),
                                            &(img_rgb[1].at<unsigned char>(0)),
                                            &(img_rgb[2].at<unsigned char>(0))
                                            );

                if(!std::isfinite(pt->energyTH)) { delete pt; continue; }

                pt->idepth_max=pt->idepth_min=1;
                /** INIT  is where  the PointsHessian memory is reserved **/
                dso::PointHessian* ph = new dso::PointHessian(pt, calib.get());
                delete pt;
                if(!std::isfinite(ph->energyTH)) {delete ph; continue;}

                ph->setIdepthScaled(point->iR*rescaleFactor);
                ph->setIdepthZero(ph->idepth);
                ph->hasDepthPrior=true;
                ph->setPointStatus(dso::PointHessian::ACTIVE);

                firstFrame->pointHessians.push_back(ph);

                /** Insert the Points in bundles energy functional **/
                bundles->insertPoint(ph);
            }

            dso::SE3 firstToNew = initializer->thisToNext;
            firstToNew.translation() /= rescaleFactor;

            firstFrame->shell->camToWorld = SE3();
            firstFrame->shell->aff_g2l = dso::AffLight(0,0);
            firstFrame->setEvalPT_scaled(firstFrame->shell->camToWorld.inverse(),firstFrame->shell->aff_g2l);
            firstFrame->shell->trackingRef=0;
            firstFrame->shell->camToTrackingRef = SE3();

            fh->shell->camToWorld = firstToNew.inverse();
            fh->shell->aff_g2l = dso::AffLight(0,0);
            fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(), fh->shell->aff_g2l);
            fh->shell->trackingRef = firstFrame->shell;
            fh->shell->camToTrackingRef = firstToNew.inverse();

            initialized=true;

            /** Optimize the first two DSO Keyframes and set the first EDS Keyframe**/
            makeKeyFrame(fh);

            /** Scale factor **/
            rescale_factor = rescaleFactor;

            std::cout<<"** [EDS_TASK] INITIALIZE FROM INITIALIZER ("<< (int)firstFrame->pointHessians.size() <<" pts) "<< rescaleFactor <<" rescale_factor!"<<std::endl;
            std::cout<<"** [EDS_TASK] INITIALIZATION FINISHED WITH T_w_kf:\n"<< pose_w_kf.getTransform().matrix()<<std::endl;
        }
    }
    else
    {
        /** if still initializing **/
        fh->shell->poseValid = false;
        delete fh;
    }
    if (initialized) return fh->get_worldToCam_evalPT().inverse(); //return T_w_kf
    else return dso::SE3();
}

// handles the event-based odometry pipeline
bool Task::eventsToImageAlignment(const std::vector<::base::samples::Event> &events_array, ::base::Transform3d &T_kf_ef)
{
    /** Keyframe to Eventframe delta pose **/
    T_kf_ef = ::base::Transform3d::Identity();

    /** Execute the tracker and get the T_kf_ef **/
    state(OPTIMIZING);
    bool success = false;
    for (int i=event_frame->event_frame.size()-1; i>=0; --i)
    {
        success = event_tracker->optimize(i, &(event_frame->event_frame[i]), T_kf_ef, ::eds::tracking::MAD);
    }

    /** Track the points and remove the ones out of the image plane **/
    std::vector<cv::Point2d> coord = event_tracker->getCoord(true);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EDS_TASK EVENT_TO_IMG ALIGNMENT] "<<(success?"SUCCESS": "NO_USABLE") <<" rescale_factor: "<< this->rescale_factor<<"\nT_kf_ef:\n"<<T_kf_ef.matrix()<<std::endl;
    #endif
    state(RUNNING);

    return success;
}

// method for loading the algorithm configuration from the YAML file
::eds::DataLoaderConfig Task::readDataLoaderConfig(YAML::Node config)
{
    ::eds::DataLoaderConfig dt_config;

    /** Read the number of events to read **/
    dt_config.num_events = config["num_events"].as<size_t>();
    dt_config.overlap = config["overlap"].as<double>(); //in percentage
    if (dt_config.overlap < 0.0) dt_config.overlap = 0.0;
    dt_config.overlap /= 100.0;

    return dt_config;
}
//================= [SLAMBENCH INTEGRATION] ================

bool sd_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings){
    slam_settings->event = true;
    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings){

     // --------- [INIT FOR EDS] ----------
    
    /** Reset counters **/
    ef_idx = frame_idx = kf_idx = 0;

    /** DSO intitialization flags **/
    initialized = false;
    first_track = true;

    /** Is Lost is false **/
    is_lost = false;

    /** Initialize T_w_kf 
        key frame camera pose with respect to world

    **/
    pose_w_kf.targetFrame = "world";
    pose_w_kf.sourceFrame = "kf";
    pose_w_kf.setTransform(::base::Transform3d::Identity());

    /** Initialize T_kf_ef 
        event camera pose with respect to keyframe
    **/
    pose_kf_ef.targetFrame = "kf";
    pose_kf_ef.sourceFrame = "ef";
    pose_kf_ef.setTransform(::base::Transform3d::Identity());

    /** Initialize T_w_ef 
        event camera pose with respect to world
    **/
    pose_w_ef.targetFrame = "world";
    pose_w_ef.sourceFrame = "ef";
    pose_w_ef.setTransform(::base::Transform3d::Identity());

    /** Initializer constructor (DSO) **/
    initializer = std::make_shared<dso::CoarseInitializer>(dso::wG[0], dso::hG[0]);

    /* Event-based Tracker (EDS) **/
    event_tracker = std::make_shared<::eds::tracking::Tracker>(eds_config.tracker);

    /** KeyFrame (EDS) **/
    key_frame = std::make_shared<eds::tracking::KeyFrame>(*(cam0), *(newcam), cam_calib.cam0.distortion_model);

    /** EventFrame (EDS) **/
    event_frame = std::make_shared<eds::tracking::EventFrame>(*(cam1), *(newcam), cam_calib.cam1.distortion_model);

    /** Image-based Tracker constructor (DSO) **/
    image_tracker = std::make_shared<dso::CoarseTracker>(dso::wG[0], dso::hG[0]);
    last_coarse_RMSE.setConstant(100);

    /** Mapping 
        output port see eds.orogen
        needed?????????
    **/
    selection_map = new float[dso::wG[0]*dso::hG[0]];
    pixel_selector = std::make_shared<dso::PixelSelector>(dso::wG[0], dso::hG[0]);
    coarse_distance_map = std::make_shared<dso::CoarseDistanceMap>(dso::wG[0], dso::hG[0]);
    depthmap = std::make_shared<eds::mapping::IDepthMap2d>(cam_calib.cam0.intrinsics);//Depthmap is in the original RGB frame

    //=================== [SLAMBench setting] ==================

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
    
    

    image.resize(event_sensor->Height, event_sensor->Width);

    eventInputSize   = make_sb_uint2(event_sensor->Width, event_sensor->Height);
    image_raw.resize(event_sensor->Height*event_sensor->Width);
    // Initialize Vertices

    greyInputSize   = make_sb_uint2(grey_sensor->Width, grey_sensor->Height);
    grey_raw.resize(grey_sensor->Height * grey_sensor->Width);
    
    // eds cannot survive without cameras. So we make cam0 and cam 1
    // cam 0 = the rgb/ grey camera

    // ============================ [From configHook()] ==============================
    // read the foncig file for the algorithm 
    // perhaps i can remove those and have them as runtime? 
    // for now everything is set in the config.yaml file
    YAML::Node config = YAML::LoadFile("./config.yaml");
    eds_config.data_loader = readDataLoaderConfig(config["data_loader"]);
    this->eds_config.tracker = ::eds::tracking::readTrackingConfig(config["tracker"]);
    this->eds_config.mapping = ::eds::mapping::readMappingConfig(config["mapping"]);
    this->eds_config.bundles = ::eds::bundles::readBundlesConfig(config["bundles"]);

    
    
    eds::calib::CameraInfo cam0_info;
    cam0_info.height = grey_sensor->Height;
    cam0_info.width = grey_sensor->Width;
    for (i =0; i<3;i++)
        cam0_info.intrinsics.push_back(grey_sensor->Intrinsic[i]);
    switch(grey_sensor->DistortionType){
        case Equidistant:
            cam0_info.distortion_model = "equidistant";
            break;
        default:
            cam0_info.distortion_model = "notequidistant";

    }
    for (i =0; i<3;i++)
        cam0_info.D.push_back(grey_sensor->Distortion[i]);
    // never comes from beamsplitter
    cam0_info.flip=false;
    eds::calib::Camera cam0(cam0_info);
    
    eds::calib::CameraInfo cam1_info;
    cam1_info.height = event_sensor->Height;
    cam1_info.width = event_sensor->Width;
    for (i =0; i<3;i++)
        cam1_info.intrinsics.push_back(grey_sensor->Intrinsic[i]);
    switch(grey_sensor->DistortionType){
        case Equidistant:
            cam1_info.distortion_model = "equidistant";
            break;
        default:
            cam1_info.distortion_model = "notequidistant";

    }
    for (i =0; i<3;i++)
        cam1_info.D.push_back(grey_sensor->Distortion[i]);
    // should have rotation provided but not available for us
    eds::calib::Camera cam1(cam1_info);

    // from task convigure hook
    cv::Size out_size{this->cam_calib.cam0.out_width, this->cam_calib.cam0.out_height};
    newcam = std::make_shared<eds::calib::Camera>(::eds::calib::setNewCamera(*(this->cam0), *(this->cam1), out_size));

     /* DSO Calibration
        -> paths differe and sensor callibrations will most likely have to be handled by SLAMBench
    **/
    newcam->toDSOFormat();
    dso::Undistort *undis = ::dso::Undistort::getUndistorterForFile("/tmp/dso_camera.txt"/* hardcode path in toDSOFormat */, "", "");
    this->undistort.reset(undis);
    Eigen::Matrix3f K_ref = this->undistort->getK().cast<float>();
    int w_out = this->undistort->getSize()[0];
    int h_out = this->undistort->getSize()[1];
    dso::setGlobalCalib(w_out, h_out, K_ref);

    /** Set the hessian calib (this needs to be after setGlobalcalib) 
        SLAMBench calibration again
    **/
    this->calib = std::make_shared<dso::CalibHessian>();
    std::cout<<"** [EDS CONFIG] fxl: "<<this->calib->fxl()<<" fxli: "<<this->calib->fxli()<<std::endl;
    std::cout<<"** [EDS CONFIG] fyl: "<<this->calib->fyl()<<" fyli: "<<this->calib->fyli()<<std::endl;
    std::cout<<"** [EDS CONFIG] cxl: "<<this->calib->cxl()<<" cxli: "<<this->calib->cxli()<<std::endl;
    std::cout<<"** [EDS CONFIG] cyl: "<<this->calib->cyl()<<" cyli: "<<this->calib->cyli()<<std::endl;

    /** DSO Settings **/
    dso::setting_desiredPointDensity = this->eds_config.mapping.num_desired_points;
    dso::setting_desiredImmatureDensity = (this->eds_config.tracker.percent_points/100.0) * (this->newcam->out_size.height *this->newcam->out_size.width);
    dso::setting_minFrames = this->eds_config.bundles.window_size;
    dso::setting_maxFrames = this->eds_config.bundles.window_size;
    dso::setting_maxOptIterations=6;
    dso::setting_minOptIterations=1;
    dso::setting_logStuff = false;
    std::cout<<"** [EDS CONFIG] global map num points: "<<dso::setting_desiredPointDensity
            <<" local map num points "<<dso::setting_desiredImmatureDensity<<std::endl;
    std::cout<<"** [EDS CONFIG] Keyframes sliding window size: "<<dso::setting_maxFrames<<std::endl;
    // std::cout<<"** [EDS CONFIG] Point Relative Baseline: "<<this->eds_config.mapping.points_rel_baseline<<std::endl;



    return true;
}

bool sb_update_frame(SLAMBenchLibraryHelper * lib, slambench::io::SLAMFrame* s){
    // we split to always have anough events whenever this gets called with an event frame
    if(s->FrameSensor==event_sensor){
        // get the indices for the  frame and timestamp
        std::pair<int,int> *indices = static_cast<std::pair<int, int> *> (s->GetData());
        begin = indices->first;
        end = indices->second;
        last_frame_timestamp=s->Timestamp;
        /** Move data (no copy) **/
        // originally everything is rocke events/ Chage for slambench events
        // the names are the same, it might just work 
        // but will make my life harder  
        // probably uld be easiest to just translate to the ther event type when i copy it 
        
        // might not work due to the way events behave has to be seen
        std::move(lib->events_->begin()+indices->first, lib->events_->begin()+indices->second, std::back_inserter(ef_events)); 
        for(i = indices->first , i<=indices->second; i++ ){
            slambench::io::Event event = (*(lib->events_))[i]; 
            ef_events.push_back(base::samples::event(event.x, event.y, base::time::fromMicroseconds(event.ts.ToMs()), event.polarity));
        }
        /** Create the Event Frame **/
        event_frame->create(ef_idx, ef_events, cam_calib.cam1,
                        eds_config.tracker.options.max_num_iterations.size(),
                        base::Affine3d::Identity(), newcam->out_size);

        /** Increment EF INDEX **/
        ef_idx++;
        s->FreeData();
        // next in the original task.cpp file is the EDS Tracker OPTIMISATION, That is the part that actually computes the 
        // pose
        return true;
    }
    else {
        // handle grey frames
        if(s->FrameSensor == grey_sensor){
            // this should get freed in the main loop
            // skip the undistort part, should be done by slaambench
            dso::ImageAndExposure* img(greyInputSize.x, greyInputSize.y, s->TimeStamp->ToS());
            memcpy(img->image, s->GetData(), s->GetSize());
            // the algorithm should get initialised on the first frame it receives
            if(initialzed == false){
                /** Track the current image frame and later decide whether it is a Keyframe **/
                if (!_frame_interrupt.value())
                    track(img, frame_idx);
                if (kf_idx < frame_hessians[frame_hessians.size()-1]->frameID){ 
                        /** Update the index **/
                        kf_idx =  frame_hessians[frame_hessians.size()-1]->frameID;
                }
            }else{
                // the algorithm is initialised on the first grey frame
                initialize(img, frame_idx, 10);
                dso::FrameHessian *first_frame = initializer->firstFrame;
                dso::FrameHessian *new_frame =  initializer->newFrame;
                if (first_frame)
                    std::cout<<"[EDS_TASK] INIT FIRST FRAME ID: "<<first_frame->shell->incoming_id;
                if (new_frame)
                    std::cout<<" NEW FRAME ID: "<<new_frame->shell->incoming_id;
                std::cout<<"\n";
                std::cout<<"[EDS_TASK] INIT T_INIT:\n"<<initializer->thisToNext.matrix3x4()<<std::endl;
                // std::cout<<"[EDS_TASK] INIT FRAME ID: "<<all_frame_history[frame_idx]->id <<"\n T_world_cam:\n"
                // <<all_frame_history[frame_idx]->camToWorld.matrix3x4()<<std::endl;
            }
        }
    }
}

bool sb_process_once(SLAMBenchLibraryHelper * slam_settings){
    /** EDS TRACKER OPTIMIZATION **/
    if (initialized)
    {
        /** Event to Image alignment T_kf_ef delta pose **/
        ::base::Transform3d T_kf_ef = pose_kf_ef.getTransform(); // initialize to current estimate
        // this is the actual tracker that handles the poses
        eventsToImageAlignment(ef_events, T_kf_ef); // EDS tracker estimate

        /** Set the EventFrame pose: T_w_ef with the result from alignment
            maybe thr event frame pose shall be enough    
        **/
        event_frame->setPose(pose_w_kf.getTransform()*T_kf_ef); // T_w_ef = T_w_kf * T_kf_ef
        // remove all the elements after using them
        ef_events.clear();
        // not exactly sure what is going on here
        /** Update the output port: T_kf_ef **/
        pose_kf_ef.time = event_frame->time;
        pose_kf_ef.setTransform(T_kf_ef);
        #ifdef DEBUG_PRINTS
        std::cout<<"** [EDS_TASK EVENTS] Wrote pose_kf_ef:\n"<<pose_kf_ef.getTransform().matrix()<<std::endl;
        #endif

        /**Next bit is meaant to write to the output port. we don't hve one so it eeds to be changed **/
        pose_w_ef.time = event_frame->time;
        pose_w_ef.setTransform(event_frame->getPose());
        pose_w_ef.velocity = pose_w_ef.getTransform() * event_tracker->linearVelocity();
        pose_w_ef.angular_velocity = pose_w_ef.getTransform() * event_tracker->angularVelocity();
        _pose_w_ef.write(pose_w_ef);
        #ifdef DEBUG_PRINTS
        std::cout<<"** [EDS_TASK EVENTS] Wrote pose_w_ef:\n"<<pose_w_ef.getTransform().matrix()<<std::endl;
        #endif
    }
    
    return true;
}

sb_update_outputs(SLAMBenchLibraryHelper * slam_settings, const slambench::TimeStamp *timestamp){
    // output the pose computed before 
    if(pose_out->IsActive()){
        std::lock_guard<FastLock> lock (slam_settings->GetOutputManager().GetLock());
        // need to figure how the psoe is represented in eds to know how to send it to slam
        // look at the output functions from Task
        // pose is Affine3d  == Transform3d type object
        // event frames have getposeMatrix function
        // i have a feeling this will not exactly work
        pose_out->AddPoint(last_frame_timestamp, new slambench::values::PoseValue(event_frame->getPoseMatrix));
    }


    if(event_frame_output->IsActive()){
        event_frame_output->AddPoint(last_frame_timestamp, new slambench::values::EventFrameValue(eventInputSize.y, eventInputSize.x,  slam_settings->events_, indices->first, indices->second));
    }
}

sb_clean_slam_system(){
    // there is a shitton of stuff to remove
    initializer.reset();
    event_tracker.reset();
    event_frame.reset();
    key_frame.reset();
    image_tracker.reset();
    delete[] selection_map;
    pixel_selector.reset();
    coarse_distance_map.reset();
    depthmap.reset();
    // bundles.reset();
    /** delete frames **/
    for (auto it : this->all_frame_history)
        if (it){delete it; it=nullptr;}

    events.clear();
    all_frame_history.clear();
    all_keyframes_history.clear();
    frame_hessians.clear();
    active_residuals.clear();
    calib.reset();
    undistort.reset();
}   