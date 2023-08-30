/*
 * This file is part of the EDS: Event-aided Direct Sparse Odometry
 * (https://rpg.ifi.uzh.ch/eds.html)
 *
 * Copyright (c) 2022 Javier Hidalgo-Carrio, Robotics and Perception
 * Group (RPG) University of Zurich.
 *
 * EDS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * EDS is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

/** Frame Helper for image conversion **/
#include <frame_helper/FrameHelper.h>

#include "Task.hpp"

using namespace eds;


#define DEBUG_PRINTS 1

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}


/**
    Gets called whenever events are received on the stream

*/
void Task::eventsCallback(const base::Time &ts, const ::base::samples::EventArray &events_sample)
{
    //#ifdef DEBUG_PRINTS
    //std::cout<<"** [EDS_TASK EVENTS] Received "<<events_sample.events.size()<<" events at["<<events_sample.time.toSeconds()<<"] **\n";
    //#endif

    /** Insert Events into the buffer **/
    this->events.insert(this->events.end(), events_sample.events.begin(), events_sample.events.end());

    /** Create an event frame **/
    //check we have enough events inside the buffer
    if (this->events.size() >= this->eds_config.data_loader.num_events)
    {

        #ifdef DEBUG_PRINTS
        ::base::Time first_ts = this->events[0].ts;
        ::base::Time last_ts = this->events[this->eds_config.data_loader.num_events-1].ts;
        std::cout<<"** [EDS_TASK EVENTS] Processing events from["<<first_ts.toSeconds()<<"] to["<<last_ts.toSeconds()<<"] size:"<<this->events.size()<<std::endl;
        #endif

        /** Move data (no copy) **/
        std::vector<::base::samples::Event> ef_events;
        std::move(this->events.begin(), this->events.begin()+this->eds_config.data_loader.num_events, std::back_inserter(ef_events)); 
    
        /** Clean events in the original buffer depending in the overlap percentage.
         there is overlp percentage between frames, so only those outside the 
         overlap are removed from buffer
         * std move only moves (it does not delete) **/
        // to be done in frame selector
        int next_element = (1.0 - this->eds_config.data_loader.overlap)*this->eds_config.data_loader.num_events;
        this->events.erase(this->events.begin(), this->events.begin()+next_element);
        #ifdef DEBUG_PRINTS
        std::cout<<"** [EDS_TASK EVENTS] ef_events at["<<ef_events[0].ts.toMicroseconds()<<"] size:"<<ef_events.size()<<std::endl;
        std::cout<<"** [EDS_TASK EVENTS] overlap ["<< this->eds_config.data_loader.overlap*100.0 <<"] this->events at["<<this->events[0].ts.toMicroseconds()<<"] size:"<<this->events.size()<<std::endl;
        #endif

        /** Create the Event Frame **/
        this->event_frame->create(this->ef_idx, ef_events, this->cam_calib.cam1,
                        this->eds_config.tracker.options.max_num_iterations.size(),
                        base::Affine3d::Identity(), this->newcam->out_size);

        /** Increment EF INDEX **/
        this->ef_idx++;

        /** EDS TRACKER OPTIMIZATION **/
        if (this->initialized)
        {
            /** Event to Image alignment T_kf_ef delta pose **/
            ::base::Transform3d T_kf_ef = this->pose_kf_ef.getTransform(); // initialize to current estimate
            // this is the actual tracker that handles the poses
            this->eventsToImageAlignment(ef_events, T_kf_ef); // EDS tracker estimate

            /** Set the EventFrame pose: T_w_ef with the result from alignment**/
            this->event_frame->setPose(this->pose_w_kf.getTransform()*T_kf_ef); // T_w_ef = T_w_kf * T_kf_ef

            // not exactly sure what is going on here
            /** Update the output port: T_kf_ef **/
            this->pose_kf_ef.time = this->event_frame->time;
            this->pose_kf_ef.setTransform(T_kf_ef);
            #ifdef DEBUG_PRINTS
            std::cout<<"** [EDS_TASK EVENTS] Wrote pose_kf_ef:\n"<<this->pose_kf_ef.getTransform().matrix()<<std::endl;
            #endif

            /** Write output port: T_w_ef **/
            this->pose_w_ef.time = this->event_frame->time;
            this->pose_w_ef.setTransform(this->event_frame->getPose());
            this->pose_w_ef.velocity = this->pose_w_ef.getTransform() * this->event_tracker->linearVelocity();
            this->pose_w_ef.angular_velocity = this->pose_w_ef.getTransform() * this->event_tracker->angularVelocity();
            _pose_w_ef.write(this->pose_w_ef);
            #ifdef DEBUG_PRINTS
            std::cout<<"** [EDS_TASK EVENTS] Wrote pose_w_ef:\n"<<this->pose_w_ef.getTransform().matrix()<<std::endl;
            #endif


            // not needed , all ecept outputTrackerInfo is visual info
            state(LOGGING);
            {
                /** Write the event frame **/
                this->outputEventFrameViz(this->event_frame);

                /** Output Generative Model **/
                this->outputGenerativeModelFrameViz(this->key_frame, this->event_frame->time);

                /** Output Optical Flow **/
                this->outputOpticalFlowFrameViz(this->key_frame, this->event_frame->time);

                /** Output Inverse depth and the KF Map **/
                this->outputInvDepthAndLocalMap(this->key_frame, this->event_frame->time);

                /** Tracker infos **/
                this->outputTrackerInfo(this->event_frame->time);
            }
            state(RUNNING);
        }
    }

}

/**
    Gets called whenever a frame image comes from the buffer
*/
void Task::frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"** [EDS_TASK] FRAME IDX:"<< this->frame_idx<<" Received Frame at ["<<frame_sample->time.toSeconds()<<"]**\n";
    #endif

    /** Get the image in DSO format **/
    // fuction needs to be modified as it expects an RTT type
    dso::ImageAndExposure* img = this->getImageAndExposure(*frame_sample);
    std::cout<<"[EDS_TASK] img time: "<<img->timestamp<<" size: "<<img->w<<"x"<<img->h <<std::endl;

    /** Get current info **/
    size_t window_size = this->frame_hessians.size();
    std::cout<<"[EDS_TASK FRAME] Keyframes size (Hessian frames): "<< window_size<<" All Frames size: "<<this->all_frame_history.size()
    <<" INIT: "<<(this->initialized?"TRUE":"FALSE")<<" INTERRUPT: "<<(_frame_interrupt.value()?"TRUE": "FALSE") <<std::endl;

    /** Track new frame **/
    if (this->initialized)
    {

        /** Track the current image frame and later decide whether it is a Keyframe **/
        if (!_frame_interrupt.value())
            this->track(img, this->frame_idx);

        if (this->kf_idx < this->frame_hessians[this->frame_hessians.size()-1]->frameID)
        {   // probably not interested in anything to do with logging
            state(LOGGING);
            {
                /** Info **/
                for (auto it : this->frame_hessians)
                {
                    std::cout<<"[EDS_TASK] KF ID: "<<it->frameID<<" FRAME ID: "<<it->shell->id<<" NUM POINTS: "<<it->pointHessians.size()
                    <<" MARGI POINTS: "<<it->pointHessiansMarginalized.size()<<" OUTLIERS: "<<it->pointHessiansOut.size()
                    <<" IMMATURE: "<<it->immaturePoints.size()<<"\n";// FH T_cam_world\n"<<it->worldToCam_evalPT.matrix3x4()
                    //<<"\nSHELL T_w_cam:\n"<<it->shell->camToWorld.matrix3x4()<<std::endl;
                }

                /** Output the map **/
                this->outputGlobalMap();

                /** Sliding windows of Keyframes **/
                this->outputKeyFrameMosaicViz(this->frame_hessians, frame_sample->time);

                /** Output pose for the sliding window **/
                this->outputPoseKFs(frame_sample->time);
            }
            state(RUNNING);

            /** Update the index **/
            this->kf_idx =  this->frame_hessians[this->frame_hessians.size()-1]->frameID;
        }

       std::cout<<"[EDS_TASK] FRAME HISTORY SIZE: "<<this->all_frame_history.size()<<" BUNDLES MAP POINTS: "<<this->bundles->nPoints<<std::endl;
    }
    else
    {
        // actually we should never get in the uninit case?
        // this seems to be init, should put it there
        state(INITIALIZING);
        this->initialize(img, this->frame_idx, 10);
        dso::FrameHessian *first_frame = this->initializer->firstFrame;
        dso::FrameHessian *new_frame = this->initializer->newFrame;
        if (first_frame)
            std::cout<<"[EDS_TASK] INIT FIRST FRAME ID: "<<first_frame->shell->incoming_id;
        if (new_frame)
            std::cout<<" NEW FRAME ID: "<<new_frame->shell->incoming_id;
        std::cout<<"\n";
        std::cout<<"[EDS_TASK] INIT T_INIT:\n"<<this->initializer->thisToNext.matrix3x4()<<std::endl;
        std::cout<<"[EDS_TASK] INIT FRAME ID: "<<this->all_frame_history[this->frame_idx]->id <<"\n T_world_cam:\n"
        <<this->all_frame_history[this->frame_idx]->camToWorld.matrix3x4()<<std::endl;
    }

    /** Increase the frame index and delete the original image **/
    this->frame_idx++;
    delete img;
}

void Task::imuCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sample)
{
    /** Buffer of inertial measurements window **/
    //this->imus.push_back(imu_sample);
}

void Task::groundtruthCallback(const base::Time &ts, const ::base::samples::RigidBodyState &groundtruth_sample)
{
    //this->pose_w_gt = groundtruth_sample;
    //this->_pose_w_gt.write(groundtruth_sample);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    /**
        Callbacks to imu event frame are set in TaskBase::configureHook() . They are binded 
        to  boost::bind. Streams are set on the frames and events and the framecallback is called whenever 
        na event/imu/frame is received on the stream. 
    */
    if (! TaskBase::configureHook())
        return false;

    /** Read the Yaml configuration file 
        -> most likely needed to be changed in slambench so config is passed differently
    **/
    YAML::Node config = YAML::LoadFile(_config_file.get());
    eds_config.data_loader = this->readDataLoaderConfig(config["data_loader"]);
    this->eds_config.tracker = ::eds::tracking::readTrackingConfig(config["tracker"]);
    this->eds_config.mapping = ::eds::mapping::readMappingConfig(config["mapping"]);
    this->eds_config.bundles = ::eds::bundles::readBundlesConfig(config["bundles"]);

    /** Read the camera calibration 
        -> agains read from yaml file, might need to be changed for slambench
    **/
    YAML::Node node_info = YAML::LoadFile(_calib_file.get());
    this->cam_calib = eds::calib::readDualCalibration(node_info);

    /** Set cameras calibration 
        -> this should be fine in SLAMBench 
    **/
    this->cam0 = std::make_shared<eds::calib::Camera>(this->cam_calib.cam0);
    this->cam1 = std::make_shared<eds::calib::Camera>(this->cam_calib.cam1, this->cam_calib.extrinsics.rotation);

    std::cout<<"** [EDS_TASK CONFIG] Configuration CAM0: **"<<std::endl;
    std::cout<<"Model: "<<cam_calib.cam0.distortion_model<<std::endl;
    std::cout<<"Size: "<<cam0->size<<std::endl;
    std::cout<<"K:\n"<<cam0->K<<std::endl;
    std::cout<<"D:\n"<<cam0->D<<std::endl;
    std::cout<<"R:\n"<<cam0->R<<std::endl;

    std::cout<<"** [EDS_TASK CONFIG] Configuration CAM1: **"<<std::endl;
    std::cout<<"Model: "<<cam_calib.cam1.distortion_model<<std::endl;
    std::cout<<"Size: "<<cam1->size<<std::endl;
    std::cout<<"K:\n"<<cam1->K<<std::endl;
    std::cout<<"D:\n"<<cam1->D<<std::endl;
    std::cout<<"R:\n"<<cam1->R<<std::endl;

    cv::Size out_size{this->cam_calib.cam0.out_width, this->cam_calib.cam0.out_height};
    this->newcam = std::make_shared<eds::calib::Camera>(::eds::calib::setNewCamera(*(this->cam0), *(this->cam1), out_size));

    std::cout<<"** [EDS_TASK CONFIG] Configuration NEWCAM: **"<<std::endl;
    std::cout<<"Size: "<<newcam->size<<std::endl;
    std::cout<<"Out Size: "<<newcam->out_size<<std::endl;
    std::cout<<"K:\n"<<newcam->K<<std::endl;
    std::cout<<"D:\n"<<newcam->D<<std::endl;
    std::cout<<"R:\n"<<newcam->R<<std::endl;

    ::eds::calib::getMapping(*cam0, *cam1, *newcam);
    std::cout<<"cam0.mapx: "<<cam0->mapx.rows<<" x "<<cam0->mapx.cols<<std::endl;
    std::cout<<"cam0.mapy: "<<cam0->mapy.rows<<" x "<<cam0->mapy.cols<<std::endl;
    std::cout<<"cam1.mapx: "<<cam1->mapx.rows<<" x "<<cam1->mapx.cols<<std::endl;
    std::cout<<"cam1.mapy: "<<cam1->mapy.rows<<" x "<<cam1->mapy.cols<<std::endl;

    /** Intrinsics info **/
    auto print = [](std::vector<double> &vector)
    {
        for (auto it: vector) {std::cout<<it<<", ";}
    };
    std::cout<<"** [EDS_TASK CONFIG] Frame intrinsics [";print(this->cam_calib.cam0.intrinsics); std::cout<<"]"<<std::endl;
    std::cout<<"** [EDS_TASK CONFIG] Events intrinsics [";print(this->cam_calib.cam1.intrinsics); std::cout<<"]"<<std::endl;

    /** For points activation **/
    this->currentMinActDist = 2;

    /* DSO Calibration
        -> paths differe and sensor callibrations will most likely have to be handled by SLAMBench
    **/
    this->newcam->toDSOFormat();
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
    std::cout<<"** [EDS CONFIG] Point Relative Baseline: "<<this->eds_config.mapping.points_rel_baseline<<std::endl;

    return true;
}


/**
    initialisation for the functional part. Config seems to be more concerned wit the dataset 
    while this one with the the logis of the tracker

    # this i
*/
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** Reset counters **/
    this->ef_idx = this->frame_idx = this->kf_idx = 0;

    /** DSO intitialization flags **/
    this->initialized = false;
    this->first_track = true;

    /** Is Lost is false **/
    this->is_lost = false;

    /** Initialize T_w_kf 
        key frame camera pose with respect to world

    **/
    this->pose_w_kf.targetFrame = "world";
    this->pose_w_kf.sourceFrame = "kf";
    this->pose_w_kf.setTransform(::base::Transform3d::Identity());

    /** Initialize T_kf_ef 
        event camera pose with respect to keyframe
    **/
    this->pose_kf_ef.targetFrame = "kf";
    this->pose_kf_ef.sourceFrame = "ef";
    this->pose_kf_ef.setTransform(::base::Transform3d::Identity());

    /** Initialize T_w_ef 
        event camera pose with respect to world
    **/
    this->pose_w_ef.targetFrame = "world";
    this->pose_w_ef.sourceFrame = "ef";
    this->pose_w_ef.setTransform(::base::Transform3d::Identity());

    /** Initializer constructor (DSO) **/
    this->initializer = std::make_shared<dso::CoarseInitializer>(dso::wG[0], dso::hG[0]);

    /* Event-based Tracker (EDS) **/
    this->event_tracker = std::make_shared<::eds::tracking::Tracker>(this->eds_config.tracker);

    /** KeyFrame (EDS) **/
    this->key_frame = std::make_shared<eds::tracking::KeyFrame>(*(this->cam0), *(this->newcam), this->cam_calib.cam0.distortion_model);

    /** EventFrame (EDS) **/
    this->event_frame = std::make_shared<eds::tracking::EventFrame>(*(this->cam1), *(this->newcam), this->cam_calib.cam1.distortion_model);

    /** Image-based Tracker constructor (DSO) **/
    this->image_tracker = std::make_shared<dso::CoarseTracker>(dso::wG[0], dso::hG[0]);
    this->last_coarse_RMSE.setConstant(100);

    /** Mapping 
        output port see eds.orogen
    **/
    this->selection_map = new float[dso::wG[0]*dso::hG[0]];
    this->pixel_selector = std::make_shared<dso::PixelSelector>(dso::wG[0], dso::hG[0]);
    this->coarse_distance_map = std::make_shared<dso::CoarseDistanceMap>(dso::wG[0], dso::hG[0]);
    this->depthmap = std::make_shared<eds::mapping::IDepthMap2d>(this->cam_calib.cam0.intrinsics);//Depthmap is in the original RGB frame

    /** Bundles constructor 
        might not be needed. Bundles should be rock dependent
    **/
    this->bundles = std::make_shared<dso::EnergyFunctional>();
    this->bundles->red = &(this->thread_reduce); //asign the threads

    return true;
}

/**
    No Idea why there's nothing here
*/
void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

/**Called when the algorithm is done. Should clean the resources used
    Together with cleanupHook is correspondent to sb_clean_slam_system
*/
void Task::stopHook()
{
    TaskBase::stopHook();
    this->printResult("stamped_traj_estimate.txt");

    this->initializer.reset();
    this->event_tracker.reset();
    this->event_frame.reset();
    this->key_frame.reset();
    this->image_tracker.reset();
    delete[] this->selection_map;
    this->pixel_selector.reset();
    this->coarse_distance_map.reset();
    this->depthmap.reset();
    this->bundles.reset();

    /** delete frames **/
    for (auto it : this->all_frame_history)
        if (it){delete it; it=nullptr;}

    this->events.clear();
    this->all_frame_history.clear();
    this->all_keyframes_history.clear();
    this->frame_hessians.clear();
    this->active_residuals.clear();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    this->calib.reset();
    this->undistort.reset();

}

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

dso::ImageAndExposure* Task::getImageAndExposure(const ::base::samples::frame::Frame &frame)
{
    /** Get the image frame in color (flip when required) **/
    cv::Mat mat_img = frame_helper::FrameHelper::convertToCvMat(frame);
    if (this->cam_calib.cam0.flip) cv::flip(mat_img, mat_img, 1);
    this->cam0->undistort(mat_img, this->img_frame);

    /** If image has attributes search for the exposure time **/
    double exposure_time = 0.0;
    if (frame.hasAttribute("exposure_time_us"))
    {
        exposure_time = frame.getAttribute<double>("exposure_time_us") * 1e-03;
    }

    /** Split images in R, G, B channels **/
    cv::split(this->img_frame,this->img_rgb);

    /** Create the ImageExposure object **/
    cv::Mat img_gray; cv::cvtColor(this->img_frame, img_gray, cv::COLOR_RGB2GRAY);
    dso::MinimalImageB* img = new dso::MinimalImageB(img_gray.cols, img_gray.rows);
    memcpy(img->data, img_gray.data, img_gray.rows*img_gray.cols);
    dso::ImageAndExposure* result = this->undistort->undistort<unsigned char>(
            img, float(exposure_time), frame.time.toSeconds());
    delete img;
    return result;
}

dso::SE3 Task::initialize(dso::ImageAndExposure* image, int id, const int &snapped_threshold)
{
    // =========================== add into allFrameHistory =========================
    dso::FrameHessian* fh = new dso::FrameHessian();
    dso::FrameShell* shell = new dso::FrameShell();
    shell->camToWorld = dso::SE3();
    shell->aff_g2l = dso::AffLight(0,0);
    shell->marginalizedAt = shell->id = this->all_frame_history.size();
    shell->timestamp = image->timestamp;
    shell->incoming_id = id;
    fh->shell = shell;
    this->all_frame_history.push_back(shell);

    // =========================== make Images / derivatives etc. =========================
    fh->ab_exposure = image->exposure_time;
    fh->makeImages(image->image, this->calib.get());

    /** Output image while trying to initialize **/
    this->outputImmaturePtsFrameViz(fh, ::base::Time::fromSeconds(fh->shell->timestamp));

    if(this->initializer->frameID<0) // first frame set. fh is kept by coarseInitializer.
    {
        this->initializer->setFirst(this->calib.get(), fh);
    }
    else if(this->initializer->trackFrame(fh, snapped_threshold))	// if SNAPPED
    {
        /** Initialize: this set the initialized flag to true and push the first KF **/
        {
            std::cout<<"[EDS_TASK] START INITIALIZE FROM DSO INITIALIZER"<<std::endl;

            /** add first keyframe: this is what EDS calls add frame to Hessian struct **/
            dso::FrameHessian* firstFrame = this->initializer->firstFrame;
            firstFrame->idx = this->frame_hessians.size();
            this->frame_hessians.push_back(firstFrame);
            firstFrame->frameID = this->all_keyframes_history.size();
            this->all_keyframes_history.push_back(firstFrame->shell);

            /** this is needed for bundles adjustment (energy functional) **/
            this->bundles->insertFrame(firstFrame, this->calib.get());
            this->setPrecalcValues();

            /** Reserve memory for points in first keyframe **/
            firstFrame->pointHessians.reserve(dso::wG[0]*dso::hG[0]*0.2f); //max 20% of all pixles
            firstFrame->pointHessiansMarginalized.reserve(dso::wG[0]*dso::hG[0]*0.2f); //max 20% of all pixels
            firstFrame->pointHessiansOut.reserve(dso::wG[0]*dso::hG[0]*0.2f);// max 20% of all pixels

            float sumID=1e-5, numID=1e-5;
            for(int i=0;i<this->initializer->numPoints[0];i++)
            {
                sumID += this->initializer->points[0][i].iR;
                numID++;
            }
            float rescaleFactor = 1 / (sumID / numID);

            /** randomly sub-select the points I need **/
            float keepPercentage = dso::setting_desiredPointDensity / this->initializer->numPoints[0];

            if(!dso::setting_debugout_runquiet)
                printf("[EDS_TASK] Initialization: keep %.1f%% (need %d, have %d)!\n", 100*keepPercentage,
                        (int)(dso::setting_desiredPointDensity), this->initializer->numPoints[0] );

            for(int i=0;i<this->initializer->numPoints[0];i++)
            {
                if(rand()/(float)RAND_MAX > keepPercentage) continue;

                dso::Pnt* point = this->initializer->points[0]+i;
                dso::ImmaturePoint* pt = new dso::ImmaturePoint(point->u+0.5f,point->v+0.5f,
                                            firstFrame,point->my_type, this->calib.get(),
                                            &(this->img_rgb[0].at<unsigned char>(0)),
                                            &(this->img_rgb[1].at<unsigned char>(0)),
                                            &(this->img_rgb[2].at<unsigned char>(0))
                                            );

                if(!std::isfinite(pt->energyTH)) { delete pt; continue; }

                pt->idepth_max=pt->idepth_min=1;
                /** INIT  is where  the PointsHessian memory is reserved **/
                dso::PointHessian* ph = new dso::PointHessian(pt, this->calib.get());
                delete pt;
                if(!std::isfinite(ph->energyTH)) {delete ph; continue;}

                ph->setIdepthScaled(point->iR*rescaleFactor);
                ph->setIdepthZero(ph->idepth);
                ph->hasDepthPrior=true;
                ph->setPointStatus(dso::PointHessian::ACTIVE);

                firstFrame->pointHessians.push_back(ph);

                /** Insert the Points in bundles energy functional **/
                this->bundles->insertPoint(ph);
            }

            dso::SE3 firstToNew = this->initializer->thisToNext;
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

            this->initialized=true;

            /** Optimize the first two DSO Keyframes and set the first EDS Keyframe**/
            this->makeKeyFrame(fh);

            /** Scale factor **/
            this->rescale_factor = rescaleFactor;

            /** INIT SUCCESSFUL : Port out first information **/
            state(LOGGING);
            {
                /** Info **/
                for (auto it : this->frame_hessians)
                {
                    std::cout<<"[EDS_TASK] KF ID: "<<it->frameID<<" FRAME ID: "<<it->shell->id<<" NUM POINTS: "<<it->pointHessians.size()
                    <<" MARGI POINTS: "<<it->pointHessiansMarginalized.size()<<" OUTLIERS: "<<it->pointHessiansOut.size()
                    <<" IMMATURE: "<<it->immaturePoints.size()<<"\n";
                }

                /** For Viz debugging the initial map in the first KF **/
                //this->outputDepthMap(this->depthmap, ::base::Time::fromSeconds(fh->shell->timestamp));

                /** Output the map **/
                this->outputGlobalMap();

                /** Output pose for the sliding window **/
                this->outputPoseKFs(::base::Time::fromSeconds(image->timestamp));
            }

            /** Initialized: set state to RUNNING **/
            state(RUNNING);

            std::cout<<"** [EDS_TASK] INITIALIZE FROM INITIALIZER ("<< (int)firstFrame->pointHessians.size() <<" pts) "<< rescaleFactor <<" rescale_factor!"<<std::endl;
            std::cout<<"** [EDS_TASK] INITIALIZATION FINISHED WITH T_w_kf:\n"<< this->pose_w_kf.getTransform().matrix()<<std::endl;
        }
    }
    else
    {
        /** if still initializing **/
        fh->shell->poseValid = false;
        delete fh;
    }
    if (this->initialized) return fh->get_worldToCam_evalPT().inverse(); //return T_w_kf
    else return dso::SE3();
}

/**
    This handles the event-based odometry pipeline
*/
bool Task::eventsToImageAlignment(const std::vector<::base::samples::Event> &events_array, ::base::Transform3d &T_kf_ef)
{
    /** Keyframe to Eventframe delta pose **/
    T_kf_ef = ::base::Transform3d::Identity();

    /** Execute the tracker and get the T_kf_ef **/
    state(OPTIMIZING);
    bool success = false;
    for (int i=this->event_frame->event_frame.size()-1; i>=0; --i)
    {
        success = this->event_tracker->optimize(i, &(this->event_frame->event_frame[i]), T_kf_ef, ::eds::tracking::MAD);
    }

    /** Track the points and remove the ones out of the image plane **/
    std::vector<cv::Point2d> coord = this->event_tracker->getCoord(true);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EDS_TASK EVENT_TO_IMG ALIGNMENT] "<<(success?"SUCCESS": "NO_USABLE") <<" rescale_factor: "<< this->rescale_factor<<"\nT_kf_ef:\n"<<T_kf_ef.matrix()<<std::endl;
    #endif
    state(RUNNING);

    return success;
}

void Task::setPrecalcValues()
{
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        fh->targetPrecalc.resize(this->frame_hessians.size());
        for(unsigned int i=0;i<this->frame_hessians.size();i++)
            fh->targetPrecalc[i].set(fh, this->frame_hessians[i], this->calib.get());
    }

    this->bundles->setDeltaF(this->calib.get());
}

void Task::track(dso::ImageAndExposure* image, int id)
{
    /** Skipping frames for the paper **/
    /*if (this->frame_hessians.size() == this->eds_config.bundles.window_size && (this->frame_idx % 10) != 0)
    {
        std::cout<<"[DSO_TASK FRAME] JUMPING FRAME: "<<this->frame_idx <<std::endl;
        return;
    }*/

    /** Add into allFrameHistory **/
    dso::FrameHessian* fh = new dso::FrameHessian();
    dso::FrameShell* shell = new dso::FrameShell();
    shell->camToWorld = SE3();//no lock required, as fh is not used anywhere yet.
    shell->aff_g2l = dso::AffLight(0,0);
    shell->marginalizedAt = shell->id = this->all_frame_history.size();
    shell->timestamp = image->timestamp;
    shell->incoming_id = id;
    fh->shell = shell;
    this->all_frame_history.push_back(shell);

    /** make Images / derivatives etc. **/
    fh->ab_exposure = image->exposure_time;
    fh->makeImages(image->image, this->calib.get());

    dso::Vec4 tres = this->trackNewFrame(fh, dso::BaseTransformToSE3(this->pose_kf_ef.getTransform()));
    if(!std::isfinite((double)tres[0]) || !std::isfinite((double)tres[1]) || !std::isfinite((double)tres[2]) || !std::isfinite((double)tres[3]))
    {
        std::cout<<"Image Tracking failed: LOST!"<<std::endl;
        return;
    }


    bool create_kf = false;
    dso::Vec2 refToFh=dso::AffLight::fromToVecExposure(this->image_tracker->lastRef->ab_exposure, fh->ab_exposure,
            this->image_tracker->lastRef_aff_g2l, fh->shell->aff_g2l);

    std::cout<<"** [EDS_TASK] "<<(this->event_tracker->needNewKeyframe() ?"NEW KEYFRAME": "NO KEYFRAME")<<std::endl;
    std::cout<<"[EDS_TASK] tres[3]: "<<tres[3]<<" sqrtf(tres[3]/(640*480): "<<sqrtf((double)tres[3]) / (dso::wG[0]+dso::hG[0])
    << "value: "<<dso::setting_kfGlobalWeight*dso::setting_maxShiftWeightRT * sqrtf((double)tres[3]) / (dso::wG[0]+dso::hG[0]) <<std::endl;

    /** CREATE A NEW KF **/
    create_kf = this->all_frame_history.size() == 1 ||
            dso::setting_kfGlobalWeight*dso::setting_maxShiftWeightT *  sqrtf((double)tres[1]) / (dso::wG[0]+dso::hG[0]) +
            dso::setting_kfGlobalWeight*dso::setting_maxShiftWeightR *  sqrtf((double)tres[2]) / (dso::wG[0]+dso::hG[0]) +
            dso::setting_kfGlobalWeight*dso::setting_maxShiftWeightRT * sqrtf((double)tres[3]) / (dso::wG[0]+dso::hG[0]) +
            dso::setting_kfGlobalWeight*dso::setting_maxAffineWeight * fabs(logf((float)refToFh[0])) > 1 ||
            2*this->image_tracker->firstCoarseRMSE < tres[0];

    if (!create_kf) std::cout<<"[EDS_TRACK]: NO NEED NEW KF"<<std::endl; else std::cout<<"[EDS_TRACK]: CREATE NEW KF"<<std::endl;

    if(create_kf) this->makeKeyFrame(fh);
    else this->makeNonKeyFrame(fh);

    return;
}

dso::Vec4 Task::trackNewFrame(dso::FrameHessian* fh, const dso::SE3 &se3_kf_ef)
{
    assert(this->all_frame_history.size() > 0);

    dso::FrameHessian* lastKF = this->image_tracker->lastRef; //Reference to the last KeyFrame

    std::vector<dso::SE3,Eigen::aligned_allocator<dso::SE3>> lastKF_2_fh_tries;//T_fh_lastKF
    lastKF_2_fh_tries.push_back(se3_kf_ef.inverse()); // assume exactly the T_ef_kf, the Event frame to the KF

    dso::Vec3 flowVecs = dso::Vec3(100,100,100);
    dso::SE3 lastF_2_fh = dso::SE3();
    dso::AffLight aff_g2l = dso::AffLight(0,0);

    /** Affine brightness transformation from the the last Frame (no Keyframe)**/
    dso::AffLight aff_last_2_l = dso::AffLight(0,0);
    aff_last_2_l = this->all_frame_history[this->all_frame_history.size()-2]->aff_g2l;

    /***********************/
    /** The Image tracker **/
    /***********************/
    dso::Vec5 achievedRes = dso::Vec5::Constant(NAN);
    dso::AffLight aff_g2l_this = aff_last_2_l;
    dso::SE3 lastF_2_fh_this = lastKF_2_fh_tries[0];
    bool good_tracking = this->image_tracker->trackNewestCoarse(
            fh, lastF_2_fh_this, aff_g2l_this,
            dso::pyrLevelsUsed-1,
            achievedRes);

    /** Did the image tracker work using the events-to-image transformation? **/
    if(good_tracking && std::isfinite((float)this->image_tracker->lastResiduals[0]) && !(this->image_tracker->lastResiduals[0] >=  achievedRes[0]))
    {
        flowVecs = this->image_tracker->lastFlowIndicators;
        aff_g2l = aff_g2l_this;
        lastF_2_fh = lastF_2_fh_this;
    }

    /** take over achieved res (always). **/
    if(good_tracking)
    {
        for(int i=0;i<5;i++)
        {
            if(!std::isfinite((float)achievedRes[i]) || achievedRes[i] > this->image_tracker->lastResiduals[i])	// take over if achievedRes is either bigger or NAN.
                achievedRes[i] = this->image_tracker->lastResiduals[i];
        }
    }

    /** Image tracking failed **/
    if(!good_tracking)
    {
        std::cout<<"[IMAGE_TRACKER] BIG ERROR! IMAGE TRACKER FAILED. MOST LIKELY THE EVENT_TRACKER FAILED BACUSE OF WRONG DEPTH MAP:"
                 <<" TAKE THE PREDICTED POSE AND HOPE FOR THE BEST"<<std::endl;

        /** We recover with DSO in case ythe event tracker is lost **/
        this->recoveryTracking(fh);
        if (!good_tracking)
        {
            flowVecs = dso::Vec3(0,0,0);
            aff_g2l = aff_last_2_l;
            lastF_2_fh = lastKF_2_fh_tries[0];
        }
    }

    this->last_coarse_RMSE = achievedRes;

    fh->shell->camToTrackingRef = lastF_2_fh.inverse();//Optimized T_kf_cam which is T_kf_ef
    fh->shell->trackingRef = lastKF->shell; //shall frame in the last KF
    fh->shell->aff_g2l = aff_g2l; //Optimize affine brigness values
    fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef; //T_w_cam = T_w_kf * T_kf_cam

    if(this->image_tracker->firstCoarseRMSE < 0)
        this->image_tracker->firstCoarseRMSE = achievedRes[0];

    std::cout<<"[IMAGE_TRACKER] "<<(good_tracking?"SUCCESS": "FAILED")<<" Coarse Tracker tracked a = "<< aff_g2l.a <<"b = "<< aff_g2l.b <<" (exposure "<< fh->ab_exposure
    <<" Residual: "<< achievedRes[0]<<" rescale_factor: "<<this->rescale_factor<<std::endl;

    return dso::Vec4(achievedRes[0], flowVecs[0], flowVecs[1], flowVecs[2]);
}

dso::Vec4 Task::recoveryTracking(dso::FrameHessian* fh)
{
    /** Use DSD Image tracker as recovery tracking **/
    assert(this->all_frame_history.size() > 0);

    dso::FrameHessian* lastF = this->image_tracker->lastRef;
    dso::AffLight aff_last_2_l = dso::AffLight(0,0);

    std::vector<dso::SE3,Eigen::aligned_allocator<dso::SE3>> lastF_2_fh_tries;
    if(this->all_frame_history.size() == 2)
        for(unsigned int i=0;i<lastF_2_fh_tries.size();i++) lastF_2_fh_tries.push_back(dso::SE3());
    else
    {
        dso::FrameShell* slast = this->all_frame_history[this->all_frame_history.size()-2];
        dso::FrameShell* sprelast = this->all_frame_history[this->all_frame_history.size()-3];
        dso::SE3 slast_2_sprelast;
        dso::SE3 lastF_2_slast;

        slast_2_sprelast = sprelast->camToWorld.inverse() * slast->camToWorld;
        lastF_2_slast = slast->camToWorld.inverse() * lastF->shell->camToWorld;
        aff_last_2_l = slast->aff_g2l;

        dso::SE3 fh_2_slast = slast_2_sprelast;// assumed to be the same as fh_2_slast.


        // get last delta-movement.
        lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast);	// assume constant motion.
        lastF_2_fh_tries.push_back(fh_2_slast.inverse() * fh_2_slast.inverse() * lastF_2_slast);	// assume double motion (frame skipped)
        lastF_2_fh_tries.push_back(SE3::exp(fh_2_slast.log()*0.5).inverse() * lastF_2_slast); // assume half motion.
        lastF_2_fh_tries.push_back(lastF_2_slast); // assume zero motion.
        lastF_2_fh_tries.push_back(SE3()); // assume zero motion FROM KF.


        // just try a TON of different initializations (all rotations). In the end,
        // if they don't work they will only be tried on the coarsest level, which is super fast anyway.
        // also, if tracking rails here we loose, so we really, really want to avoid that.
        for(float rotDelta=0.02; rotDelta < 0.05; rotDelta++)
        {
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,0), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,0), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,rotDelta), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,0), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,0), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,-rotDelta), dso::Vec3(0,0,0)));			// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,0), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,0), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,0), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,0), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,-rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
            lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,rotDelta), dso::Vec3(0,0,0)));	// assume constant motion.
        }

        if(!slast->poseValid || !sprelast->poseValid || !lastF->shell->poseValid)
        {
            lastF_2_fh_tries.clear();
            lastF_2_fh_tries.push_back(SE3());
        }
    }


    dso::Vec3 flowVecs = dso::Vec3(100,100,100);
    dso::SE3 lastF_2_fh = dso::SE3();
    dso::AffLight aff_g2l = dso::AffLight(0,0);

    // as long as maxResForImmediateAccept is not reached, I'll continue through the options.
    // I'll keep track of the so-far best achieved residual for each level in achievedRes.
    // If on a coarse level, tracking is WORSE than achievedRes, we will not continue to save time.

    dso::Vec5 achievedRes = dso::Vec5::Constant(NAN);
    bool haveOneGood = false;
    int tryIterations=0;
    for(unsigned int i=0;i<lastF_2_fh_tries.size();i++)
    {
        dso::AffLight aff_g2l_this = aff_last_2_l;
        dso::SE3 lastF_2_fh_this = lastF_2_fh_tries[i];
        bool trackingIsGood = this->image_tracker->trackNewestCoarse(
                fh, lastF_2_fh_this, aff_g2l_this,
                dso::pyrLevelsUsed-1,
                achievedRes);	// in each level has to be at least as good as the last try.
        tryIterations++;

        if(i != 0)
        {
            printf("RE-TRACK ATTEMPT %d with initOption %d and start-lvl %d (ab %f %f): %f %f %f %f %f -> %f %f %f %f %f \n",
                    i,
                    i, dso::pyrLevelsUsed-1,
                    aff_g2l_this.a,aff_g2l_this.b,
                    achievedRes[0],
                    achievedRes[1],
                    achievedRes[2],
                    achievedRes[3],
                    achievedRes[4],
                    this->image_tracker->lastResiduals[0],
                    this->image_tracker->lastResiduals[1],
                    this->image_tracker->lastResiduals[2],
                    this->image_tracker->lastResiduals[3],
                    this->image_tracker->lastResiduals[4]);
        }


        // do we have a new winner?
        if(trackingIsGood && std::isfinite((float)this->image_tracker->lastResiduals[0]) && !(this->image_tracker->lastResiduals[0] >=  achievedRes[0]))
        {
            flowVecs = this->image_tracker->lastFlowIndicators;
            aff_g2l = aff_g2l_this;
            lastF_2_fh = lastF_2_fh_this;
            haveOneGood = true;
        }

        // take over achieved res (always).
        if(haveOneGood)
        {
            for(int i=0;i<5;i++)
            {
                if(!std::isfinite((float)achievedRes[i]) || achievedRes[i] > this->image_tracker->lastResiduals[i])	// take over if achievedRes is either bigger or NAN.
                    achievedRes[i] = this->image_tracker->lastResiduals[i];
            }
        }


        if(haveOneGood &&  achievedRes[0] < this->last_coarse_RMSE[0]*dso::setting_reTrackThreshold)
            break;

    }

    if(!haveOneGood)
    {
        std::cout<<"BIG ERROR! tracking failed entirely. Take predictred pose and hope we may somehow recover."<<std::endl;
        flowVecs = dso::Vec3(0,0,0);
        aff_g2l = aff_last_2_l;
        lastF_2_fh = lastF_2_fh_tries[0];
    }

    this->last_coarse_RMSE = achievedRes;

    // no lock required, as fh is not used anywhere yet.
    fh->shell->camToTrackingRef = lastF_2_fh.inverse();//Optimized T_kf_cam which is T_kf_ef
    fh->shell->trackingRef = lastF->shell; //shall frame in the last KF
    fh->shell->aff_g2l = aff_g2l; //Optimize affine brigness values
    fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef; //T_w_cam = T_w_kf * T_kf_cam

    if(this->image_tracker->firstCoarseRMSE < 0)
        this->image_tracker->firstCoarseRMSE = achievedRes[0];

    std::cout<<"[TRACKER] "<<(haveOneGood?"SUCCESS": "FAILED")<<" Coarse Tracker tracked a = "<< aff_g2l.a <<"b = "<< aff_g2l.b <<" (exposure "<< fh->ab_exposure <<" Residual: "<< achievedRes[0]<<std::endl;

    return dso::Vec4(achievedRes[0], flowVecs[0], flowVecs[1], flowVecs[2]);
}

void Task::makeNonKeyFrame(dso::FrameHessian* fh)
{
    /** reference to the KF cannot be null **/
    assert(fh->shell->trackingRef != 0);
    fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef; //T_w_cam = T_w_kf * T_kf_cam
    fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l);

    /* Here it traces immature points in the last Frame **/
    this->traceNewPoints(fh);

    /** THIS IS OPTIONAL BUT IS BRING ROBUSTNESS (when a frame is given and KF is not create, rarelly happens):
     * Update the event_tracker with the new T_kf_ef transform from the frame  **/
    //base::Transform3d T_kf_ef = this->pose_w_kf.getTransform().inverse() * dso::SE3ToBaseTransform(fh->shell->camToWorld);
    //this->event_tracker->set(T_kf_ef);

    delete fh;
}

void Task::makeKeyFrame(dso::FrameHessian* fh)
{
    /** reference to the KF cannot be null **/
    assert(fh->shell->trackingRef != 0);
    fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef; //T_w_cam = T_w_kf * T_kf_cam
    fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l);

    /* Here it traces immature points in the last Frame **/
    this->traceNewPoints(fh);

    /** Flag Frames to be Marginalized. **/
    this->flagFramesForMarginalization(fh);

    /** Add New Keyframe. New Frame to Hessian Struct. **/
    fh->idx = this->frame_hessians.size();
    this->frame_hessians.push_back(fh);
    fh->frameID = this->all_keyframes_history.size();
    this->all_keyframes_history.push_back(fh->shell);
    this->bundles->insertFrame(fh, this->calib.get());

    this->setPrecalcValues();

    /** Add new residuals for old point **/
    int numFwdResAdde=0;
    for(dso::FrameHessian* fh1 : this->frame_hessians)
    {
        if(fh1 == fh) continue;
        for(dso::PointHessian* ph : fh1->pointHessians)
        {
            dso::PointFrameResidual* r = new dso::PointFrameResidual(ph, fh1, fh);
            r->setState(dso::ResState::IN);
            ph->residuals.push_back(r);
            this->bundles->insertResidual(r);
            ph->lastResiduals[1] = ph->lastResiduals[0];
            ph->lastResiduals[0] = std::pair<dso::PointFrameResidual*, dso::ResState>(r, dso::ResState::IN);
            numFwdResAdde+=1;
        }
    }

    /**  Activate Points (& flag for marginalization) **/
    this->activatePointsMT();// Here is where immature points become hessian points (points in bundle adjustment)
    this->bundles->makeIDX(); //Here DSO reconstruct indexes before optimization backend

    /** OPTIMIZE ALL KFs IN THE SLIDING WINDOW **/
    fh->frameEnergyTH = this->frame_hessians.back()->frameEnergyTH;
    float rmse = this->optimize(dso::setting_maxOptIterations);

    /** IN CASE INITIALIZATION FAILED **/
    if(this->all_keyframes_history.size() <= 4)
    {
        if(this->all_keyframes_history.size()==2 && rmse > 20*dso::benchmark_initializerSlackFactor)
        {
            std::cout<<"INITIALIZATION FAILED! Resetting"<<std::endl;
        }
        if(this->all_keyframes_history.size()==3 && rmse > 13*dso::benchmark_initializerSlackFactor)
        {
            std::cout<<"INITIALIZATION FAILED! Resetting"<<std::endl;
        }
        if(this->all_keyframes_history.size()==4 && rmse > 9*dso::benchmark_initializerSlackFactor)
        {
            std::cout<<"INITIALIZATION FAILED! Resetting"<<std::endl;
        }
    }

    /**  REMOVE OUTLIER IN THE KEYFRAMES **/
    this->removeOutliers();

    /** Set the new keyframe raference for the DSO image tracker **/
    this->image_tracker->makeK(this->calib.get()); //here calib info at all pyramide levels
    this->image_tracker->setCoarseTrackingRef(this->frame_hessians); // here the new info to be a KeyFrame (host ref frame) for the image_tracker

    /** (Activate-)Marginalize Points **/
    this->flagPointsForRemoval();
    this->bundles->dropPointsF();
    this->getNullspaces(
            this->bundles->lastNullspaces_pose,
            this->bundles->lastNullspaces_scale,
            this->bundles->lastNullspaces_affA,
            this->bundles->lastNullspaces_affB);
    this->bundles->marginalizePointsF();

    /** Add new Immature points & new residuals. Initialize Immature points with the GlobalMap **/
    this->makeNewTraces(fh, this->depthmap.get()); //this creates new points (ImmaturePoints) in the frame with inverse depth = 0 (UNINITIALIZED)

    /** Update the pose_w_kf: Time and the transformation of the optimized KF: T_w_kf**/
    this->pose_w_kf.time = ::base::Time::fromSeconds(fh->shell->timestamp);
    this->pose_w_kf.setTransform(dso::SE3ToBaseTransform(fh->shell->camToWorld)); //T_w_cam

    /** Event frame pose is set the new Keyframe **/
    this->event_frame->setPose(this->pose_w_kf.getTransform());

    /** Set the pose_w_ef: Time and the transformation of the optimized KF: T_w_kf**/
    this->pose_w_ef.setTransform(dso::SE3ToBaseTransform(fh->shell->camToWorld)); //T_w_cam

    /** Set the Keyframe to Eventframe transformation is the Identity**/
    this->pose_kf_ef.setTransform(::base::Transform3d::Identity());

    /** Create the Next Keyframe for the Event Tracker **/
    this->key_frame->create(fh->frameID, ::base::Time::fromSeconds(fh->shell->timestamp), this->img_frame, *(this->depthmap.get()), this->pose_w_kf.getTransform(), this->newcam->out_size);

    /** Reset the tracker with the new keyframe **/
    this->event_tracker->reset(this->key_frame, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

    /**  MARGINALIZE KEYFRAMES **/
    for(unsigned int i=0;i<this->frame_hessians.size();i++)
        if(this->frame_hessians[i]->flaggedForMarginalization)
            {this->marginalizeFrame(this->frame_hessians[i]); i=0;}
}

void Task::traceNewPoints(dso::FrameHessian* fh)
{
    int trace_total=0, trace_good=0, trace_oob=0, trace_out=0, trace_skip=0, trace_badcondition=0, trace_uninitialized=0;

    dso::Mat33f K = dso::Mat33f::Identity();
    K(0,0) = this->calib->fxl();
    K(1,1) = this->calib->fyl();
    K(0,2) = this->calib->cxl();
    K(1,2) = this->calib->cyl();

    for(dso::FrameHessian* host : this->frame_hessians)
    {

        dso::SE3 hostToNew = fh->PRE_worldToCam * host->PRE_camToWorld;
        dso::Mat33f KRKi = K * hostToNew.rotationMatrix().cast<float>() * K.inverse();
        dso::Vec3f Kt = K * hostToNew.translation().cast<float>();

        dso::Vec2f aff = dso::AffLight::fromToVecExposure(host->ab_exposure, fh->ab_exposure, host->aff_g2l(), fh->aff_g2l()).cast<float>();

        for(dso::ImmaturePoint* ph : host->immaturePoints)
        {
            /** This is the DSO method that traces the immature point with respect to the
             * current Frame (fh), The KeyFrame is host, which is the one storing the ImmaturePoints**/
            ph->traceOn(fh, KRKi, Kt, aff, this->calib.get(), false );

            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_GOOD) trace_good++;
            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_BADCONDITION) trace_badcondition++;
            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_OOB) trace_oob++;
            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_OUTLIER) trace_out++;
            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_SKIPPED) trace_skip++;
            if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_UNINITIALIZED) trace_uninitialized++;
            trace_total++;
        }
    }
    std::cout<<"[TRACE_NEW_POINTS] TOTAL:"<<trace_total <<" points. "
             << trace_good <<" ("<< 100*trace_good/(float)trace_total <<") good. "
             << trace_skip <<"("<< 100*trace_skip/(float)trace_total <<") skip. "
             << trace_badcondition<<"("<< 100*trace_badcondition/(float)trace_total <<") badcond. "
             << trace_oob <<"("<< 100*trace_oob/(float)trace_total <<") oob. "
             << trace_out <<"("<< 100*trace_out/(float)trace_total <<") out. "
             << trace_uninitialized <<"("<< 100*trace_uninitialized/(float)trace_total <<") uninit.\n";
}

void Task::marginalizeFrame(dso::FrameHessian* frame)
{
    // marginalize or remove all this frames points.
    assert((int)frame->pointHessians.size()==0);

    this->bundles->marginalizeFrame(frame->efFrame);

    // drop all observations of existing points in that frame.
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        if(fh==frame) continue;

        for(dso::PointHessian* ph : fh->pointHessians)
        {
            for(unsigned int i=0;i<ph->residuals.size();i++)
            {
                dso::PointFrameResidual* r = ph->residuals[i];
                if(r->target == frame)
                {
                    if(ph->lastResiduals[0].first == r)
                        ph->lastResiduals[0].first=0;
                    else if(ph->lastResiduals[1].first == r)
                        ph->lastResiduals[1].first=0;


                    this->bundles->dropResidual(r->efResidual);
                    this->deleteOut<dso::PointFrameResidual>(ph->residuals,i);
                    break;
                }
            }
        }
    }

    frame->shell->marginalizedAt = this->frame_hessians.back()->shell->id;
    frame->shell->movedByOpt = frame->w2c_leftEps().norm();

    this->deleteOutOrder<dso::FrameHessian>(this->frame_hessians, frame);
    for(unsigned int i=0;i<this->frame_hessians.size();i++)
        this->frame_hessians[i]->idx = i;

    this->setPrecalcValues();
    this->bundles->setAdjointsF(this->calib.get());
}

std::vector<base::Point> Task::getPoints(const ::base::Transform3d &T_kf_w, const bool &single_point)
{
    /** Instrinsics **/
    float fx = this->calib->fxl();
    float fy = this->calib->fyl();
    float cx = this->calib->cxl();
    float cy = this->calib->cyl();

    std::vector<base::Point> points;

    /** For all the keyframes **/
    for (auto fh : this->frame_hessians)
    {
        /** Move the point to world frame **/
        base::Transform3d T_w_kf = dso::SE3ToBaseTransform(fh->get_worldToCam_evalPT()).inverse();

        /** Move the poit in the target keyframe **/
        base::Transform3d T_kf_kf_i = T_kf_w * T_w_kf;

        /** For all the points in the keyframe **/
        for (auto p : fh->pointHessians)
        {
            double d_i = 1.0/p->idepth_scaled;
            points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-cy)/fy, d_i));//u-v point8

            if (!single_point)
            {
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v+2-cy)/fy, d_i));//x-y point1
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v+1-cy)/fy, d_i ));//x-y point2
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point3
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point4
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-2-cy)/fy, d_i ));//x-y point5
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u+1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point6
                points.push_back(T_kf_kf_i * ::base::Point(d_i*(p->u+2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point7
            }
        }
    }

    return points;
}

std::vector<cv::Point2d> Task::getImmatureCoords(const dso::FrameHessian *fh)
{
    std::vector<cv::Point2d> coord;

    for(dso::ImmaturePoint* p : fh->immaturePoints)
    {
        /** If point is good or is good but not tracked or is uninialized **/
        if(p->lastTraceStatus==dso::ImmaturePointStatus::IPS_GOOD ||
           p->lastTraceStatus==dso::ImmaturePointStatus::IPS_SKIPPED ||
           p->lastTraceStatus==dso::ImmaturePointStatus::IPS_UNINITIALIZED)
        {
            coord.push_back(cv::Point2d(p->u, p->v)); //Get the points coordinate
        }
    }

    return coord;
}

void Task::flagFramesForMarginalization(dso::FrameHessian* newFH)
{
    if(dso::setting_minFrameAge > dso::setting_maxFrames)
    {
        for(int i=dso::setting_maxFrames;i<(int)this->frame_hessians.size();i++)
        {
            dso::FrameHessian* fh = this->frame_hessians[i-dso::setting_maxFrames];
            fh->flaggedForMarginalization = true;
        }
        return;
    }

    int flagged = 0;
    // marginalize all frames that have not enough points.
    for(int i=0;i<(int)this->frame_hessians.size();i++)
    {
        dso::FrameHessian* fh = this->frame_hessians[i];
        int in = fh->pointHessians.size() + fh->immaturePoints.size();
        int out = fh->pointHessiansMarginalized.size() + fh->pointHessiansOut.size();

        dso::Vec2 refToFh=dso::AffLight::fromToVecExposure(this->frame_hessians.back()->ab_exposure, fh->ab_exposure,
                this->frame_hessians.back()->aff_g2l(), fh->aff_g2l());

        if( (in < dso::setting_minPointsRemaining *(in+out) || fabs(logf((float)refToFh[0])) > dso::setting_maxLogAffFacInWindow)
                && ((int)this->frame_hessians.size())-flagged > dso::setting_minFrames)
        {
            fh->flaggedForMarginalization = true;
            flagged++;
        }
    }

    // marginalize one.
    if((int)this->frame_hessians.size()-flagged >= dso::setting_maxFrames)
    {
        double smallestScore = 1;
        dso::FrameHessian* toMarginalize=0;
        dso::FrameHessian* latest = this->frame_hessians.back();


        for(dso::FrameHessian* fh : this->frame_hessians)
        {
            if(fh->frameID > latest->frameID-dso::setting_minFrameAge || fh->frameID == 0) continue;

            double distScore = 0;
            for(dso::FrameFramePrecalc &ffh : fh->targetPrecalc)
            {
                if(ffh.target->frameID > latest->frameID-dso::setting_minFrameAge+1 || ffh.target == ffh.host) continue;
                distScore += 1/(1e-5+ffh.distanceLL);

            }
            distScore *= -sqrtf(fh->targetPrecalc.back().distanceLL);


            if(distScore < smallestScore)
            {
                smallestScore = distScore;
                toMarginalize = fh;
            }
        }

        toMarginalize->flaggedForMarginalization = true;
        flagged++;
    }
}

void Task::activatePointsMT()
{
    if(this->bundles->nPoints < dso::setting_desiredPointDensity*0.66)
        this->currentMinActDist -= 1.0;
    if(this->bundles->nPoints < dso::setting_desiredPointDensity*0.8)
        this->currentMinActDist -= 0.5;
    else if(this->bundles->nPoints < dso::setting_desiredPointDensity*0.9)
        this->currentMinActDist -= 0.3;
    else if(this->bundles->nPoints < dso::setting_desiredPointDensity)
        this->currentMinActDist -= 0.2;

    if(this->bundles->nPoints > dso::setting_desiredPointDensity*1.5)
        this->currentMinActDist += 1.0;
    if(this->bundles->nPoints > dso::setting_desiredPointDensity*1.3)
        this->currentMinActDist += 0.6;
    if(this->bundles->nPoints > dso::setting_desiredPointDensity*1.15)
        this->currentMinActDist += 0.5;
    if(this->bundles->nPoints > dso::setting_desiredPointDensity)
        this->currentMinActDist += 0.2;

    if(this->currentMinActDist < 0) this->currentMinActDist = 0;
    if(this->currentMinActDist > 10) this->currentMinActDist = 10;

    std::cout<<"[ACTIVATE POINTS] SPARSITY:  MinActDist "<< this->currentMinActDist<<" (need "<< (int)(dso::setting_desiredPointDensity)
            <<" points, have "<< this->bundles->nPoints <<" points)!"<<std::endl;

    dso::FrameHessian* newestHs = this->frame_hessians.back();

    // make dist map.
    this->coarse_distance_map->makeK(this->calib.get());
    this->coarse_distance_map->makeDistanceMap(this->frame_hessians, newestHs);

    std::vector<dso::ImmaturePoint*> toOptimize; toOptimize.reserve(20000);

    for(dso::FrameHessian* host : this->frame_hessians)
    {
        if(host == newestHs) continue;

        dso::SE3 fhToNew = newestHs->PRE_worldToCam * host->PRE_camToWorld;
        dso::Mat33f KRKi = (this->coarse_distance_map->K[1] * fhToNew.rotationMatrix().cast<float>() * this->coarse_distance_map->Ki[0]);
        dso::Vec3f Kt = (this->coarse_distance_map->K[1] * fhToNew.translation().cast<float>());

        for(unsigned int i=0;i<host->immaturePoints.size();i+=1)
        {
            dso::ImmaturePoint* ph = host->immaturePoints[i];
            ph->idxInImmaturePoints = i;

            // delete points that have never been traced successfully, or that are outlier on the last trace.
            if(!std::isfinite(ph->idepth_max) || ph->lastTraceStatus == dso::IPS_OUTLIER)
            {
                // remove point.
                delete ph;
                host->immaturePoints[i]=nullptr;
                continue;
            }

            // can activate only if this is true.
            bool canActivate = (ph->lastTraceStatus == dso::IPS_GOOD
                    || ph->lastTraceStatus == dso::IPS_SKIPPED
                    || ph->lastTraceStatus == dso::IPS_BADCONDITION
                    || ph->lastTraceStatus == dso::IPS_OOB )
                            && ph->lastTracePixelInterval < 8
                            && ph->quality > dso::setting_minTraceQuality
                            && (ph->idepth_max+ph->idepth_min) > 0;

            // if I cannot activate the point, skip it. Maybe also delete it.
            if(!canActivate)
            {
                // if point will be out afterwards, delete it instead.
                if(ph->host->flaggedForMarginalization || ph->lastTraceStatus == dso::IPS_OOB)
                {
                    delete ph;
                    host->immaturePoints[i]=nullptr;
                }
                continue;
            }

            // see if we need to activate point due to distance map. Here is where the inverse depth from the
            // immature point is tacken.
            dso::Vec3f ptp = KRKi * dso::Vec3f(ph->u, ph->v, 1) + Kt*(0.5f*(ph->idepth_max+ph->idepth_min));
            int u = ptp[0] / ptp[2] + 0.5f;
            int v = ptp[1] / ptp[2] + 0.5f;

            if((u > 0 && v > 0 && u < dso::wG[1] && v < dso::hG[1]))
            {
                float dist = this->coarse_distance_map->fwdWarpedIDDistFinal[u+dso::wG[1]*v] + (ptp[0]-floorf((float)(ptp[0])));
                if(dist>=currentMinActDist* ph->my_type)
                {
                    this->coarse_distance_map->addIntoDistFinal(u,v);
                    toOptimize.push_back(ph);
                }
            }
            else
            {
                delete ph;
                host->immaturePoints[i]=nullptr;
            }
        }
    }

    std::vector<dso::PointHessian*> optimized; optimized.resize(toOptimize.size());

    if(dso::multiThreading)
        this->thread_reduce.reduce(boost::bind(&Task::activatePointsMT_Reductor, this, &optimized, &toOptimize, _1, _2, _3, _4), 0, toOptimize.size(), 50);
    else
        this->activatePointsMT_Reductor(&optimized, &toOptimize, 0, toOptimize.size(), 0, 0);

    for(unsigned k=0;k<toOptimize.size();k++)
    {
        dso::PointHessian* newpoint = optimized[k];
        dso::ImmaturePoint* ph = toOptimize[k];

        if(newpoint != 0 && newpoint != (dso::PointHessian*)((long)(-1)))
        {
            newpoint->host->immaturePoints[ph->idxInImmaturePoints]=0;
            newpoint->host->pointHessians.push_back(newpoint);
            this->bundles->insertPoint(newpoint);
            for(dso::PointFrameResidual* r : newpoint->residuals)
                this->bundles->insertResidual(r);
            assert(newpoint->efPoint != 0);
            delete ph;
        }
        else if(newpoint == (dso::PointHessian*)((long)(-1)) || ph->lastTraceStatus==dso::IPS_OOB)
        {
            delete ph;
            ph->host->immaturePoints[ph->idxInImmaturePoints]=nullptr;
        }
        else
        {
            assert(newpoint == 0 || newpoint == (PointHessian*)((long)(-1)));
        }
    }

    /** Cleaning up immatures points pointing to nullptr in the
    * window of keyframes **/
    for(dso::FrameHessian* host : this->frame_hessians)
    {
        for(int i=0;i<(int)host->immaturePoints.size();i++)
        {
            if(host->immaturePoints[i]==nullptr)
            {
                host->immaturePoints[i] = host->immaturePoints.back();
                host->immaturePoints.pop_back();
                i--;
            }
        }
    }
}

void Task::activatePointsMT_Reductor(
        std::vector<dso::PointHessian*>* optimized,
        std::vector<dso::ImmaturePoint*>* toOptimize,
        int min, int max, dso::Vec10* stats, int tid)
{
    dso::ImmaturePointTemporaryResidual* tr = new dso::ImmaturePointTemporaryResidual[this->frame_hessians.size()];
    for(int k=min;k<max;k++)
    {
        (*optimized)[k] = this->optimizeImmaturePoint((*toOptimize)[k],1,tr);
    }
    delete[] tr;
}

dso::PointHessian* Task::optimizeImmaturePoint(dso::ImmaturePoint* point, int minObs, dso::ImmaturePointTemporaryResidual* residuals)
{
    int nres = 0;
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        if(fh != point->host)
        {
            residuals[nres].state_NewEnergy = residuals[nres].state_energy = 0;
            residuals[nres].state_NewState = dso::ResState::OUTLIER;
            residuals[nres].state_state = dso::ResState::IN;
            residuals[nres].target = fh;
            nres++;
        }
    }
    assert(nres == ((int)this->frame_hessians.size())-1);

    bool print = false;
    float lastEnergy = 0;
    float lastHdd=0;
    float lastbd=0;
    float currentIdepth=(point->idepth_max+point->idepth_min)*0.5f;

    for(int i=0;i<nres;i++)
    {
        lastEnergy += point->linearizeResidual(this->calib.get(), 1000, residuals+i,lastHdd, lastbd, currentIdepth);
        residuals[i].state_state = residuals[i].state_NewState;
        residuals[i].state_energy = residuals[i].state_NewEnergy;
    }

    if(!std::isfinite(lastEnergy) || lastHdd < dso::setting_minIdepthH_act)
    {
        if(print)
            printf("OptPoint: Not well-constrained (%d res, H=%.1f). E=%f. SKIP!\n",
                nres, lastHdd, lastEnergy);
        return 0;
    }

    if(print) printf("Activate point. %d residuals. H=%f. Initial Energy: %f. Initial Id=%f\n" ,
            nres, lastHdd,lastEnergy,currentIdepth);

    float lambda = 0.1;
    for(int iteration=0;iteration<dso::setting_GNItsOnPointActivation;iteration++)
    {
        float H = lastHdd;
        H *= 1+lambda;
        float step = (1.0/H) * lastbd;
        float newIdepth = currentIdepth - step;

        float newHdd=0; float newbd=0; float newEnergy=0;
        for(int i=0;i<nres;i++)
            newEnergy += point->linearizeResidual(this->calib.get(), 1, residuals+i,newHdd, newbd, newIdepth);

        if(!std::isfinite(lastEnergy) || newHdd < dso::setting_minIdepthH_act)
        {
            if(print) printf("OptPoint: Not well-constrained (%d res, H=%.1f). E=%f. SKIP!\n",
                    nres,
                    newHdd,
                    lastEnergy);
            return 0;
        }

        if(print) printf("%s %d (L %.2f) %s: %f -> %f (idepth %f)!\n",
                (true || newEnergy < lastEnergy) ? "ACCEPT" : "REJECT",
                iteration,
                log10(lambda),
                "",
                lastEnergy, newEnergy, newIdepth);

        if(newEnergy < lastEnergy)
        {
            currentIdepth = newIdepth;
            lastHdd = newHdd;
            lastbd = newbd;
            lastEnergy = newEnergy;
            for(int i=0;i<nres;i++)
            {
                residuals[i].state_state = residuals[i].state_NewState;
                residuals[i].state_energy = residuals[i].state_NewEnergy;
            }

            lambda *= 0.5;
        }
        else
        {
            lambda *= 5;
        }

        if(fabsf(step) < 0.0001*currentIdepth)
            break;
    }

    if(!std::isfinite(currentIdepth))
    {
        printf("MAJOR ERROR! point idepth is nan after initialization (%f).\n", currentIdepth);
        return (dso::PointHessian*)((long)(-1));		// yeah I'm like 99% sure this is OK on 32bit systems.
    }


    int numGoodRes=0;
    for(int i=0;i<nres;i++)
        if(residuals[i].state_state == dso::ResState::IN) numGoodRes++;

    if(numGoodRes < minObs)
    {
        if(print) printf("OptPoint: OUTLIER!\n");
        return (dso::PointHessian*)((long)(-1));		// yeah I'm like 99% sure this is OK on 32bit systems.
    }



    /** HERE: is where  the PointsHessian memeory is reserved **/
    dso::PointHessian* p = new dso::PointHessian(point, this->calib.get());
    if(!std::isfinite(p->energyTH)) {delete p; return (dso::PointHessian*)((long)(-1));}

    p->lastResiduals[0].first = 0;
    p->lastResiduals[0].second = dso::ResState::OOB;
    p->lastResiduals[1].first = 0;
    p->lastResiduals[1].second = dso::ResState::OOB;
    p->setIdepthZero(currentIdepth);
    p->setIdepth(currentIdepth);
    p->setPointStatus(dso::PointHessian::ACTIVE);

    for(int i=0;i<nres;i++)
    {
        if(residuals[i].state_state == dso::ResState::IN)
        {
            dso::PointFrameResidual* r = new dso::PointFrameResidual(p, p->host, residuals[i].target);
            r->state_NewEnergy = r->state_energy = 0;
            r->state_NewState = dso::ResState::OUTLIER;
            r->setState(dso::ResState::IN);
            p->residuals.push_back(r);

            if(r->target == this->frame_hessians.back())
            {
                p->lastResiduals[0].first = r;
                p->lastResiduals[0].second = dso::ResState::IN;
            }
            else if(r->target == (this->frame_hessians.size()<2 ? 0 : this->frame_hessians[this->frame_hessians.size()-2]))
            {
                p->lastResiduals[1].first = r;
                p->lastResiduals[1].second = dso::ResState::IN;
            }
        }
    }

    return p;
}

void Task::removeOutliers()
{
    int numPointsDropped=0;
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        for(unsigned int i=0;i<fh->pointHessians.size();i++)
        {
            dso::PointHessian* ph = fh->pointHessians[i];
            if(ph==0) continue;

            if(ph->residuals.size() == 0)
            {
                fh->pointHessiansOut.push_back(ph);
                ph->efPoint->stateFlag = dso::EFPointStatus::PS_DROP;
                fh->pointHessians[i] = fh->pointHessians.back();
                fh->pointHessians.pop_back();
                i--;
                numPointsDropped++;
            }
        }
    }
    this->bundles->dropPointsF();
}

void Task::flagPointsForRemoval()
{
    assert(dso::EFIndicesValid);

    std::vector<dso::FrameHessian*> fhsToKeepPoints;
    std::vector<dso::FrameHessian*> fhsToMargPoints;

    for(int i=((int)this->frame_hessians.size())-1;i>=0 && i >= ((int)this->frame_hessians.size());i--)
        if(!this->frame_hessians[i]->flaggedForMarginalization) fhsToKeepPoints.push_back(this->frame_hessians[i]);

    for(int i=0; i< (int)this->frame_hessians.size();i++)
        if(this->frame_hessians[i]->flaggedForMarginalization) fhsToMargPoints.push_back(this->frame_hessians[i]);

    int flag_oob=0, flag_in=0, flag_inin=0, flag_nores=0;
    for(dso::FrameHessian* host : this->frame_hessians)
    {
        for(unsigned int i=0;i<host->pointHessians.size();i++)
        {
            dso::PointHessian* ph = host->pointHessians[i];
            if(ph==0) continue;

            if(ph->idepth_scaled < 0 || ph->residuals.size()==0)
            {
                host->pointHessiansOut.push_back(ph);
                ph->efPoint->stateFlag = dso::EFPointStatus::PS_DROP;
                host->pointHessians[i]=0;
                flag_nores++;
            }
            else if(ph->isOOB(fhsToKeepPoints, fhsToMargPoints) || host->flaggedForMarginalization)
            {
                flag_oob++;
                if(ph->isInlierNew())
                {
                    flag_in++;
                    int ngoodRes=0;
                    for(dso::PointFrameResidual* r : ph->residuals)
                    {
                        r->resetOOB();
                        r->linearize(this->calib.get());
                        r->efResidual->isLinearized = false;
                        r->applyRes(true);
                        if(r->efResidual->isActive())
                        {
                            r->efResidual->fixLinearizationF(this->bundles.get());
                            ngoodRes++;
                        }
                    }
                    if(ph->idepth_hessian > dso::setting_minIdepthH_marg)
                    {
                        flag_inin++;
                        ph->efPoint->stateFlag = dso::EFPointStatus::PS_MARGINALIZE;
                        host->pointHessiansMarginalized.push_back(ph);
                    }
                    else
                    {
                        ph->efPoint->stateFlag = dso::EFPointStatus::PS_DROP;
                        host->pointHessiansOut.push_back(ph);
                    }
                }
                else
                {
                    host->pointHessiansOut.push_back(ph);
                    ph->efPoint->stateFlag = dso::EFPointStatus::PS_DROP;
                }
                host->pointHessians[i]=0;
            }
        }

        for(int i=0;i<(int)host->pointHessians.size();i++)
        {
            if(host->pointHessians[i]==0)
            {
                host->pointHessians[i] = host->pointHessians.back();
                host->pointHessians.pop_back();
                i--;
            }
        }
    }
}

std::vector<dso::VecX> Task::getNullspaces(
    std::vector<dso::VecX> &nullspaces_pose,
    std::vector<dso::VecX> &nullspaces_scale,
    std::vector<dso::VecX> &nullspaces_affA,
    std::vector<dso::VecX> &nullspaces_affB)
{
    nullspaces_pose.clear();
    nullspaces_scale.clear();
    nullspaces_affA.clear();
    nullspaces_affB.clear();


    int n=CPARS+this->frame_hessians.size()*8;
    std::vector<dso::VecX> nullspaces_x0_pre;
    for(int i=0;i<6;i++)
    {
        dso::VecX nullspace_x0(n);
        nullspace_x0.setZero();
        for(dso::FrameHessian* fh : this->frame_hessians)
        {
            nullspace_x0.segment<6>(CPARS+fh->idx*8) = fh->nullspaces_pose.col(i);
            nullspace_x0.segment<3>(CPARS+fh->idx*8) *= SCALE_XI_TRANS_INVERSE;
            nullspace_x0.segment<3>(CPARS+fh->idx*8+3) *= SCALE_XI_ROT_INVERSE;
        }
        nullspaces_x0_pre.push_back(nullspace_x0);
        nullspaces_pose.push_back(nullspace_x0);
    }
    for(int i=0;i<2;i++)
    {
        dso::VecX nullspace_x0(n);
        nullspace_x0.setZero();
        for(dso::FrameHessian* fh : this->frame_hessians)
        {
            nullspace_x0.segment<2>(CPARS+fh->idx*8+6) = fh->nullspaces_affine.col(i).head<2>();
            nullspace_x0[CPARS+fh->idx*8+6] *= SCALE_A_INVERSE;
            nullspace_x0[CPARS+fh->idx*8+7] *= SCALE_B_INVERSE;
        }
        nullspaces_x0_pre.push_back(nullspace_x0);
        if(i==0) nullspaces_affA.push_back(nullspace_x0);
        if(i==1) nullspaces_affB.push_back(nullspace_x0);
    }

    dso::VecX nullspace_x0(n);
    nullspace_x0.setZero();
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        nullspace_x0.segment<6>(CPARS+fh->idx*8) = fh->nullspaces_scale;
        nullspace_x0.segment<3>(CPARS+fh->idx*8) *= SCALE_XI_TRANS_INVERSE;
        nullspace_x0.segment<3>(CPARS+fh->idx*8+3) *= SCALE_XI_ROT_INVERSE;
    }
    nullspaces_x0_pre.push_back(nullspace_x0);
    nullspaces_scale.push_back(nullspace_x0);

    return nullspaces_x0_pre;
}

void Task::makeNewTraces(dso::FrameHessian* newFrame, ::eds::mapping::IDepthMap2d *depthmap)
{
    /** Get the num points according to the map selection **/
    this->pixel_selector->allowFast = true;
    int numPointsTotal = this->pixel_selector->makeMaps(newFrame, this->selection_map, dso::setting_desiredImmatureDensity);

    /** Reserve memory **/
    newFrame->pointHessians.reserve(numPointsTotal*1.2f);
    newFrame->pointHessiansMarginalized.reserve(numPointsTotal*1.2f);
    newFrame->pointHessiansOut.reserve(numPointsTotal*1.2f);

    /** The global Map in the current keyframe. Help for better Immature Points initialization **/
    depthmap->fromPoints(this->getPoints(dso::SE3ToBaseTransform(newFrame->shell->camToWorld).inverse()), cv::Size(dso::wG[0], dso::hG[0]));

    /** Create a KD-Tree to search the initial value **/
    ::eds::mapping::KDTree<eds::mapping::Point2d> kdtree(depthmap->coord);

    /** The coordinates of the points in the new Keyframe **/
    std::vector<::eds::mapping::Point2d> coord;

    /** The initial inverse depth of the points in the new Keyframe **/
    std::vector<double> idp;
 
    for(int y=patternPadding+1;y<dso::hG[0]-patternPadding-2;y++)
    for(int x=patternPadding+1;x<dso::wG[0]-patternPadding-2;x++)
    {
        int i = x+y*dso::wG[0];
        if(this->selection_map[i]==0) continue;

        auto point = eds::mapping::Point2d(x, y); //Get the selected point coordinate

        /** Index of the closest points **/
        const int idx = kdtree.nnSearch(point);

        /** Distance: eucledian norm to the closest point in pixels **/
        cv::Point2d dist(depthmap->coord[idx].x() - point.x(), depthmap->coord[idx].y() - point.y());

        /** Create the immature point **/
        dso::ImmaturePoint* impt = new dso::ImmaturePoint(x, y, newFrame,
                                        this->selection_map[i],
                                        /*depthmap->idepth[idx],*/
                                        /*cv::norm(dist),*/
                                        this->calib.get(),
                                        &(this->img_rgb[0].at<unsigned char>(0)),
                                        &(this->img_rgb[1].at<unsigned char>(0)),
                                        &(this->img_rgb[2].at<unsigned char>(0))
                                       );

        if(!std::isfinite(impt->energyTH)) delete impt;
        else
        {
            /** Push the point coordinate and inverse depth value **/
            coord.push_back(point);
            idp.push_back(depthmap->idepth[idx]);
            newFrame->immaturePoints.push_back(impt);
        }
    }

    /** Re-create the depthmap with the selected immature points **/
    depthmap->clear();
    depthmap->coord = coord;
    depthmap->idepth = idp;

    //std::cout<<"MADE "<< (int)newFrame->immaturePoints.size() <<"IMMATURE POINTS!"<<std::endl;
}

void Task::solveSystem(int iteration, double lambda)
{
    this->bundles->lastNullspaces_forLogging = this->getNullspaces(
            this->bundles->lastNullspaces_pose,
            this->bundles->lastNullspaces_scale,
            this->bundles->lastNullspaces_affA,
            this->bundles->lastNullspaces_affB);

    this->bundles->solveSystemF(iteration, lambda, this->calib.get());
}

bool Task::doStepFromBackup(float stepfacC,float stepfacT,float stepfacR,float stepfacA,float stepfacD)
{
    dso::Vec10 pstepfac;
    pstepfac.segment<3>(0).setConstant(stepfacT);
    pstepfac.segment<3>(3).setConstant(stepfacR);
    pstepfac.segment<4>(6).setConstant(stepfacA);

    float sumA=0, sumB=0, sumT=0, sumR=0, sumID=0, numID=0;

    float sumNID=0;

    if(dso::setting_solverMode & SOLVER_MOMENTUM)
    {
        this->calib->setValue(this->calib->value_backup + this->calib->step);
        for(dso::FrameHessian* fh : this->frame_hessians)
        {
            dso::Vec10 step = fh->step;
            step.head<6>() += 0.5f*(fh->step_backup.head<6>());

            fh->setState(fh->state_backup + step);
            sumA += step[6]*step[6];
            sumB += step[7]*step[7];
            sumT += step.segment<3>(0).squaredNorm();
            sumR += step.segment<3>(3).squaredNorm();

            for(dso::PointHessian* ph : fh->pointHessians)
            {
                float step = ph->step+0.5f*(ph->step_backup);
                ph->setIdepth(ph->idepth_backup + step);
                sumID += step*step;
                sumNID += fabsf(ph->idepth_backup);
                numID++;

                ph->setIdepthZero(ph->idepth_backup + step);
            }
        }
    }
    else
    {
        this->calib->setValue(this->calib->value_backup + stepfacC*this->calib->step);
        for(dso::FrameHessian* fh : this->frame_hessians)
        {
            fh->setState(fh->state_backup + pstepfac.cwiseProduct(fh->step));
            sumA += fh->step[6]*fh->step[6];
            sumB += fh->step[7]*fh->step[7];
            sumT += fh->step.segment<3>(0).squaredNorm();
            sumR += fh->step.segment<3>(3).squaredNorm();

            for(dso::PointHessian* ph : fh->pointHessians)
            {
                ph->setIdepth(ph->idepth_backup + stepfacD*ph->step);
                sumID += ph->step*ph->step;
                sumNID += fabsf(ph->idepth_backup);
                numID++;

                ph->setIdepthZero(ph->idepth_backup + stepfacD*ph->step);
            }
        }
    }

    sumA /= this->frame_hessians.size();
    sumB /= this->frame_hessians.size();
    sumR /= this->frame_hessians.size();
    sumT /= this->frame_hessians.size();
    sumID /= numID;
    sumNID /= numID;

    if(!dso::setting_debugout_runquiet)
        printf("STEPS: A %.1f; B %.1f; R %.1f; T %.1f. \t",
                sqrtf(sumA) / (0.0005*dso::setting_thOptIterations),
                sqrtf(sumB) / (0.00005*dso::setting_thOptIterations),
                sqrtf(sumR) / (0.00005*dso::setting_thOptIterations),
                sqrtf(sumT)*sumNID / (0.00005*dso::setting_thOptIterations));

    dso::EFDeltaValid=false;
    this->setPrecalcValues();

    return sqrtf(sumA) < 0.0005*dso::setting_thOptIterations &&
            sqrtf(sumB) < 0.00005*dso::setting_thOptIterations &&
            sqrtf(sumR) < 0.00005*dso::setting_thOptIterations &&
            sqrtf(sumT)*sumNID < 0.00005*dso::setting_thOptIterations;
}

// sets linearization point.
void Task::backupState(bool backupLastStep)
{
	if(dso::setting_solverMode & SOLVER_MOMENTUM)
	{
		if(backupLastStep)
		{
			this->calib->step_backup = this->calib->step;
			this->calib->value_backup = this->calib->value;
			for(dso::FrameHessian* fh : this->frame_hessians)
			{
				fh->step_backup = fh->step;
				fh->state_backup = fh->get_state();
				for(dso::PointHessian* ph : fh->pointHessians)
				{
					ph->idepth_backup = ph->idepth;
					ph->step_backup = ph->step;
				}
			}
		}
		else
		{
			this->calib->step_backup.setZero();
			this->calib->value_backup = this->calib->value;
			for(dso::FrameHessian* fh : this->frame_hessians)
			{
				fh->step_backup.setZero();
				fh->state_backup = fh->get_state();
				for(dso::PointHessian* ph : fh->pointHessians)
				{
					ph->idepth_backup = ph->idepth;
					ph->step_backup=0;
				}
			}
		}
	}
	else
	{
		this->calib->value_backup = this->calib->value;
		for(dso::FrameHessian* fh : this->frame_hessians)
		{
			fh->state_backup = fh->get_state();
			for(dso::PointHessian* ph : fh->pointHessians)
				ph->idepth_backup = ph->idepth;
		}
	}
}

// sets linearization point.
void Task::loadSateBackup()
{
    this->calib->setValue(this->calib->value_backup);
    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        fh->setState(fh->state_backup);
        for(dso::PointHessian* ph : fh->pointHessians)
        {
            ph->setIdepth(ph->idepth_backup);

            ph->setIdepthZero(ph->idepth_backup);
        }
    }

    dso::EFDeltaValid=false;
    this->setPrecalcValues();
}

double Task::calcMEnergy()
{
    if(dso::setting_forceAceptStep) return 0;
    return this->bundles->calcMEnergyF();
}

double Task::calcLEnergy()
{
    if(dso::setting_forceAceptStep) return 0;

    double Ef = this->bundles->calcLEnergyF_MT();
    return Ef;
}

void Task::applyRes_Reductor(bool copyJacobians, int min, int max, dso::Vec10* stats, int tid)
{
    for(int k=min;k<max;k++)
        this->active_residuals[k]->applyRes(true);
}

void Task::linearizeAll_Reductor(bool fixLinearization, std::vector<dso::PointFrameResidual*>* toRemove, int min, int max, dso::Vec10* stats, int tid)
{
    for(int k=min;k<max;k++)
    {
        dso::PointFrameResidual* r = this->active_residuals[k];
        (*stats)[0] += r->linearize(this->calib.get());

        if(fixLinearization)
        {
            r->applyRes(true);

            if(r->efResidual->isActive())
            {
                if(r->isNew)
                {
                    dso::PointHessian* p = r->point;
                    dso::Vec3f ptp_inf = r->host->targetPrecalc[r->target->idx].PRE_KRKiTll * dso::Vec3f(p->u,p->v, 1);	// projected point assuming infinite depth.
                    dso::Vec3f ptp = ptp_inf + r->host->targetPrecalc[r->target->idx].PRE_KtTll*p->idepth_scaled;	// projected point with real depth.
                    float relBS = 0.01*((ptp_inf.head<2>() / ptp_inf[2])-(ptp.head<2>() / ptp[2])).norm();	// 0.01 = one pixel.


                    if(relBS > p->maxRelBaseline)
                        p->maxRelBaseline = relBS;

                    p->numGoodResiduals++;
                }
            }
            else
            {
                toRemove[tid].push_back(this->active_residuals[k]);
            }
        }
    }
}

void Task::setNewFrameEnergyTH()
{
    // collect all residuals and make decision on TH.
    this->all_res_vec.clear();
    this->all_res_vec.reserve(this->active_residuals.size()*2);
    dso::FrameHessian* newFrame = this->frame_hessians.back();

    for(dso::PointFrameResidual* r : this->active_residuals)
        if(r->state_NewEnergyWithOutlier >= 0 && r->target == newFrame)
        {
            this->all_res_vec.push_back(r->state_NewEnergyWithOutlier);

        }

    if(this->all_res_vec.size()==0)
    {
        newFrame->frameEnergyTH = 12*12*patternNum;
        return;		// should never happen, but lets make sure.
    }

    int nthIdx = dso::setting_frameEnergyTHN*this->all_res_vec.size();

    assert(nthIdx < (int)this->all_res_vec.size());
    assert(dso::setting_frameEnergyTHN < 1);

    std::nth_element(this->all_res_vec.begin(), this->all_res_vec.begin()+nthIdx, this->all_res_vec.end());
    float nthElement = sqrtf(this->all_res_vec[nthIdx]);

    newFrame->frameEnergyTH = nthElement*dso::setting_frameEnergyTHFacMedian;
    newFrame->frameEnergyTH = 26.0f*dso::setting_frameEnergyTHConstWeight + newFrame->frameEnergyTH*(1-dso::setting_frameEnergyTHConstWeight);
    newFrame->frameEnergyTH = newFrame->frameEnergyTH*newFrame->frameEnergyTH;
    newFrame->frameEnergyTH *= dso::setting_overallEnergyTHWeight*dso::setting_overallEnergyTHWeight;
}

dso::Vec3 Task::linearizeAll(bool fixLinearization)
{
    double lastEnergyP = 0;
    double lastEnergyR = 0;
    double num = 0;

    std::vector<dso::PointFrameResidual*> toRemove[NUM_THREADS];
    for(int i=0;i<NUM_THREADS;i++) toRemove[i].clear();

    if(dso::multiThreading)
    {
        this->thread_reduce.reduce(boost::bind(&Task::linearizeAll_Reductor, this, fixLinearization, toRemove, _1, _2, _3, _4), 0, this->active_residuals.size(), 0);
        lastEnergyP = this->thread_reduce.stats[0];
    }
    else
    {
        dso::Vec10 stats;
        this->linearizeAll_Reductor(fixLinearization, toRemove, 0,this->active_residuals.size(),&stats,0);
        lastEnergyP = stats[0];
    }

    this->setNewFrameEnergyTH();

    if(fixLinearization)
    {

        for(dso::PointFrameResidual* r : this->active_residuals)
        {
            dso::PointHessian* ph = r->point;
            if(ph->lastResiduals[0].first == r)
                ph->lastResiduals[0].second = r->state_state;
            else if(ph->lastResiduals[1].first == r)
                ph->lastResiduals[1].second = r->state_state;



        }

        int nResRemoved=0;
        for(int i=0;i<NUM_THREADS;i++)
        {
            for(dso::PointFrameResidual* r : toRemove[i])
            {
                dso::PointHessian* ph = r->point;

                if(ph->lastResiduals[0].first == r)
                    ph->lastResiduals[0].first=0;
                else if(ph->lastResiduals[1].first == r)
                    ph->lastResiduals[1].first=0;

                for(unsigned int k=0; k<ph->residuals.size();k++)
                    if(ph->residuals[k] == r)
                    {
                        this->bundles->dropResidual(r->efResidual);
                        this->deleteOut<dso::PointFrameResidual>(ph->residuals,k);
                        nResRemoved++;
                        break;
                    }
            }
        }
    }
    return dso::Vec3(lastEnergyP, lastEnergyR, num);
}

float Task::optimize(int mnumOptIts)
{
    if(this->frame_hessians.size() < 2) return 0;
    if(this->frame_hessians.size() < 3) mnumOptIts = 20;
    if(this->frame_hessians.size() < 4) mnumOptIts = 15;

    // get statistics and active residuals.
    this->active_residuals.clear();
    int numPoints = 0;
    int numLRes = 0;
    for(dso::FrameHessian* fh : this->frame_hessians)
        for(dso::PointHessian* ph : fh->pointHessians)
        {
            for(dso::PointFrameResidual* r : ph->residuals)
            {
                if(!r->efResidual->isLinearized)
                {
                    this->active_residuals.push_back(r);
                    r->resetOOB();
                }
                else
                    numLRes++;
            }
            numPoints++;
        }

    std::cout<<"[BUNDLES] OPTIMIZE "<< this->bundles->nPoints <<" pts, "<<(int)this->active_residuals.size()<<" active res, "<< numLRes<<" lin res!"<<std::endl;

    dso::Vec3 lastEnergy = this->linearizeAll(false);
    double lastEnergyL = this->calcLEnergy();
    double lastEnergyM = this->calcMEnergy();

    if(dso::multiThreading)
        this->thread_reduce.reduce(boost::bind(&Task::applyRes_Reductor, this, true, _1, _2, _3, _4), 0, this->active_residuals.size(), 50);
    else
        this->applyRes_Reductor(true,0,this->active_residuals.size(),0,0);

    if(!dso::setting_debugout_runquiet)
    {
        std::cout<<"Initial Error       \t";
        this->printOptRes(lastEnergy, lastEnergyL, lastEnergyM, 0, 0, this->frame_hessians.back()->aff_g2l().a, this->frame_hessians.back()->aff_g2l().b);
    }

    double lambda = 1e-1;
    float stepsize=1;
    dso::VecX previousX = dso::VecX::Constant(CPARS+ 8*this->frame_hessians.size(), NAN);
    for(int iteration=0;iteration<mnumOptIts;iteration++)
    {
        // solve!
        this->backupState(iteration!=0);
        this->solveSystem(iteration, lambda);
        double incDirChange = (1e-20 + previousX.dot(this->bundles->lastX)) / (1e-20 + previousX.norm() * this->bundles->lastX.norm());
        previousX = this->bundles->lastX;

        if(std::isfinite(incDirChange) && (dso::setting_solverMode & SOLVER_STEPMOMENTUM))
        {
            float newStepsize = exp(incDirChange*1.4);
            if(incDirChange<0 && stepsize>1) stepsize=1;

            stepsize = sqrtf(sqrtf(newStepsize*stepsize*stepsize*stepsize));
            if(stepsize > 2) stepsize=2;
            if(stepsize <0.25) stepsize=0.25;
        }

        bool canbreak = this->doStepFromBackup(stepsize,stepsize,stepsize,stepsize,stepsize);

        // eval new energy!
        dso::Vec3 newEnergy = this->linearizeAll(false);
        double newEnergyL = this->calcLEnergy();
        double newEnergyM = this->calcMEnergy();

        if(!dso::setting_debugout_runquiet)
        {
            printf("%s %d (L %.2f, dir %.2f, ss %.1f): \t",
                (newEnergy[0] +  newEnergy[1] +  newEnergyL + newEnergyM <
                        lastEnergy[0] + lastEnergy[1] + lastEnergyL + lastEnergyM) ? "ACCEPT" : "REJECT",
                iteration,
                log10(lambda),
                incDirChange,
                stepsize);
            this->printOptRes(newEnergy, newEnergyL, newEnergyM , 0, 0, this->frame_hessians.back()->aff_g2l().a, this->frame_hessians.back()->aff_g2l().b);
        }

        if(dso::setting_forceAceptStep || (newEnergy[0] +  newEnergy[1] +  newEnergyL + newEnergyM <
                lastEnergy[0] + lastEnergy[1] + lastEnergyL + lastEnergyM))
        {

            if(dso::multiThreading)
                this->thread_reduce.reduce(boost::bind(&Task::applyRes_Reductor, this, true, _1, _2, _3, _4), 0, this->active_residuals.size(), 50);
            else
                this->applyRes_Reductor(true,0,this->active_residuals.size(),0,0);

            lastEnergy = newEnergy;
            lastEnergyL = newEnergyL;
            lastEnergyM = newEnergyM;

            lambda *= 0.25;
        }
        else
        {
            this->loadSateBackup();
            lastEnergy = this->linearizeAll(false);
            lastEnergyL = this->calcLEnergy();
            lastEnergyM = this->calcMEnergy();
            lambda *= 1e2;
        }


        if(canbreak && iteration >= dso::setting_minOptIterations) break;
    }



    dso::Vec10 newStateZero = dso::Vec10::Zero();
    newStateZero.segment<2>(6) = this->frame_hessians.back()->get_state().segment<2>(6);

    this->frame_hessians.back()->setEvalPT(this->frame_hessians.back()->PRE_worldToCam,
            newStateZero);
    dso::EFDeltaValid=false;
    dso::EFAdjointsValid=false;
    this->bundles->setAdjointsF(this->calib.get());
    this->setPrecalcValues();

    lastEnergy = linearizeAll(true);

    if(!std::isfinite((double)lastEnergy[0]) || !std::isfinite((double)lastEnergy[1]) || !std::isfinite((double)lastEnergy[2]))
    {
        std::cout<<"KF Tracking failed: LOST!"<<std::endl;
        this->is_lost=true;
    }

    for(dso::FrameHessian* fh : this->frame_hessians)
    {
        fh->shell->camToWorld = fh->PRE_camToWorld;
        fh->shell->aff_g2l = fh->aff_g2l();
    }

    return sqrtf((float)(lastEnergy[0] / (patternNum*this->bundles->resInA)));
}

void Task::printOptRes(const dso::Vec3 &res, double resL, double resM, double resPrior, double LExact, float a, float b)
{
    printf("A(%f)=(AV %.3f). Num: A(%'d) + M(%'d); ab %f %f!\n",
            res[0],
            sqrtf((float)(res[0] / (patternNum*this->bundles->resInA))),
            this->bundles->resInA,
            this->bundles->resInM,
            a,
            b
    );
}

void Task::outputPoseKF(const dso::FrameHessian *current_kf, const base::Time &timestamp)
{
    /** It returns the world frame wrt to the camera frame: T_cam_world **/
    base::Transform3d T_cam_w = dso::SE3ToBaseTransform(current_kf->get_worldToCam_evalPT());

    /** Write output port: T_w_kf **/
    base::samples::RigidBodyState local_pose_w_kf;
    local_pose_w_kf.time = timestamp;
    local_pose_w_kf.targetFrame = "world";
    local_pose_w_kf.sourceFrame = "kf";
    local_pose_w_kf.setTransform(T_cam_w.inverse());
    //_pose_w_last_kf.write(local_pose_w_kf);

}
void Task::outputPoseKFs(const base::Time &timestamp)
{
    /** Write sliding window KF poses port: T_w_kf[i] **/
    eds::VectorKFs pose_kfs;
    pose_kfs.kfs.clear();
    pose_kfs.time = timestamp;

    /** Get all the poses **/
    for (auto fh : this->frame_hessians)
    {
        /** This returns T_cam_w **/
        base::Transform3d T_cam_w = dso::SE3ToBaseTransform(fh->get_worldToCam_evalPT());
        ::base::samples::RigidBodyState rbs;

        /** Get T_w_cam **/
        rbs.setTransform(T_cam_w.inverse());
        rbs.time = ::base::Time::fromSeconds(fh->shell->timestamp);
        rbs.targetFrame = "world";
        rbs.sourceFrame = std::to_string(fh->frameID);
        pose_kfs.kfs.push_back(rbs);
    }

    if (!pose_kfs.kfs.empty())
    {
        pose_kfs.time = pose_kfs.kfs[pose_kfs.kfs.size()-1].time;
        _pose_w_kfs.write(pose_kfs);
    }

    /** Also port out the current event frame pose **/
    _pose_w_ef.write(this->pose_w_ef);
}

void Task::outputTrackerInfo(const ::base::Time &timestamp)
{
    eds::TrackerInfo tracker_info= this->event_tracker->getInfo();
    tracker_info.time = timestamp;
    _tracker_info.write(tracker_info);
}

void Task::outputEventFrameViz(const std::shared_ptr<eds::tracking::EventFrame> &event_frame)
{
    /** Prepare output port image **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> event_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    event_img.reset(img);
    img = nullptr;

    /** Get the event frame visualization **/
    cv::Mat event_viz = event_frame->getEventFrameViz(0, false);
    //cv::Mat event_viz = this->kf->eventsOnKeyFrameViz(this->ef->frame[0]);
    //cv::Mat event_viz = this->ef->pyramidViz(false);
    /** Event frame with Epilines **/
    //cv::Mat event_viz = this->ef->epilinesViz(this->kf->coord, this->event_tracker->getFMatrix(), 100);

    /** Write min and max values on image **/
    double min = * std::min_element(std::begin(event_frame->event_frame[0]), std::end(event_frame->event_frame[0]));
    double max = * std::max_element(std::begin(event_frame->event_frame[0]), std::end(event_frame->event_frame[0]));
    std::string text = "min: " + std::to_string(min) + " max: " + std::to_string(max);
    cv::putText(event_viz, text, cv::Point(5, event_viz.rows-5), 
    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);

    /** Write in the output port **/
    ::base::samples::frame::Frame *event_img_ptr = event_img.write_access();
    event_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(event_viz, *event_img_ptr);
    event_img.reset(event_img_ptr);
    event_img_ptr->time = event_frame->time;
    _event_frame.write(event_img);

    /** Write the Event Frame Vector **/
    ::eds::EventFrameVector event_frame_vector;
    event_frame_vector.time = event_frame->time;
    event_frame_vector.data = event_frame->event_frame[0];
    _event_frame_vector.write(event_frame_vector);

    /** Prepare output port image **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> residuals_img;
    img = new ::base::samples::frame::Frame();
    residuals_img.reset(img);
    img = nullptr;

    /** Residuals **/
    cv::Mat residuals_viz = this->key_frame->residualsViz();

    /** Write min and max values on image **/
    min = * std::min_element(std::begin(this->key_frame->residuals), std::end(this->key_frame->residuals));
    max = * std::max_element(std::begin(this->key_frame->residuals), std::end(this->key_frame->residuals));
    text = "min: " + std::to_string(min) + " max: " + std::to_string(max);
    cv::putText(residuals_viz, text, cv::Point(5, residuals_viz.rows-5), 
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255), 0.1, cv::LINE_AA);

    /** Write in the output port **/
    ::base::samples::frame::Frame *residuals_img_ptr = residuals_img.write_access();
    frame_helper::FrameHelper::copyMatToFrame(residuals_viz, *residuals_img_ptr);
    residuals_img.reset(residuals_img_ptr);
    residuals_img_ptr->time = event_frame->time;
    _residuals_frame.write(residuals_img);
}

void Task::outputGenerativeModelFrameViz(const std::shared_ptr<::eds::tracking::KeyFrame> &keyframe, const ::base::Time &timestamp)
{
    /** Model image **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> model;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    model.reset(img);

    //std::vector<cv::Point2d> coord = this->event_tracker->getCoord();
    cv::Mat model_img = keyframe->getModel(this->event_tracker->linearVelocity(),
                                    this->event_tracker->angularVelocity(), "bilinear", 0.0);
    cv::Mat model_viz = keyframe->viz(model_img, false);

    /** Write min and max values on model image **/
    double min, max;
    cv::minMaxLoc(model_img, &min, &max);
    std::string text = "min: " + std::to_string(min) + " max: " + std::to_string(max);
    cv::putText(model_viz, text, cv::Point(5, model_viz.rows-5), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255), 0.1, cv::LINE_AA);

    ::base::samples::frame::Frame *model_ptr = model.write_access();
    model_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(model_viz, *model_ptr);
    model.reset(model_ptr);
    model_ptr->time = timestamp;
    _model_frame.write(model);

    /** Write Model frame in vector form **/
    eds::ModelFrameVector model_frame_vector;
    model_frame_vector.time = timestamp;
    for (int row=0; row<model_img.rows; row++)
    {
        for (int col=0; col<model_img.cols;col++)
        {
            model_frame_vector.data.push_back(model_img.at<double>(row,col));
        }
    }

    _model_frame_vector.write(model_frame_vector);
}

void Task::outputInvDepthAndLocalMap(const std::shared_ptr<::eds::tracking::KeyFrame> &keyframe, const ::base::Time &timestamp)
{
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> inv_depth_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    inv_depth_img.reset(img);
 
    /** Get the inverse depth map viz **/
    std::vector<double> idp = keyframe->inv_depth.getIDepth();
    cv::Mat inv_depth_viz = keyframe->idepthmapViz(keyframe->coord, idp, "nn", 0.0);

    /** Write min and max values on image **/
    double min = * std::min_element(std::begin(idp), std::end(idp));
    double max = * std::max_element(std::begin(idp), std::end(idp));
    std::string text = "min: " + std::to_string(min) + " max: " + std::to_string(max);
    cv::putText(inv_depth_viz, text, cv::Point(5, inv_depth_viz.rows-5), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,0), 0.1, cv::LINE_AA);

    /** Output inverse depth map **/
    ::base::samples::frame::Frame *inv_depth_img_ptr = inv_depth_img.write_access();
    inv_depth_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(inv_depth_viz, *inv_depth_img_ptr);
    inv_depth_img.reset(inv_depth_img_ptr);
    inv_depth_img_ptr->time = timestamp;
    _inv_depth_frame.write(inv_depth_img);

    /** Output the Local map with event-based colors **/
    std::vector<double> model = keyframe->getSparseModel(this->event_tracker->linearVelocity(), this->event_tracker->angularVelocity());
    this->outputLocalMap(keyframe, idp, model);
}

void Task::outputOpticalFlowFrameViz(const std::shared_ptr<::eds::tracking::KeyFrame> &keyframe, const ::base::Time &timestamp)
{
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> of_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    of_img.reset(img);

    /** Get the optical flow **/
    cv::Mat optical_flow = eds::utils::flowArrowsOnImage(keyframe->img, keyframe->coord,
                                                keyframe->tracks, cv::Vec3b(0.0, 255.0, 0.0) /*BGR*/, 10);

    /** Output the optical flow image **/
    ::base::samples::frame::Frame *of_img_ptr = of_img.write_access();
    of_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(optical_flow, *of_img_ptr);
    of_img.reset(of_img_ptr);
    of_img_ptr->time = timestamp;
    _of_frame.write(of_img);
}

void Task::outputGradientsFrameViz(const std::shared_ptr<::eds::tracking::KeyFrame> &keyframe, const ::base::Time &timestamp)
{
    /** Gradient along x-axis **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> grad_x_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    grad_x_img.reset(img);

    std::vector<cv::Point2d> coord = this->event_tracker->getCoord();
    cv::Mat grad_x_viz = keyframe->viz(keyframe->getGradient_x(coord, "bilinear"), false);

    ::base::samples::frame::Frame *grad_x_img_ptr = grad_x_img.write_access();
    grad_x_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(grad_x_viz, *grad_x_img_ptr);
    grad_x_img.reset(grad_x_img_ptr);
    grad_x_img_ptr->time = timestamp;
    //_grad_x_frame.write(grad_x_img);

    /** Gradient along y-axis **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> grad_y_img;
    img = new ::base::samples::frame::Frame();
    grad_y_img.reset(img);
 
    cv::Mat grad_y_viz = keyframe->viz(keyframe->getGradient_y(coord, "bilinear"), false);

    ::base::samples::frame::Frame *grad_y_img_ptr = grad_y_img.write_access();
    grad_y_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(grad_y_viz, *grad_y_img_ptr);
    grad_y_img.reset(grad_y_img_ptr);
    grad_y_img_ptr->time = keyframe->time;
    //_grad_y_frame.write(grad_y_img);

    /** Gradient magnitude **/
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> grad_mag;
    img = new ::base::samples::frame::Frame();
    grad_mag.reset(img);

    cv::Mat grad_mag_mat = keyframe->getGradientMagnitude(coord, "bilinear");
    cv::Mat grad_mag_viz = keyframe->viz(grad_mag_mat, true);

    /** Write min and max values on grad magnigute image **/
    double min, max;
    cv::minMaxLoc(grad_mag_mat, &min, &max);
    std::string text = "min: " + std::to_string(min) + " max: " + std::to_string(max);
    cv::putText(grad_mag_viz, text, cv::Point(5, grad_mag_viz.rows-5), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255), 0.1, cv::LINE_AA);

    ::base::samples::frame::Frame *grad_mag_ptr = grad_mag.write_access();
    grad_mag_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(grad_mag_viz, *grad_mag_ptr);
    grad_mag.reset(grad_mag_ptr);
    grad_mag_ptr->time = keyframe->time;
    //_mag_frame.write(grad_mag);// commented: so currently this is not port out anymore
}

void Task::outputLocalMap(const std::shared_ptr<::eds::tracking::KeyFrame> &keyframe, const std::vector<double> &idp, const std::vector<double> &model)
{
    /** Output the Local map **/
    ::base::samples::Pointcloud point_cloud;
    if (keyframe->norm_coord.size() == idp.size())
    {
        if (model.size() == 0)
        {
            point_cloud = keyframe->getMap(idp, model, ::eds::tracking::MAP_COLOR_MODE::RED);
        }
        else
        {
            point_cloud = keyframe->getMap(idp, model, ::eds::tracking::MAP_COLOR_MODE::EVENTS);
        }
    }
    _local_map.write(point_cloud);
}

void Task::outputKeyFrameMosaicViz(const std::vector<dso::FrameHessian*> &frame_hessians, const ::base::Time &timestamp)
{
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> mosaic_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    mosaic_img.reset(img);

    std::vector<cv::Mat> img_vector;
    for (auto &it : frame_hessians)
    {
        /** Convert FrameHessian to MinimalImage **/
        dso::MinimalImageB3* img_b3 = eds::io::toMinimalImageB3(it->dI, dso::wG[0], dso::hG[0]);

        /** Draw the inverse depth points **/
        for(dso::PointHessian* ph : it->pointHessians)
        {
            if(ph==0) continue;
            img_b3->setPixelCirc(ph->u, ph->v, dso::makeJet3B(ph->idepth_scaled));
        }

        /** First keyframe in full resolution **/
       if (it == frame_hessians[0])
            img_vector.push_back(cv::Mat(img_b3->h, img_b3->w, CV_8UC3, img_b3->data));

        /** Image in Opencv and resize image **/
        cv::Mat img_mat;
        cv::resize(cv::Mat(img_b3->h, img_b3->w, CV_8UC3, img_b3->data), img_mat, cv::Size(img_b3->w/2, img_b3->h/2), cv::INTER_CUBIC);

        /** Write the text **/
        const std::string text = "KF:"+std::to_string(it->frameID);
        cv::putText(img_mat, text, cv::Point(15, img_mat.rows-5), 
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0,255,0), 0.1, cv::LINE_AA);

        img_vector.push_back(img_mat);
    }

    /** Create the mosaic **/
    int n = std::ceil(dso::setting_maxFrames/2.0);
    cv::Mat mosaic = cv::Mat((n * dso::hG[0]/2 ) + dso::hG[0], dso::wG[0], CV_8UC3, cv::Scalar(0));

    /** Fill the mosaic **/
    img_vector[0].copyTo(mosaic(cv::Rect(0, 0, img_vector[0].cols, img_vector[0].rows)));
    for (size_t i=1; i<img_vector.size(); ++i)
    {
        img_vector[i].copyTo(mosaic(cv::Rect(((i-1)%2)*img_vector[i].cols,
                                    img_vector[0].rows + (((i-1)/2)*img_vector[i].rows),
                                    img_vector[i].cols, img_vector[i].rows)));
    }

    /** Output the mosaic of global map keyframes **/
    ::base::samples::frame::Frame *mosaic_img_ptr = mosaic_img.write_access();
    mosaic_img_ptr->image.clear();
    frame_helper::FrameHelper::copyMatToFrame(mosaic, *mosaic_img_ptr);
    mosaic_img.reset(mosaic_img_ptr);
    mosaic_img_ptr->time = timestamp;
    _keyframes_frame.write(mosaic_img);
}

void Task::outputImmaturePtsFrameViz(const dso::FrameHessian *input, const ::base::Time &timestamp)
{
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> pts_img;
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    pts_img.reset(img);

    /** Convert FrameHessian to MinimalImage **/
    dso::MinimalImageB3* img_b3 = eds::io::toMinimalImageB3(input->dI, dso::wG[0], dso::hG[0]);

    /** Draw the immature points **/
    for(dso::ImmaturePoint* ph : input->immaturePoints)
    {
        if(ph==0) continue;

        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_GOOD)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(0,255,0));//GREEN
        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_OOB)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(0,0,255));//RED
        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_OUTLIER)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(255,0,0));//BLUE
        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_SKIPPED)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(255,255,0));//CYAN
        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_BADCONDITION)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(255,255,255)); //WHITE
        if(ph->lastTraceStatus==dso::ImmaturePointStatus::IPS_UNINITIALIZED)
            img_b3->setPixel9(ph->u+0.5f, ph->v+0.5f, dso::Vec3b(0,0,0));//BLACK
    }

    cv::Mat img_mat =  cv::Mat(img_b3->h, img_b3->w, CV_8UC3, img_b3->data);

    if (!this->initialized)
    {
        const std::string text = "INITIALIZING...";
        cv::putText(img_mat, text, cv::Point(10, img_mat.rows/2), 
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);
    }

    /** Output inverse depth image **/
    ::base::samples::frame::Frame *pts_img_ptr = pts_img.write_access();
    eds::io::MinimalImageB3ToFrame(img_b3, timestamp, *pts_img_ptr);
    pts_img.reset(pts_img_ptr);
    _inv_depth_frame.write(pts_img);
    delete img_b3;
}

void Task::outputDepthMap(const std::shared_ptr<::eds::mapping::IDepthMap2d> &depthmap, const ::base::Time &timestamp)
{
    if (depthmap->empty()) {return;}
    base::Transform3d T_w_kf = this->pose_w_kf.getTransform();
    /** Points in the KF coordinate **/
    base::samples::Pointcloud depthmap_pcl; depthmap->toPointCloud(depthmap_pcl);
    /** Points in the World coordinate **/
    for (auto &it:depthmap_pcl.points) {it = T_w_kf * it;}
    depthmap_pcl.time = timestamp; //current timestamp
    _local_map.write(depthmap_pcl);
}

void Task::outputGlobalMap()
{
    /** Get the Global Map **/
    //base::samples::Pointcloud global_map = this->getMap(false);
    this->getMap(this->global_map, false, false/*color*/);

    /** Build the output global map **/
    base::samples::Pointcloud out_global_map;
    for (auto &m : this->global_map)
    {
        for (size_t i=0; i<m.second.points.size(); ++i)
        {
            out_global_map.points.push_back(m.second.points[i]);
            out_global_map.colors.push_back(m.second.colors[i]);
        }
    }
    int num_kfs = this->frame_hessians.size();
    dso::FrameHessian *last_fh = this->frame_hessians[num_kfs-1];
    out_global_map.time = ::base::Time::fromSeconds(last_fh->shell->timestamp);
    _global_map.write(out_global_map);
}

base::samples::Pointcloud Task::getMap(const bool &single_point, const bool &color)
{

    /** Instrinsics **/
    float fx = this->calib->fxl();
    float fy = this->calib->fyl();
    float cx = this->calib->cxl();
    float cy = this->calib->cyl();

    base::samples::Pointcloud pcl;

    /** For all the keyframes **/
    for (auto fh : this->frame_hessians)
    {
        /** Get KF transform **/
        base::Transform3d T_w_kf = dso::SE3ToBaseTransform(fh->get_worldToCam_evalPT()).inverse();

        /** For all the points in the keyframe **/
        for (auto p : fh->pointHessians)
        {
            if (p->idepth_scaled <= 0)
                continue;

            double d_i = 1.0/p->idepth_scaled;
            /** Push points in the world frame coordinate i **/
            pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-cy)/fy, d_i));//u-v point8

            /** Get the color **/
            float *red, *green, *blue;
            if (color)
            { red = &(p->red[0]); green = &(p->green[0]); blue = &(p->blue[0]); }
            else
            { red = &(p->color[0]); green = &(p->color[0]); blue = &(p->color[0]); }

            /** Push the color**/
            pcl.colors.push_back(::base::Vector4d(red[0]/255.0, green[0]/255.0, blue[0]/255.0, 1.0));//point color RGB

            if (!single_point)
            {
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v+2-cy)/fy, d_i));//x-y point1
                pcl.colors.push_back(::base::Vector4d(red[1]/255.0, green[1]/255.0, blue[1]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v+1-cy)/fy, d_i ));//x-y point2
                pcl.colors.push_back(::base::Vector4d(red[2]/255.0, green[2]/255.0, blue[2]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point3
                pcl.colors.push_back(::base::Vector4d(red[3]/255.0, green[3]/255.0, blue[3]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point4
                pcl.colors.push_back(::base::Vector4d(red[4]/255.0, green[4]/255.0, blue[4]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-2-cy)/fy, d_i ));//x-y point5
                pcl.colors.push_back(::base::Vector4d(red[5]/255.0, green[5]/255.0, blue[5]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u+1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point6
                pcl.colors.push_back(::base::Vector4d(red[6]/255.0, green[6]/255.0, blue[6]/255.0, 1.0));//point color RGB
                pcl.points.push_back(T_w_kf * ::base::Point(d_i*(p->u+2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point7
                pcl.colors.push_back(::base::Vector4d(red[7]/255.0, green[7]/255.0, blue[7]/255.0, 1.0));//point color RGB
           }
        }
    }

    return pcl;
}

void Task::getMap(std::map<int, base::samples::Pointcloud> &global_map, const bool &single_point, const bool &color)
{
    /** Instrinsics **/
    float fx = this->calib->fxl();
    float fy = this->calib->fyl();
    float cx = this->calib->cxl();
    float cy = this->calib->cyl();

    /** For all the keyframes **/
    for (auto fh : this->frame_hessians)
    {
        /** Get KF transform **/
        base::Transform3d T_w_kf = dso::SE3ToBaseTransform(fh->get_worldToCam_evalPT()).inverse();

        /** Create a new element in the map **/
        if (global_map.find(fh->frameID) == global_map.end())
        {
            global_map.emplace(std::make_pair(fh->frameID, base::samples::Pointcloud()));
        }

        /** Reset the points of the KF which are active **/
        global_map[fh->frameID].colors.clear();
        global_map[fh->frameID].points.clear();

        /** For all the points in the keyframe **/
        for (auto p : fh->pointHessians)
        {
            if (p->maxRelBaseline < this->eds_config.mapping.points_rel_baseline)
                continue;

            if (p->idepth_scaled <= 0)
                continue;

            double d_i = 1.0/p->idepth_scaled;
            /** Push points in the world frame coordinate i **/
            global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-cy)/fy, d_i));//u-v point8

            /** Get the color **/
            float *red, *green, *blue;
            if (color)
            { red = &(p->red[0]); green = &(p->green[0]); blue = &(p->blue[0]); }
            else
            { red = &(p->color[0]); green = &(p->color[0]); blue = &(p->color[0]); }

            /** Push the color**/
            global_map[fh->frameID].colors.push_back(::base::Vector4d(red[0]/255.0, green[0]/255.0, blue[0]/255.0, 1.0));//point color RGB

            if (!single_point)
            {
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v+2-cy)/fy, d_i));//x-y point1
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[1]/255.0, green[1]/255.0, blue[1]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v+1-cy)/fy, d_i ));//x-y point2
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[2]/255.0, green[2]/255.0, blue[2]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point3
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[3]/255.0, green[3]/255.0, blue[3]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point4
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[4]/255.0, green[4]/255.0, blue[4]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u-cx)/fx, d_i*(p->v-2-cy)/fy, d_i ));//x-y point5
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[5]/255.0, green[5]/255.0, blue[5]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u+1-cx)/fx, d_i*(p->v-1-cy)/fy, d_i ));//x-y point6
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[6]/255.0, green[6]/255.0, blue[6]/255.0, 1.0));//point color RGB
                global_map[fh->frameID].points.push_back(T_w_kf * ::base::Point(d_i*(p->u+2-cx)/fx, d_i*(p->v-cy)/fy, d_i ));//x-y point7
                global_map[fh->frameID].colors.push_back(::base::Vector4d(red[7]/255.0, green[7]/255.0, blue[7]/255.0, 1.0));//point color RGB
           }
        }
    }
}
void Task::printResult(std::string file)
{
	std::ofstream myfile;
	myfile.open (file.c_str());
	myfile << std::setprecision(15);

	for(dso::FrameShell* s : this->all_frame_history)
	{
		if(!s->poseValid) continue;

		if(dso::setting_onlyLogKFPoses && s->marginalizedAt == s->id) continue;

		myfile << s->timestamp <<
			" " << s->camToWorld.translation().transpose()<<
			" " << s->camToWorld.so3().unit_quaternion().x()<<
			" " << s->camToWorld.so3().unit_quaternion().y()<<
			" " << s->camToWorld.so3().unit_quaternion().z()<<
			" " << s->camToWorld.so3().unit_quaternion().w() << "\n";
	}
	myfile.close();
}