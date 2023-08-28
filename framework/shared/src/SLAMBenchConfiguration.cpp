/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfiguration.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"

#include <io/FrameBufferSource.h>
#include <io/openni2/ONI2FrameStream.h>
#include <io/openni2/ONI2InputInterface.h>
#include <io/openni15/ONI15FrameStream.h>
#include <io/openni15/ONI15InputInterface.h>

#include <io/InputInterface.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>

#include <outputs/TrajectoryAlignmentMethod.h>
#include <outputs/OutputManagerWriter.h>
#include <metrics/DurationMetric.h>
#include <metrics/PowerMetric.h>
#include <metrics/Metric.h>
#include <metrics/MemoryMetric.h>
#include <metrics/ATEMetric.h>
#include <metrics/RPEMetric.h>
#include <metrics/DepthEstimationMetric.h>

#include <values/Value.h>
#include <outputs/Output.h>

#include <stdexcept>
#include <map>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <memory>
#include <assert.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "ResultWriter.h"
#include <io/Event.h>

void SLAMBenchConfiguration::LoadEvents(){
   // not how you use gt buffer as a normal buffer. 
   auto buffer = new slambench::io::GTBufferingFrameStream(input_interface_manager_->GetCurrentInputInterface()->GetFrames());
   int count =0;
   while(buffer->HasNextFrame()){
        auto cur_frame = buffer->GetNextFrame();
        if(cur_frame==nullptr)
            break;
        // std::cout<<"Managed to retrieve frame number "<<count;
        // std::cout<<cur_frame<<' ';
        if(cur_frame->FrameSensor->IsEvent()){
            int elements  = cur_frame->GetVariableSize()/sizeof(slambench::io::Event);
            slambench::io::Event * events = static_cast<slambench::io::Event*>(cur_frame->GetData());
            for(int i=0; i<elements; i++){
                event_stream_.push_back(std::move(events[i]));
            }
            // save the sensor that was used
            event_idf_ = cur_frame->FrameSensor;
            cur_frame->FreeData();
            
            break;
        }
        // std::cout<<" - not event"<<'\n';
        count++;
   }

    std::cout<<event_stream_.size()<<'\n';
    // delete buffer;
  
}


// some event based algorithms will use dynamic event frame assignment. 
// for those we split the frames as they run. Otherwise frame split is done ar init time

slambench::io::SLAMFrame * SLAMBenchConfiguration::pickEventFrame(SLAMBenchLibraryHelper * lib){
    // size_t current_index = 0, i;
    // auto current_ts = events[current_index].ts;
    // // loop runs once per SLAM Frame
    // while(current_index < events.size() - 1) {

    //     auto event_frame = new SLAMInMemoryFrame();
    //     event_frame->FrameSensor = event_sensor;
    //     event_frame->Timestamp = current_ts;

    //     // loop from current position until frame found with time difference greater than framerate
    //     for(i = current_index; i < events.size(); ++i) {
    //         auto delta = events[i].ts - current_ts;
    //         if (delta > std::chrono::milliseconds{20}) break;
    //     }

    //     size_t count = (i - 1) - current_index;
    //     size_t variable_size = sizeof(Event) * count;

    //     // copy from and to points into malloc'd memory
    //     event_frame->SetVariableSize(variable_size);
    //     event_frame->Data = malloc(variable_size);
    //     memcpy(event_frame->Data, &events[current_index], variable_size);

    //     file.AddFrame(event_frame);

    //     // set index to beginning of next frame
    //     current_index = i;
    //     current_ts = events[current_index].ts;
    // }

    // gets the range from the lock and makes the frame based on it 
    // lock the mutex in order to read the frames
    std::lock_guard<std::mutex> lock(lib->GetMutex());
    int begin_idx = lib->GetBeginIndex();
    int end_idx = lib->GetEndIndex();
    if(begin_idx==-1000 && end_idx==-1000){
        return nullptr;
    }
    auto current_ts = event_stream_[begin_idx].ts;
    size_t events_size = (end_idx - begin_idx)*sizeof(slambench::io::Event);
    auto event_frame = new slambench::io::SLAMEventFrame();
    event_frame->FrameSensor = event_idf_;
    event_frame->Timestamp = current_ts;
    event_frame->indices = new std::pair<int,int>(begin_idx, end_idx);
    // std::cout<<"Created frame with "<<event_frame->indices->first<<' '<<event_frame->indices->second<<' '<<event_frame->Timestamp<<'\n';
    return event_frame;
}

SLAMBenchConfiguration::SLAMBenchConfiguration(void (*custom_input_callback)(Parameter*, ParameterComponent*),void (*libs_callback)(Parameter*, ParameterComponent*)) :
        ParameterComponent("") {

    if(!custom_input_callback)
        custom_input_callback = input_callback;
    if(!libs_callback)
        libs_callback = slam_library_callback;
    initialised_ = false;
    log_stream_ = nullptr;
    slam_library_names_ = {};
    // Run Related
    addParameter(TypedParameter<unsigned int>("fl", "frame-limit", "last frame to compute", &frame_limit_, &default_frame_limit));
    addParameter(TypedParameter<unsigned int>("s", "start-frame", "first frame to compute", &start_frame_, &default_start_frame));
    addParameter(TypedParameter<std::string>("o", "log-file", "Output log file", &log_file_, &default_log_file, log_callback));
    addParameter(TypedParameter<std::vector<std::string>>("i", "input" , "Specify the input file or mode." , &input_files_, &default_input_files , custom_input_callback));
    addParameter(TypedParameter<std::vector<std::string> >("load", "load-slam-library" , "Load a specific SLAM library."     , &slam_library_names_, &default_slam_libraries, libs_callback));
    addParameter(TriggeredParameter("dse",   "dse",    "Output solution space of parameters.",    dse_callback));
    addParameter(TriggeredParameter("h",     "help",   "Print the help.", help_callback));
    addParameter(TypedParameter<bool>("realtime",     "realtime-mode",      "realtime frame loading mode",                   &realtime_mode_, &default_is_false));
    addParameter(TypedParameter<double>("realtime-mult",     "realtime-multiplier",      "realtime frame loading mode",                   &realtime_mult_, &default_realtime_mult));

    param_manager_.AddComponent(this);
}

SLAMBenchConfiguration::~SLAMBenchConfiguration() {
    //Clean algorithms
    for (auto lib : slam_libs_) {
        std::cerr << "Cleaning " << lib->getName() << std::endl;
        if (!lib->c_sb_clean_slam_system()) {
            std::cerr << "Algorithm cleaning failed." << std::endl;
            exit(1);
        } else {
            std::cerr << "Algorithm cleaning succeed." << std::endl;
        }
    }
}

void SLAMBenchConfiguration::AddSLAMLibrary(const std::string& so_file, const std::string &id) {

    std::cerr << "new library name: " << so_file  << std::endl;

    void* handle = dlopen(so_file.c_str(),RTLD_LAZY);

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << std::endl;
        exit(1);
    }
    auto libname_start = so_file.find_last_of('/')+3;
    auto libName = so_file.substr(libname_start, so_file.find('-', libname_start));
    auto lib_ptr = new SLAMBenchLibraryHelper(id, libName, this->GetLogStream(), input_interface_manager_->GetCurrentInputInterface());
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_slam_configuration);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_frame);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_once);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_clean_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_outputs);
    // workaround to be compatible with benchmarks that does not implement the relocalize API
    if (dlsym(handle, "_Z13sb_relocalizeP22SLAMBenchLibraryHelper")) {
        LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_relocalize);
    } else {
        std::cout << "Benchmark does not implement sb_relocalize(). Will use the default." << std::endl;
        lib_ptr->c_sb_relocalize = lib_ptr->c_sb_process_once;
    }
    slam_libs_.push_back(lib_ptr);


    size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    if (!lib_ptr->c_sb_new_slam_configuration(lib_ptr)) {
        std::cerr << "Configuration construction failed." << std::endl;
        exit(1);
    }
    size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    std::cerr << "Configuration consumed " << post-pre  << " bytes" << std::endl;
    param_manager_.AddComponent(lib_ptr);

    std::cerr << "SLAM library loaded: " << so_file << std::endl;
}

void SLAMBenchConfiguration::InitGroundtruth(bool with_point_cloud) {
    if(initialised_)
        return;
    // LoadEvents();
    auto interface = input_interface_manager_->GetCurrentInputInterface();
    if(interface != nullptr) {
        std::cout<<typeid(interface->GetFrames()).name()<<'\n';
        auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(interface->GetFrames());
        input_interface_manager_->input_stream_ = gt_buffering_stream;
        
        if(realtime_mode_) {
            std::cerr << "Real time mode enabled" << std::endl;
            input_interface_manager_->input_stream_ = new slambench::io::RealTimeFrameStream(input_interface_manager_->input_stream_, realtime_mult_, true);
        } else {
            std::cerr << "Process every frame mode enabled" << std::endl;
        }

        ground_truth_.LoadGTOutputsFromSLAMFile(interface->GetSensors(), gt_buffering_stream->GetGTFrames(), with_point_cloud);
    }

    auto gt_trajectory = ground_truth_.GetMainOutput(slambench::values::VT_POSE);
    if(gt_trajectory == nullptr) {
        // Warn if there is no ground truth
        gt_available_ = false;
        std::cerr << "Dataset does not provide a GT trajectory" << std::endl;
    }
    else {
        gt_available_ = true;
    }

    // LoadEvents();
    // ----------------- [LOAD EVENTS ALTERNATIVE] -----------------
    // first frame should always be event frame
    
    auto current_frame = input_interface_manager_->GetNextFrame();

    if(current_frame->FrameSensor->IsEvent()){
        
        std::cout<<"it was the first frame\n";
        int elements  = current_frame->GetVariableSize()/sizeof(slambench::io::Event);
        slambench::io::Event * events = static_cast<slambench::io::Event*>(current_frame->GetData());
        for(int i=0; i<elements; i++){
            event_stream_.push_back(std::move(events[i]));
        }
        // save the sensor that was used
        event_idf_ = current_frame->FrameSensor;
        current_frame->FreeData();
        delete current_frame;
        // get next one
        std::cout<<"event no 5 is "<<event_stream_[4]<<'\n';
        for (auto &lib : slam_libs_){
            lib->events_=&event_stream_;   
        }
    }   
    // ---------------------------------------------------------------
        
    initialised_ = true;
}

void SLAMBenchConfiguration::InitAlgorithms() {

    // load events
    std::cout<<"Don't make me load those events, i warn ya!!!\n\n";
    assert(initialised_);
    // set envents acess where necessary
    for (auto &lib : slam_libs_) {

        // lib->events_=&event_stream_;
        lib->GetMetricManager().BeginInit();
        bool init_worked = lib->c_sb_init_slam_system(lib);
        lib->GetMetricManager().EndInit();

        if (!init_worked) {
            std::cerr << "Algorithm initialization failed." << std::endl;
            exit(1);
        }

        auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if(trajectory == nullptr) {
            std::cerr << "Algo does not provide a main pose output" << std::endl;
            exit(1);
        }
    }
    std::cout<<"----------------------------------- ALG -----------------------------------\n";
}

void SLAMBenchConfiguration::InitAlignment() {
    std::cout<<"Try alignment\n";
    if(!gt_available_)
        return;
    slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
    if(alignment_technique_ == "original") {
        alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "new") {
        alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "umeyama") {
        alignment_method = new slambench::outputs::UmeyamaTrajectoryAlignmentMethod();
    } else {
        std::cerr << "Unknown alignment method " << alignment_technique_ << std::endl;
        throw std::logic_error("Unknown alignment method");
    }

    auto gt_traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    alignments_.clear();
    for(size_t i = 0; i < slam_libs_.size(); i++) {
        SLAMBenchLibraryHelper *lib = slam_libs_[i];
        auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        auto alignment = new slambench::outputs::AlignmentOutput(lib->getName() + "Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(
                                                                                                                           gt_traj), lib_traj, alignment_method);
        alignment->SetActive(true);
        alignment->SetKeepOnlyMostRecent(true);
        alignments_.push_back(alignment);
        auto aligned = new slambench::outputs::AlignedPoseOutput(lib_traj->GetName() + " (Aligned)", alignments_[i], lib_traj);
        lib->GetOutputManager().RegisterOutput(aligned);
        //Align point cloud
        auto pointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD);
        if(pointcloud != nullptr) {
            auto pc_aligned = new slambench::outputs::AlignedPointCloudOutput(lib->getName() + pointcloud->GetName() + "(Aligned)", alignment, pointcloud);
            lib->GetOutputManager().RegisterOutput(pc_aligned);
        }
    }
    std::cout<<"Alignment done\n";
}

void SLAMBenchConfiguration::ComputeLoopAlgorithm(bool *stay_on, SLAMBenchUI *ui) {

    std::cout<<"PoseValue takes "<<sizeof(slambench::values::PoseValue)<<"-----\n\n";
    assert(initialised_);

    int input_seq = 0;
    bool ongoing = false;
    // having the flag set here will only make it work when only one algorithm is loaded. 
    // any more than one and the entire thing will break in a million pieces
    // will get changed eventually when i figure out how 
    bool event_picked;
    std::vector<Eigen::Matrix4f> libs_trans;
    // ********* [[ MAIN LOOP ]] *********
    unsigned int frame_count = 0;
    while(true) {
        if (frame_count != 0 && ui && !ongoing) {
            if (!(ui->WaitForFrame() || !ui->IsFreeRunning())) {
                std::cerr << "!ui->WaitForFrame() ==> break" << std::endl;
                break;
            }
        }
        slambench::io::SLAMFrame * current_frame, * event_frame=nullptr;
        // if it's an event algorithm everything should be handled differently

        auto non_event_frame = input_interface_manager_->GetNextFrame();
        const std::type_info& typeInfo = typeid(*non_event_frame);
        std::cout << "Object type: " << typeInfo.name() << std::endl;
        // non_event_frame->GetData();
        bool available_events = true;
        // while there are non_event_frames or event frames
        while (non_event_frame != nullptr || available_events) {
            event_picked = false;
            frame_count++;
            
            // if the non event frame is not gt then compare it ith event to see which one 
            // is used next
            if (non_event_frame->FrameSensor->GetType() != slambench::io::GroundTruthSensor::kGroundTruthTrajectoryType ) {
                // std::cout<<frame_count<<'\n';
                // ********* [[ NEW FRAME PROCESSED BY ALGO ]] *********
                for (size_t i = 0; i < slam_libs_.size(); i++) {
                    auto lib = slam_libs_[i];
                    // ********* [[ EVENT FRAME SELECTION]] *********
                    current_frame = non_event_frame;
                    if(lib->IsEventAlg()){
                        event_frame = pickEventFrame(lib);
                        // std::cout<<non_event_frame->Timestamp<<' '<<event_frame->Timestamp<<' ';
                        if(event_frame!=nullptr){ 
                            if(non_event_frame->Timestamp>event_frame->Timestamp){
                                current_frame = event_frame;
                                event_picked = true;
                            }
                            else{
                                event_frame->FreeData();
                                delete event_frame;
                            }
                        }else{
                            // if there are no more events 
                            available_events=false;
                        }
                    }else{
                        // if not an event based algorithm then don't wait for events
                        available_events = false;
                    }
                    // ********* [[ SEND THE FRAME ]] *********
                    ongoing = not lib->c_sb_update_frame(lib, current_frame);

                    // This algorithm hasn't received enough frames yet.
                    if (ongoing) {
                        continue;
                    }

                    // ********* [[ PROCESS ALGO START ]] *********
                    lib->GetMetricManager().BeginFrame();
                    slambench::TimeStamp ts = current_frame->Timestamp;

                    // std::cout<<"input_interface_manager_->updated_ = "<<input_interface_manager_->updated_<<'\n';
                    if (!input_interface_manager_->updated_) {
                        if (not lib->c_sb_process_once(lib)) {
                            std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
                            exit(1);
                        }
                    } else {
                        // ********** [[or relocalization]] **********
                        //Mihai: need assertion / safety mechanism to avoid ugly errors
                        bool res = lib->c_sb_relocalize(lib);
                        std::cout<<"Relocalise: "<<res<<'\n';
                        input_interface_manager_->updated_ = false;
                        frame_count = 0;
                        /* If the library failed to re-localize at the beginning of a new input,
                           the framework will send a ground-truth pose to it (so-called aided_reloc).
                           The sent pose is transformed to be relative to the first estimated pose
                           from the library. */
                        if(!res && gt_available_)
                        {
                            aided_reloc_ = true;
                            //Find the nearest one
                            auto gt_frame = input_interface_manager_->GetClosestGTFrameToTime(ts);
                            Eigen::Matrix4f &t = alignments_[i]->getTransformation();
                            Eigen::Matrix4f gt;
                            memcpy(gt.data(), gt_frame->GetData(), gt_frame->GetSize());

                            slambench::io::FrameBuffer* buffer = &(dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer());
                            buffer->resetLock();
                            Eigen::Matrix4f es = t.inverse() * gt;
                            memcpy(gt_frame->GetData(), es.data(), gt_frame->GetSize());
                            buffer->resetLock();

                            lib->c_sb_update_frame(lib, gt_frame);// groundtruth feed

                            buffer->resetLock();
                            memcpy(gt_frame->GetData(), gt.data(), gt_frame->GetSize());
                            buffer->resetLock();
                        }
                    }

                    if (!lib->c_sb_update_outputs(lib, &ts)) {
                        std::cerr << "Failed to get outputs" << std::endl;
                        exit(1);
                    }
                    // std::cout<<"metric manager\n";
                    lib->GetMetricManager().EndFrame();
                    // std::cout<<"Metric manager done";
                }
                // ********* [[ FINALIZE ]] *********
                if (!ongoing) {
                    FireEndOfFrame();
                    if (ui) ui->stepFrame();
                    frame_count += 1;

                    if (frame_limit_ > 0 && frame_count >= frame_limit_) {
                            break;
                    }
                }
            }
            
            // if(event_frame!=nullptr){
            //     event_frame->FreeData();
            // }
            
            // won't work. I can't choose the next event frame until the algorithm is executed 
            if(!event_picked){
                // only free and delete is a new one is coming
                non_event_frame->FreeData();
                delete non_event_frame;
                non_event_frame = input_interface_manager_->GetNextFrame();
            }else{
                event_frame->FreeData();
                delete event_frame;
            }
            
        } // we're done with the frame
        if (!output_filename_.empty())
            SaveResults();
        // Freeze the alignment after end of the first input
        if (input_seq++ == 0)
            for(auto& alignment : alignments_)
                alignment->SetFreeze(true);
        
        // if no more inputs to try exit computation loop
        if (!LoadNextInputInterface())
            break;
    }
}

bool SLAMBenchConfiguration::LoadNextInputInterface() {
    if(!input_interface_manager_->LoadNextInputInterface())
        return false;

    initialised_ = false;
    ResetSensors();
    InitGroundtruth();
    InitAlignment();
    InitWriter();
    for (auto lib : this->slam_libs_) {
        lib->update_input_interface(input_interface_manager_->GetCurrentInputInterface());
    }
    current_input_id_++;

    return true;
}

//saves results in TUM format TODO: move out of SLAMBenchConfiguration
void SLAMBenchConfiguration::SaveResults()
{
    if (output_filename_.empty() ) {
        return;
    }
    boost::filesystem::path input_name(input_filenames_[current_input_id_]);
    input_name = input_name.filename();
    boost::filesystem::path output_name = boost::filesystem::path(output_filename_);
    boost::filesystem::path output_prefix;
    boost::filesystem::path gt_dir;
    if (boost::filesystem::is_directory(output_name)) {
        output_name.append("/");
        output_prefix = output_name;
        gt_dir = output_name;
    } else {
        output_prefix = output_name.replace_extension().append("-");
        gt_dir = output_name.parent_path();
    }
    boost::filesystem::path gt_file = gt_dir;
    gt_file += input_name;
    gt_file.replace_extension(".gt");

    bool first_input = current_input_id_ == 0;
    static std::string cpu_info = ResultWriter::GetCPUModel();
    static std::string gpu_info = "";// memory_metric->cuda_monitor.IsActive() ? memory_metric->cuda_monitor.device_name : "";
    static std::string mem_info = ResultWriter::GetMemorySize();

    for(SLAMBenchLibraryHelper *lib : slam_libs_) {
        std::string filename = output_prefix.append(lib->GetLibraryName() + ".txt").string();
        std::ofstream out(filename, first_input ? std::ios::out : std::ios::app);
        ResultWriter writer(out);
        if (first_input) {
            writer.WriteKV("benchmark", lib->GetLibraryName());
            writer.WriteKV("inputs", input_filenames_);
            writer.WriteKV("CPU", cpu_info);
            if (!gpu_info.empty()) writer.WriteKV("GPU", gpu_info);
            writer.WriteKV("memory", mem_info);
        }
        out << std::endl;
        writer.WriteKV("input", input_name.string());
        writer.WriteKV("aided_reloc", std::to_string(aided_reloc_));
        slambench::outputs::BaseOutput::value_map_t traj = lib->GetOutputManager().GetOutput("Pose")->GetValues();
        writer.WriteTrajectory(traj);
        std::cout << "Results saved into " << filename << std::endl;
    }

    if (!boost::filesystem::exists(gt_file)) {
        std::ofstream out(gt_file.string());
        slambench::outputs::BaseOutput::value_map_t traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE)->GetValues();
        ResultWriter writer(out);
        writer.WriteKV("input", input_name.string());
        writer.WriteTrajectory(traj);
        std::cout << "Ground-truth saved into " << gt_file.string() << std::endl;
    }
}

void SLAMBenchConfiguration::InitWriter() {
    if (writer_) {
        for(SLAMBenchLibraryHelper *lib : slam_libs_) {
            lib->GetMetricManager().reset();
            lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->reset();
        }
        //TODO: move this to callback when starting a new sequence
        aided_reloc_ = false;
    } else {
        // the following metrics last for all inputs; other metrics are allocated for only one input
        duration_metric_ = std::make_shared<slambench::metrics::DurationMetric>();
        power_metric_ = std::make_shared<slambench::metrics::PowerMetric>();
    }
    auto gt_traj = ground_truth_.GetMainOutput(slambench::values::VT_POSE);

    writer_ = std::make_unique<slambench::ColumnWriter>(this->GetLogStream(), "\t");
    writer_->AddColumn(&(row_number_));
    int i = 0;
    for(SLAMBenchLibraryHelper *lib : slam_libs_) {

        // retrieve the trajectory of the lib
        auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if (lib_traj == nullptr) {
            std::cerr << "There is no output trajectory in the library outputs." << std::endl;
            exit(1);
        }

        writer_->AddColumn(new slambench::OutputTimestampColumnInterface(lib_traj));
        if (gt_traj) {
            // Create an aligned trajectory
            auto aligned = new slambench::outputs::AlignedPoseOutput("", &*alignments_[i], lib_traj);

            // Add ATE metric
            auto ate_metric = std::make_shared<slambench::metrics::ATEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
            if (!ate_metric->GetValueDescription().GetStructureDescription().empty()) {
                lib->GetMetricManager().AddFrameMetric(ate_metric);
                writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*ate_metric, lib->GetMetricManager().GetFramePhase()));
            }

            // Add RPE metric
            auto rpe_metric = std::make_shared<slambench::metrics::RPEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
            lib->GetMetricManager().AddFrameMetric(rpe_metric);
            writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*rpe_metric, lib->GetMetricManager().GetFramePhase()));
        }
        else {
            std::cerr<<"NO GT TRAJECTORY!!"<<std::endl;
        }

        // Add a duration metric
        lib->GetMetricManager().AddFrameMetric(duration_metric_);
        lib->GetMetricManager().AddPhaseMetric(duration_metric_);
        writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, lib->GetMetricManager().GetFramePhase()));

        for(auto phase : lib->GetMetricManager().GetPhases()) {
            writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, phase));
        }

        // Add a memory metric
        auto memory_metric = std::make_shared<slambench::metrics::MemoryMetric>();
        lib->GetMetricManager().AddFrameMetric(memory_metric);
        writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*memory_metric, lib->GetMetricManager().GetFramePhase()));

        // Add a power metric if it makes sense
        if (!power_metric_->GetValueDescription().GetStructureDescription().empty()) {
            lib->GetMetricManager().AddFrameMetric(power_metric_);
            writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*power_metric_, lib->GetMetricManager().GetFramePhase()));
        }

        // Add XYZ row from the trajectory
        auto traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        traj->SetActive(true);
        writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, new slambench::outputs::PoseToXYZOutput(traj)));
        i++;
    }

    frame_callbacks_.clear();
    AddFrameCallback([this]{writer_->PrintRow();}); // @suppress("Invalid arguments")
    writer_->PrintHeader();
}

void SLAMBenchConfiguration::PrintDse() {
    for (SLAMBenchLibraryHelper* lib : slam_libs_) {
        std::cout << "libs:" << lib->GetIdentifier() << "\n" ;
        for (auto parameter : lib->getParameters()) {
            std::cout << "argument:" << parameter->getLongOption(lib) << "\n" ;
            std::cout << parameter->getStrDetails(lib) << "\n" ;
        }
    }
    exit(0);
}

void SLAMBenchConfiguration::StartStatistics() {

    GetLogStream().setf(std::ios::fixed, std::ios::floatfield);
    GetLogStream().precision(10);

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo=localtime(&rawtime);
    strftime(buffer,80,"%Y-%m-%d %I:%M:%S",timeinfo);
    this->GetLogStream() << "SLAMBench Report run started:\t" << buffer << std::endl << std::endl;

    // Print arguments known so far

    this->GetLogStream() << "Properties:" << std::endl << "=================" << std::endl << std::endl;

    param_manager_.PrintValues(GetLogStream());
}
