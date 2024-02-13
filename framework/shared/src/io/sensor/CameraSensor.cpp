/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/SensorDatabase.h"
#include <opencv2/opencv.hpp>
#include <cassert>
#include <Parameters.h>

using namespace slambench::io;

const Sensor::sensor_type_t CameraSensor::kCameraType = "Camera";

CameraSensor::CameraSensor(const Sensor::sensor_name_t  &name,  const Sensor::sensor_type_t &sensor_type) :
		Sensor(name, sensor_type) ,
		Width (0),
		Height (0),
		FrameFormat (slambench::io::frameformat::UNKNOWN),
		PixelFormat(slambench::io::pixelformat::UNKNOWN),
		DistortionType(NoDistortion)
{
	this->addParameter(TypedParameter<intrinsics_t>("ip", "intrinsics-parameters","Focal length and Principal Point Offset : fx,fy,cx,cy", &(this->Intrinsics),nullptr));
}

size_t CameraSensor::GetFrameSize(const SLAMFrame *frame) const {
	(void)frame;
	return Width * Height * pixelformat::GetPixelSize(PixelFormat);
}

void CameraSensor::CopyRadialTangentialDistortion(const distortion_coefficients_t &other) {
	for(unsigned int i = 0; i < 5 ; ++i) {
		RadialTangentialDistortion[i] = other[i];
	}
}

void CameraSensor::CopyEquidistantDistortion(const distortion_coefficients_t &other) {
  for(unsigned int i = 0; i < 4 ; ++i) {
    EquidistantDistortion[i] = other[i];
  }
}
void CameraSensor::CopyDistortion(const distortion_coefficients_t &other, const distortion_type_t& type) {
  for(unsigned int i = 0; i < 5 ; ++i) {
    Distortion[i] = other[i];
  }
}

void CameraSensor::CopyIntrinsics(const intrinsics_t &other) {
	for(unsigned int i = 0; i < 4 ; ++i) {
		Intrinsics[i] = other[i];
	}
}

void CameraSensor::CopyIntrinsics(const CameraSensor* other) {
	CopyIntrinsics(other->Intrinsics);
}

void * CameraSensor::Enhance(void * raw_image, std::unordered_map<std::string, std::vector<std::string>> * filters, std::unordered_map<std::string, slambench::io::FilterSettings>* settings){
	slambench::io::FilterSettings sensor_settings;
	auto settings_it = settings->find("camera");
	if(settings_it!=settings->end()){
		sensor_settings = settings_it->second;
	}
	else {
    	// Load default settings
    	sensor_settings = this->getDefaultSettings();
	}
	printf("Trying to enhance camera\n");
	for (const auto& sensorFilter : *filters) {
                const std::string& sensorName = sensorFilter.first;
                const std::vector<std::string>& sensor_filters = sensorFilter.second;

                std::cout << "Sensor: " << sensorName << ", Filters: ";
                for (const auto& filter : sensor_filters) {
                    std::cout << filter << " ";
                }
                std::cout << std::endl;
            }
	int img_type ;
	if(pixelformat::IsGrey(this->PixelFormat)){
		img_type = CV_8UC1;	
	}
	else{
		if(pixelformat::IsRGB(this->PixelFormat)){
			img_type = CV_8UC3;
		}
	}
	cv::Mat image_mat = cv::Mat(this->Height, this->Width, img_type, raw_image);
	// cv::imshow("Pre-edit", image_mat);
	
	// apply filters to the image in specified order
	auto filters_it = filters->find("camera");
	if(filters_it != filters->end()){
		std::vector<std::string>& filters_to_apply = filters_it->second;
		std::cout<<filters_it->first<<std::endl;
		for(const auto type : filters_to_apply){
			std::cout<<"Applying "<<type<<" to image"<<std::endl;
			if(type=="blur"){
			cv::blur(image_mat, image_mat, cv::Size(sensor_settings.kernel_size,sensor_settings.kernel_size));
			}else{
				if(type=="brightness"){
					// standard 50 brightness
					image_mat.convertTo(image_mat, -1, 1, sensor_settings.brightness);
				}
				else{
					if(type=="contrast"){
						image_mat.convertTo(image_mat, -1, sensor_settings.contrast, 0);
					}
					else{
						if(type=="noise"){
							cv::Mat noise(this->Height, this->Width, img_type);
							if(img_type==CV_8UC1)
								cv::randn(noise, sensor_settings.mean, sensor_settings.standard_deviation); //mean and variance
							if(img_type==CV_8UC3)
								cv::randn(noise, cv::Scalar(sensor_settings.mean,sensor_settings.mean,sensor_settings.mean), cv::Scalar(sensor_settings.standard_deviation,sensor_settings.standard_deviation,sensor_settings.standard_deviation));
							image_mat += noise;
						}else{
							// default if the enhancement method is not specified
							// return nullptr;
							continue;
						}
					}
				}
			}
		}
	}
	else{
		// message not necessary should otherwise be do nothing
		//  message  is for debugging
		printf("Cannot find camera sensor filter specifications for the frame\n");
	}
	
	cv::imshow("Post-edit", image_mat);
	cv::waitKey(0);
	void* edited_image = malloc(image_mat.total() * image_mat.elemSize());
    if (edited_image == nullptr) {
        // Handle memory allocation failure
        return nullptr;
    }

	memcpy(edited_image, image_mat.data, image_mat.total() * image_mat.elemSize());

	return edited_image;

}

class CameraSensorSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		CameraSensor *sensor = (CameraSensor*)s;
		
		serialiser->Write(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		serialiser->Write(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		serialiser->Write(&sensor->Width, sizeof(sensor->Width));
		serialiser->Write(&sensor->Height, sizeof(sensor->Height));
		serialiser->Write(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		serialiser->Write(&sensor->DistortionType, sizeof(sensor->DistortionType));
		serialiser->Write(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));
        serialiser->Write(&sensor->EquidistantDistortion, sizeof(sensor->EquidistantDistortion));
        serialiser->Write(&sensor->Distortion, sizeof(sensor->Distortion));
          return true;
	}
};

class CameraSensorDeserialiser : public SensorDeserialiser {
	
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type == CameraSensor::kCameraType) {
			*s = new CameraSensor(sensor_name, type);
			return true;
		} else {
			return false;
		}
	}

	bool DeserialiseSensorSpecific(const Deserialiser* deserialiser, Sensor* s) override {
		CameraSensor *sensor = (CameraSensor*)s;
		
		assert(sensor->GetType() == CameraSensor::kCameraType);
		
		deserialiser->Read(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		deserialiser->Read(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		deserialiser->Read(&sensor->Width, sizeof(sensor->Width));
		deserialiser->Read(&sensor->Height, sizeof(sensor->Height));
		deserialiser->Read(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		deserialiser->Read(&sensor->DistortionType, sizeof(sensor->DistortionType));
		deserialiser->Read(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));
        deserialiser->Read(&sensor->EquidistantDistortion, sizeof(sensor->EquidistantDistortion));
        deserialiser->Read(&sensor->Distortion, sizeof(sensor->Distortion));

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration camera_reg(CameraSensor::kCameraType, slambench::io::SensorDatabaseEntry(new CameraSensorSerialiser(), new CameraSensorDeserialiser(), false, false));

