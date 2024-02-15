/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFRAME_H
#define IO_SLAMFRAME_H

#include <string>

#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "TimeStamp.h"
#include "PixelFormat.h"
#include <io/sensor/Sensor.h>

namespace slambench {
	namespace io {
		
		
		class Sensor;
		class FrameBuffer;
		
		class SLAMFrame {
		public:
			virtual ~SLAMFrame();
		
			TimeStamp Timestamp;
			Sensor *FrameSensor;
			
			std::unordered_map<std::string, std::pair<std::vector<std::string>, slambench::io::FilterSettings>> * filters;
    		std::unordered_map<std::string, slambench::io::FilterSettings> * settings;

			bool enhance_=false;

			size_t GetSize() const;
			void SetVariableSize(uint32_t size);
			virtual void Enhance(std::unordered_map<std::string, std::pair<std::vector<std::string>, slambench::io::FilterSettings>> * filters){printf("This is the base class\n");}
			uint32_t GetVariableSize() const;
            virtual void *GetData() = 0;
            virtual void FreeData() = 0;
            inline void SetData(void* data, size_t size)
            {
                    data_ = malloc(size);
                    memcpy(data_, data, size);
            }
            inline void SetData(void* data)
            {
                assert(data);
                data_ = data;
            }

		protected:
            void *data_;

		private:
			uint32_t size_if_variable_sized_;
			
		};
		
		class SLAMInMemoryFrame : public SLAMFrame {
		public:
			void *Data;
			// void Enhance(std::unordered_map<std::string, std::vector<std::string>> * filters, std::unordered_map<std::string, std::unordered_map<std::string, slambench::io::FilterSettings>>* settings) override;
			void *GetData() override;
			void FreeData() override;
		};
		
		class SLAMFileFrame : public SLAMFrame {
		public:
			SLAMFileFrame();
			
			typedef void (*callback_t)(SLAMFileFrame *, void *);
			callback_t ProcessCallback;
			std::string filename;
			
			void *GetData() override;
			void FreeData() override;
			
		protected:
			virtual void *LoadFile() = 0;
		};
		
		class TxtFileFrame : public SLAMFileFrame {
		public:
			pixelformat::EPixelFormat input_pixel_format;

		protected:
			void *LoadFile() override;
			
		private:
			void *LoadCameraFile();
		};
		
		class ImageFileFrame : public SLAMFileFrame {
		protected:
			void *LoadFile() override;
			
		private:
			// void Enhance(std::unordered_map<std::string, std::vector<std::string>> * filters, std::unordered_map<std::string, std::unordered_map<std::string, slambench::io::FilterSettings>>* settings) override;
			void *LoadPng();
			void *LoadPbm();
		};
		
		class DeserialisedFrame : public SLAMFrame {
		public:
			DeserialisedFrame(FrameBuffer &buffer, FILE *file);
			void Enhance(std::unordered_map<std::string, std::pair<std::vector<std::string>, slambench::io::FilterSettings>> * filters) override;
			void SetOffset(size_t data_offset);
			void *GetData() override;
			void *GetDataHelper();
			void FreeData() override;
			FrameBuffer& getFrameBuffer() {
				return buffer_;
			}
			
		private:
			FrameBuffer &buffer_;
			FILE *file_;
			void *enhanced_image_=nullptr;
			size_t offset_;
		};
	}
}

#endif
