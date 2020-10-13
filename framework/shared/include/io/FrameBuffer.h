/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_FRAMEBUFFER_H
#define IO_FRAMEBUFFER_H

#include <atomic>
#include <cstring>

namespace slambench {
	namespace io {
		class FrameBuffer {
		public:
			FrameBuffer();
			~FrameBuffer();
		
			void Acquire();
			bool TryAcquire();
			void Release();
			void ResetBuffer();
			bool Busy();
			bool Reserve(size_t size);
			void *Data();
			void resetLock() {
				lock_.exchange(false);
			}
		
		private:
			std::atomic<bool> lock_;
			size_t size_;
			void *data_;
		};
	}
}

#endif
