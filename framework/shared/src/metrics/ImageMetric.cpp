#include "sb_malloc.h"
#include "metrics/ImageMetric.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <cmath>

using namespace slambench::metrics;

/* Image Metric */

ImageMetric::ImageMetric(const slambench::outputs::BaseOutput * const frame) : Metric("ImageQuality"), frame_(frame)
{

}

void ImageMetric::MeasureStart(Phase* phase)
{
	(void)phase;
}

void ImageMetric::MeasureEnd(Phase* phase)
{
	(void)phase;
}

Value *ImageMetric::GetValue(Phase *phase)
{
	(void)phase;

    auto current_frame_sb = frame_->GetMostRecentValue();
    auto current_image_sb = reinterpret_cast<const FrameValue*>(current_frame_sb.second);

    /* ---------- Convert to cv::Mat type ---------- */
    const void* data = current_image_sb->GetData();
    uint32_t width = current_image_sb->GetWidth();
    uint32_t height = current_image_sb->GetHeight();
    auto pxl_format = current_image_sb->GetFormat();
    auto bytesPerPixel = slambench::io::pixelformat::GetPixelSize(pxl_format);
    
    // Determine the OpenCV type based on the pixel format
    int cvType;
    switch (bytesPerPixel) {
        case 1:
            cvType = CV_8UC1;
            break;
        case 3:
            cvType = CV_8UC3;
            break;
        default:
            throw std::runtime_error("Unsupported pixel format, Check ImageMetric.cpp");
    }

    cv::Mat current_image(height, width, cvType);
    size_t dataSize = current_image.step[0] * current_image.rows;

    std::memcpy(current_image.data, data, dataSize);
    /* --------------------------------------------- */

    /* ---------- Compute Image Quality ------------ */
    cv::Mat laplacian, absLaplacian;
    double sharpness, contrast, brightness, blur;

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


    auto sharpness_sb = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(sharpness);
    auto brightness_sb  = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(brightness);
	auto contrast_sb = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(contrast);

	return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
				{"Sharpness", sharpness_sb},
				{"Brightness", brightness_sb},
				{"Contrast", contrast_sb}});
}

const slambench::values::ValueDescription &ImageMetric::GetValueDescription() const
{
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		std::make_pair("Sharpness", slambench::values::VT_DOUBLE),
        std::make_pair("Brightness", slambench::values::VT_DOUBLE),
        std::make_pair("Contrast", slambench::values::VT_DOUBLE)});

	return desc;
}

const std::string &ImageMetric::GetDescription() const
{
	static std::string desc = "Image Quality";
	return desc;
}
