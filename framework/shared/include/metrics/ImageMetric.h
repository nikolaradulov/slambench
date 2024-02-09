#ifndef IMAGEMETRIC_H
#define IMAGEMETRIC_H
#include "Metric.h"
#include <outputs/Output.h>

namespace slambench {
    namespace metrics {

        using slambench::values::FrameValue;

        class ImageMetric : public Metric {

        public:
            ImageMetric(const slambench::outputs::BaseOutput * const image);

            ~ImageMetric() = default;

            const slambench::values::ValueDescription& GetValueDescription() const override;
            const std::string& GetDescription() const override;
            void MeasureStart(Phase* phase) override;
            void MeasureEnd(Phase* phase) override;
            Value *GetValue(Phase* phase) override;

        private:
            const slambench::outputs::BaseOutput * const frame_;
            // const slambench::outputs::BaseOutput * const image2_;

        };
    }
}

#endif //IMAGEMETRIC_H