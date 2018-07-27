//
// Created by ernest on 18-7-22.
//

#include "Observation.h"

namespace dso{
    namespace IOWrap{
        Observation::Observation(int frame_id_, const Eigen::Vector2f & ob_) : frame_id(frame_id_), projection(ob_) {}

        Observation::~Observation() {}
    }
}