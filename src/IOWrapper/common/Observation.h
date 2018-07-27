//
// Created by ernest on 18-7-22.
//

#ifndef DSO_OBSERVATION_H
#define DSO_OBSERVATION_H

#include <Eigen/Core>

namespace dso {

    namespace IOWrap {

        class Observation {
        public:
            int frame_id;

            // [u;v]
            Eigen::Vector2f projection;


            Observation(int frame_id_, const Eigen::Vector2f & ob_);
            ~Observation();
        };

    }
}


#endif //DSO_OBSERVATION_H
