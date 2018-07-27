//
// Created by ernest on 18-7-19.
//

#include "JdeRobotOutputIO.h"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;


    namespace IOWrap {

        JdeRobotOutputIO::JdeRobotOutputIO(std::string filename) : target_filename(filename),
                                                                   model3DMutex(),
                                                                   keyframes(),
                                                                   keyframesByKFID(),
                                                                   Hcalib_() {

        }

        void JdeRobotOutputIO::publishGraph(
                const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                        Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) {

        }

        void JdeRobotOutputIO::publishKeyframes(std::vector<dso::FrameHessian *> &frames, bool final,
                                                dso::CalibHessian *HCalib) {

            Hcalib_ = HCalib;

            boost::unique_lock<boost::mutex> lk(model3DMutex);
            for(FrameHessian* fh : frames)
            {
                if(keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
                {
                    KeyFrameDisplay* kfd = new KeyFrameDisplay();
                    keyframesByKFID[fh->frameID] = kfd;
                    keyframes.push_back(kfd);
                }
                keyframesByKFID[fh->frameID]->setFromKF(fh, HCalib);
            }

        }

        void JdeRobotOutputIO::publishCamPose(FrameShell *frame, CalibHessian *HCalib) {

        }

        void JdeRobotOutputIO::pushLiveFrame(FrameHessian *image) {

        }

        void JdeRobotOutputIO::pushDepthImage(MinimalImageB3 *image) {

        }

        bool JdeRobotOutputIO::needPushDepthImage() {

        }

        void JdeRobotOutputIO::reset() {
            throw std::runtime_error("Not implemented yet");
        }

        void JdeRobotOutputIO::join() {
            for (auto ph : keyframes){
                // apply threshold
                ph->refreshPC(true, 0.001, 0.001, 1, 0.1, 1);
            }

            std::ofstream myfile(target_filename.c_str());
            if (!myfile){
                std::cerr << "[ERROR] Failed to open file: " <<  target_filename << std::endl;
                throw std::runtime_error("");
            }

            // YAML 1.0
            std::string output = "%YAML:1.0\n";
            // Save camera parameters
            output += "camera:\n";
            output += "  fx: " + std::to_string(Hcalib_->fxl()) + "\n";
            output += "  fy: " + std::to_string(Hcalib_->fyl()) + "\n";
            output += "  cx: " + std::to_string(Hcalib_->cxl()) + "\n";
            output += "  cy: " + std::to_string(Hcalib_->cyl()) + "\n";
            // because Monocular Dataset is using vignette, set distortion to 0
            output += "  k1: " + std::to_string(0.0) + "\n";
            output += "  k2: " + std::to_string(0.0) + "\n";
            output += "  p1: " + std::to_string(0.0) + "\n";
            output += "  p2: " + std::to_string(0.0) + "\n";
            output += "  k3: " + std::to_string(0.0) + "\n";
            myfile << output;

            // save keyframe poses
            myfile << "keyframes:\n";
            for (auto kf : keyframes){
                // get rotation and translation, convert Sophus::SE3 to Eigen::Matrix4d
                auto cam_r = kf->camToWorld.rotationMatrix();
                auto cam_t = kf->camToWorld.translation();
                Eigen::Matrix3d rotation_mat(3,3);
                for (int i = 0; i < 3; i++){
                    for (int j = 0; j < 3; j++){
                        rotation_mat(i,j) = cam_r(i,j);
                    }
                }

                Eigen::Quaterniond q(rotation_mat);
                Eigen::Vector3d t(cam_t);

                output = "";
                output += "  - id: " + std::to_string(kf->id) + "\n";
                output += "    filename: \"" + kf->filename + "\"\n";
                output += "    pose:\n";
                output += "      - " + std::to_string(q.w()) + "\n";
                output += "      - " + std::to_string(q.x()) + "\n";
                output += "      - " + std::to_string(q.y()) + "\n";
                output += "      - " + std::to_string(q.z()) + "\n";
                output += "      - " + std::to_string(t(0)) + "\n";
                output += "      - " + std::to_string(t(1)) + "\n";
                output += "      - " + std::to_string(t(2)) + "\n";
                myfile << output;
            }


            myfile << "points:\n";
            int counter = 0;
            for (KeyFrameDisplay * kf : keyframes){
                counter += kf->printPoints(counter,myfile);
            }

            std::cout << "[INFO] Trajectory saved to file: " << target_filename << std::endl;
            std::cout << "Number of points: " << counter << std::endl;
            std::cout << "Numebr of keyframes: " << keyframes.size() << std::endl;

            myfile.close();
        }


    }
}