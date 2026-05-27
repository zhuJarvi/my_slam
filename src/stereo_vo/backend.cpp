#include "algorithm.hpp"
#include "opt/g2o_types.hpp"
#include "stereo_vo/backend.hpp"
#include "stereo_vo/map.hpp"
#include "stereo_vo/struct_base/mappoint.hpp"
#include "stereo_vo/struct_base/feature.hpp"

namespace my_slam
{
    Backend::Backend()
    {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop()
    {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop()
    {
        while (backend_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            /// 后端仅优化激活的Frames和Landmarks
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyframesType &keyframes,
                           Map::LandmarksType &landmarks)
    {
        // setup g2o
        typedef g2o::BlockSolverX BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(
                std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // state 顶点，使用Keyframe id
        std::map<unsigned long, VertexNavState *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            VertexNavState *vertex_state = new VertexNavState();
            NavState init;
            init.pose_ = kf->GetPose();
            init.vel_ = kf->GetVelocity();
            init.ba_ = kf->GetBiasAcc();
            init.bg_ = kf->GetBiasGyro();
            vertex_state->setId(kf->keyframe_id_);
            vertex_state->setEstimate(init);
            optimizer.addVertex(vertex_state);
            if (kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_state});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // K 和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991; // robust kernel 阈值
        std::map<EdgeProjectionNavState *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks)
        {
            if (landmark.second->is_outlier_)
                continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations)
            {
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                auto frame = feat->frame_.lock();
                EdgeProjectionNavState *edge = nullptr;
                if (feat->is_on_left_image_)
                {
                    edge = new EdgeProjectionNavState(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjectionNavState(K, right_ext);
                }

                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                if (vertices.find(frame->keyframe_id_) !=
                        vertices.end() &&
                    vertices_landmarks.find(landmark_id) !=
                        vertices_landmarks.end())
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));   // state
                    edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Mat22::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert(std::make_pair(edge, feat));
                    optimizer.addEdge(edge);
                    index++;
                }
                else
                    delete edge;
            }
        }

        // IMU edges between consecutive keyframes
        int imu_edge_index = index;
        std::map<EdgeIMUPreint *, std::pair<unsigned long, unsigned long>> imu_edges;
        for (auto it = keyframes.begin(); it != keyframes.end(); ++it)
        {
            auto kf = it->second;
            if (!kf->imu_preint_ || kf->imu_prev_keyframe_id_ == 0)
                continue;
            auto prev_it = vertices.find(kf->imu_prev_keyframe_id_);
            auto cur_it = vertices.find(kf->keyframe_id_);
            if (prev_it == vertices.end() || cur_it == vertices.end())
                continue;

            auto *edge = new EdgeIMUPreint();
            edge->setId(imu_edge_index++);
            edge->setVertex(0, prev_it->second);
            edge->setVertex(1, cur_it->second);
            edge->setMeasurement(*kf->imu_preint_);
            Mat1515 info = Mat1515::Identity();
            Mat1515 cov = kf->imu_preint_->covariance();
            for (int i = 0; i < 15; ++i)
            {
                double sigma2 = cov(i, i);
                info(i, i) = 1.0 / (sigma2 + 1e-8);
            }
            edge->setInformation(info);
            optimizer.addEdge(edge);
            imu_edges.insert(std::make_pair(edge, std::make_pair(kf->imu_prev_keyframe_id_, kf->keyframe_id_)));
        }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5)
        {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features)
            {
                if (ef.first->chi2() > chi2_th)
                {
                    cnt_outlier++;
                }
                else
                {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5)
            {
                break;
            }
            else
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            }
            else
            {
                ef.second->is_outlier_ = false;
            }
        }

        spdlog::info("Optimization done with {} inliers and {} outliers.", cnt_inlier, cnt_outlier);

        // Set pose and lanrmark position
        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate().pose_);
            keyframes.at(v.first)->SetVelocity(v.second->estimate().vel_);
            keyframes.at(v.first)->SetBiases(v.second->estimate().ba_, v.second->estimate().bg_);
        }
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }
}