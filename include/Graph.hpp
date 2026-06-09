#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>


#include <as_lib/structures/iOctree.hpp>

#include "Objects.hpp"
#include "Config.hpp"
#include "LandmarkFactor.hpp"

using namespace gtsam;
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L;  // Lidar  (x,y,z)



class Graph {

    NonlinearFactorGraph g;

    int nImu = 0;
    int nLidar = 0;

    noiseModel::Diagonal::shared_ptr lidarNoise;

    Values initial;

    

 
    IncrementalFixedLagSmoother solver;
    FixedLagSmoother::KeyTimestampMap timestamps;

    PreintegratedCombinedMeasurements preImu;


    
    NavState prevX;
    NavState state;
    imuBias::ConstantBias prevB;
    

    bool loopClosed = false;  
    bool changed = true;
    Point2 finishLine;

    // Define y < mx + n perpendicular to the starting point
	double m = 0.0; // Slope
	double n = 0.0; // Intercept

    as_lib::structures::Octree<Cone> octree;
    std::unordered_map<int, Point3> landmarks;

public:

    Graph() {}

    void init(double time){

        Config &cfg = Config::getInstance();

        // Initial pose
        Pose3 x0(Rot3(Eigen::Quaterniond(cfg.imu2baselink.linear())), 
                 Point3(cfg.imu2baselink.translation()));
        Vector3 v0(0.0, 0.0, 0.0);
      
        m = tan(x0.rotation().yaw());
        finishLine = Point2(x0.translation().x(), x0.translation().y());
        n = finishLine.x() - m * finishLine.y(); 

        imuBias::ConstantBias biasImu(
                cfg.bias.accel,
                cfg.bias.gyro
        );

        initial.insert(X(nImu), x0);
        initial.insert(V(nImu), v0);
        initial.insert(B(nImu), biasImu);

        timestamps[X(nImu)] = time;
        timestamps[V(nImu)] = time;
        timestamps[B(nImu)] = time;

        // Noise
        auto poseNoise = noiseModel::Isotropic::Sigma(6, cfg.cov.initial.pose);  // rad, rad, rad, m, m, m  
        auto velNoise = noiseModel::Isotropic::Sigma(3, cfg.cov.initial.vel);           // m/s
        auto biasNoise = noiseModel::Isotropic::Sigma(6, cfg.cov.initial.bias); 

        // Prior
        g.addPrior(X(nImu), x0, poseNoise);
        g.addPrior(V(nImu), v0, velNoise);
        g.addPrior(B(nImu), biasImu, biasNoise);

        prevX = NavState(x0, v0);
        prevB = biasImu;

   
        
        FastMap<char, Vector> thresholds;
        thresholds['x'] = Eigen::Map<const Eigen::VectorXd>(
            cfg.isam.threshold.pose.data(), cfg.isam.threshold.pose.size());

        thresholds['v'] = Eigen::Map<const Eigen::VectorXd>(
            cfg.isam.threshold.vel.data(), cfg.isam.threshold.vel.size());

        thresholds['b'] = Eigen::Map<const Eigen::VectorXd>(
            cfg.isam.threshold.bias.data(), cfg.isam.threshold.bias.size());

        thresholds['l'] = Eigen::Map<const Eigen::VectorXd>(
            cfg.isam.threshold.landmark.data(), cfg.isam.threshold.landmark.size());
            
        ISAM2Params parameters; 
        parameters.relinearizeThreshold = thresholds;
        
        parameters.relinearizeSkip = cfg.isam.skip;
        parameters.enableDetailedResults = cfg.gtsam_debug;

        solver = IncrementalFixedLagSmoother(cfg.isam.lag, parameters);

        


        // Imu cov 
        auto params = PreintegrationCombinedParams::MakeSharedU(cfg.bias.gravity); // U ==> Up, D ==> Down
        params->setIntegrationCovariance(I_3x3 * cfg.cov.process);
        params->setAccelerometerCovariance(I_3x3 * cfg.cov.accel);
        params->setGyroscopeCovariance(I_3x3 * cfg.cov.gyro);
        params->setBiasAccCovariance(I_3x3 * cfg.cov.biasA);
        params->setBiasOmegaCovariance(I_3x3 * cfg.cov.biasG);
        params->setBiasAccOmegaInit(I_6x6 * cfg.cov.initial.bias);
        params->setUse2ndOrderCoriolis(false);
        params->setOmegaCoriolis(Vector3(0, 0, 0));
        preImu = PreintegratedCombinedMeasurements(params, biasImu);

        lidarNoise = noiseModel::Isotropic::Sigma(3, cfg.cov.initial.lidar);

        octree.setBucketSize(2);
        octree.setDownsample(false);
        octree.setMinExtent(0.2);
    }


    void addImu(const Imu &imu, double dt) {
        PROFC_NODE_

        preImu.integrateMeasurement(imu.lin_accel, imu.ang_vel, dt);
        state = preImu.predict(prevX, prevB);
        
    }

    // TODO: no afegir sempre que veig un factor, afegirlo cada cert temps/distancia, aixi no tens tants facotrs 
    void addCones(const Cones &cones, double time) {
        PROFC_NODE_
        Config &cfg = Config::getInstance();

        // Guard: don't add an IMU factor with zero integration time
        if (preImu.deltaTij() <= 0.0) {
            RCLCPP_WARN(rclcpp::get_logger("graph"), 
                "Skipping cone update: preImu deltaTij=0 (no IMU received since last cone callback)");
            return;
        }


        nImu++;

        NavState nextX = preImu.predict(prevX, prevB);
        initial.insert(X(nImu), nextX.pose());
        initial.insert(V(nImu), nextX.v());
        initial.insert(B(nImu), prevB);

        timestamps[X(nImu)] = time;
        timestamps[V(nImu)] = time;
        timestamps[B(nImu)] = time;

        Point3 prevPos = prevX.pose().translation();
        Point3 currPos =  nextX.pose().translation();

        if(!changed &&
		    prevPos.x() - (m * prevPos.y() + n) < 0 && 
			currPos.x() - (m * currPos.y() + n) > 0 &&
			sqrt(pow(currPos.x() - finishLine.x(), 2) + pow(currPos.y() - finishLine.y(), 2)) < cfg.closingDist){

            loopClosed = true;
            changed = !changed;
            std::cout << "Loop closed" << std::endl;
        } else if(changed && nextX.v().x() > 1.0 ){
            changed = false;
        }

        
        CombinedImuFactor iFact(X(nImu - 1), V(nImu - 1), X(nImu), V(nImu), B(nImu -1), B(nImu), preImu);
        g.add(iFact);
        

        int N = cones.size();

        Pose3 actualPose = nextX.pose();
        for(int i = 0; i < N; i++) { 

            const Cone &c = cones[i];
            Point3 worldCone = actualPose.transformFrom(c.toEigen());
            Cone p(worldCone.x(), worldCone.y(), worldCone.z(), nLidar);

            Cones neighbors;
            std::vector<float> sqDistances;
            octree.knn(p, 1, neighbors, sqDistances);       

            if(octree.size() == 0 || sqDistances[0] > cfg.maxSqDist * (1 + 0.5 * loopClosed)) {
                if(loopClosed) 
                    continue;

                LandmarkFactor lFact(X(nImu), L(nLidar), c.toEigen(), lidarNoise);
                g.add(lFact);

                
                initial.insert(L(nLidar), worldCone);
                timestamps[L(nLidar)] = time;
                landmarks[nLidar] = worldCone; 

                nLidar++;
            } else {
                int nIdx = neighbors[0].idx;
                if (timestamps.find(L(nIdx)) == timestamps.end()) {
                    // if (!landmarks.count(nIdx))
                        continue;
                    // initial.insert(L(nIdx), landmarks[nIdx]);
                }
                LandmarkFactor lFact(X(nImu), L(nIdx), c.toEigen(), lidarNoise);
                g.add(lFact);
                timestamps[L(nIdx)] = time;
            }
        }

        Values result;

        try
        {
            {
                PROFC_NODE("SOLVER")
                solver.update(g, initial, timestamps);
            }
            {
                PROFC_NODE("ESTIMATE")
                result = solver.calculateEstimate();
                // result = solver.isam_.calculateBestEstimate(); //TODO: mirar si val la pena fer el wrapper?
 
            }
        } catch (const gtsam::IndeterminantLinearSystemException &e) {
            std::cerr << "GTSAM IndeterminantLinearSystem: " << e.what() << std::endl;
            std::cerr << "Offending key: " << gtsam::DefaultKeyFormatter(e.nearbyVariable()) << std::endl;
            
            // Print the full factor graph to see what's connected to it
            std::cerr << "Factor graph size: " << g.size() << std::endl;
            std::cerr << "nImu: " << nImu << " nLidar: " << nLidar << std::endl;
            
            // Print which factors involve this key
            for (size_t i = 0; i < g.size(); i++) {
                auto factor = g[i];
                if (!factor) continue;
                for (auto key : factor->keys()) {
                    if (key == e.nearbyVariable()) {
                        std::cerr << "Factor " << i << " involves offending variable" << std::endl;
                        factor->print();
                    }
                }
            }
            return; // or handle gracefully
        }
        
        if(cfg.gtsam_debug) 
            GTSAM_PRINT(result);

        

        // Reset g & initial
        g.resize(0);
        initial.clear();
        
        auto linPoint = solver.getLinearizationPoint();
        for (auto it = timestamps.begin(); it != timestamps.end(); ) {
            if (!linPoint.exists(it->first))
                it = timestamps.erase(it);  // ya marginalizada, sacar del mapa
            else
                ++it;
        }

        
        // Overwrite the beginning of the preintegration for the next step.
        prevX = NavState(result.at<Pose3>(X(nImu)), result.at<Vector3>(V(nImu)));
        prevB = result.at<imuBias::ConstantBias>(B(nImu));
        state = prevX;

        {
            PROFC_NODE("OCTREE + LDMRKS")
            for(int i = 0; i < nLidar; i++) {
                if (result.exists(L(i))) {
                    landmarks[i] = result.at<Point3>(L(i));
                }
            }

            octree.clear();
            Cones toAdd;
            for (const auto& [idx, p] : landmarks) {
                Cone cone(p.x(), p.y(), p.z(), idx);
                toAdd.push_back(cone);
            }
            octree.update(toAdd);
        }

        // Reset the preintegration object.
        preImu.resetIntegrationAndSetBias(prevB);
    }
    
    Eigen::Quaterniond R() const { return state.quaternion(); }
    Eigen::Vector3d t() const { return state.t(); }

    Cones cones() {
        return octree.getData<Cones>();
    }


    Eigen::Isometry3d isometry() const {
        Eigen::Isometry3d I;
        I.linear() = R().toRotationMatrix();
        I.translation() = t();
        return I;
    }
    
    // TODO: posar w si la utilitzo
    Eigen::Matrix< double, 6, 1 > v_w() const
    {
        Eigen::Matrix< double, 6, 1 > v_w;
        v_w.head<3>() = state.v();
        v_w.tail<3>().setZero();
        return v_w;
    }

};