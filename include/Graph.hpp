#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

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
    
    imuBias::ConstantBias biasImu;

    noiseModel::Diagonal::shared_ptr biasNoise;
    noiseModel::Diagonal::shared_ptr lidarNoise;

    Values initial;

    ISAM2 solver;

    PreintegratedImuMeasurements preImu;

    NavState prevX;
    NavState state;
    imuBias::ConstantBias prevB;
    
    // Pose3 ext;  

    as_lib::structures::Octree<Cone> octree;

public:

    Graph() {}

    void init(){

        Config &cfg = Config::getInstance();

        // Initial pose
        Pose3 x0(Rot3(Eigen::Quaterniond(cfg.imu2baselink.linear())), 
                 Point3(cfg.imu2baselink.translation()));
        Vector3 v0(0.0, 0.0, 0.0);
      
        biasImu = imuBias::ConstantBias(
                cfg.bias.accel,
                cfg.bias.gyro
        );

        initial.insert(X(nImu), x0);
        initial.insert(V(nImu), v0);
        initial.insert(B(nImu), biasImu);

        // Noise
        auto poseNoise = noiseModel::Isotropic::Sigma(6, cfg.cov.pose);  // rad, rad, rad, m, m, m  
        auto velNoise = noiseModel::Isotropic::Sigma(3, 0.1);           // m/s
        biasNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 
                Eigen::Vector3d::Constant(cfg.cov.biasA),       // m, m, m,
                Eigen::Vector3d::Constant(cfg.cov.biasG)        // rad, rad, rad
        ).finished());  

        // Prior
        g.addPrior(X(nImu), x0, poseNoise);
        g.addPrior(V(nImu), v0, velNoise);
        g.addPrior(B(nImu), biasImu, biasNoise);

        prevX = NavState(x0, v0);
        prevB = biasImu;


        // ISAM2
        ISAM2Params parameters;
        parameters.relinearizeThreshold = cfg.isam.th;
        parameters.relinearizeSkip = cfg.isam.skip;
        solver = ISAM2(parameters);


        // Imu cov 
        auto params = PreintegrationParams::MakeSharedU(cfg.bias.gravity); // U es de g Up D seria de Doww
        params->setAccelerometerCovariance(I_3x3 * cfg.cov.accel);
        params->setGyroscopeCovariance(I_3x3 * cfg.cov.gyro);
        params->setIntegrationCovariance(I_3x3 * cfg.cov.process);
        params->setUse2ndOrderCoriolis(false);
        params->setOmegaCoriolis(Vector3(0, 0, 0));
        preImu = PreintegratedImuMeasurements(params, biasImu);


        lidarNoise = noiseModel::Isotropic::Sigma(3, cfg.cov.lidar);

        octree.setBucketSize(2);
        octree.setDownsample(false);
        octree.setMinExtent(0.2);
    }


    void addImu(const Imu &imu, double dt) {

        preImu.integrateMeasurement(imu.lin_accel, imu.ang_vel, dt);
        state = preImu.predict(prevX, prevB);
        
    }


    void addCones(const Cones &cones) {
        PROFC_NODE("GTSAM")
        Config &cfg = Config::getInstance();

        nImu++;


        NavState nextX = preImu.predict(prevX, prevB);
        initial.insert(X(nImu), nextX.pose());
        initial.insert(V(nImu), nextX.v());
        initial.insert(B(nImu), prevB);

        // if(nextX.pose().equals(Pose3(), 1.) && nImu > 100) {
        //     // loopClosed = true;
        //     // ImuFactor loopClousre(X(nImu - 1), V(nImu - 1), X(0), V(nImu), B(nImu -1), preImu);
        //     // g.add(loopClousre);
        // } 
        
        ImuFactor iFact(X(nImu - 1), V(nImu - 1), X(nImu), V(nImu), B(nImu -1), preImu);
        g.add(iFact);
        

        imuBias::ConstantBias zeroBias;
        g.add(BetweenFactor<imuBias::ConstantBias>(B(nImu - 1), B(nImu), zeroBias, biasNoise));


        int N = cones.size();

        Cones toAdd;
        Pose3 actualPose = nextX.pose();
        for(int i = 0; i < N; i++) { 

            const Cone &c = cones[i];
            Point3 worldCone = actualPose.transformFrom(c.toEigen());
            Cone p(worldCone.x(), worldCone.y(), worldCone.z(), nLidar);

            Cones neighbors;
            std::vector<float> sqDistances;
            octree.knn(p, 1, neighbors, sqDistances);       

            if(octree.size() == 0 || sqDistances[0] > cfg.maxSqDist) {

                LandmarkFactor lFact(X(nImu), L(nLidar), c.toEigen(), lidarNoise);
                g.add(lFact);

                
                initial.insert(L(nLidar), worldCone);
                
                toAdd.push_back(p);
                nLidar++;
            } else {
                int nIdx = neighbors[0].idx;
                LandmarkFactor lFact(X(nImu), L(nIdx), c.toEigen(), lidarNoise);
                g.add(lFact);
            }
        }
        std::cout << "Seen cones " << octree.size() << std::endl;
        std::cout << "Added cones " << toAdd.size() << std::endl;

        octree.update(toAdd);
        

        Values result;

        solver.update(g, initial);
        result = solver.calculateEstimate();
        
        if(cfg.gtsam_debug)
            GTSAM_PRINT(result);

        Matrix covX = solver.marginalCovariance(X(nImu));
        std::cout << "Cov X" << nImu << ":\n" << covX << std::endl;

        // Reset g & initial
        g.resize(0);
        initial.clear();

        // Overwrite the beginning of the preintegration for the next step.
        prevX = NavState(result.at<Pose3>(X(nImu)), result.at<Vector3>(V(nImu)));
        prevB = result.at<imuBias::ConstantBias>(B(nImu));
        state = prevX;

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