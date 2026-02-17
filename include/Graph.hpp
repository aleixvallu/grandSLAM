#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


#include "Objects.hpp"
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
    

    // noiseModel::Diagonal::shared_ptr imuNoise;
    noiseModel::Diagonal::shared_ptr lidarNoise;

    Values initial;

    ISAM2 solver;

    PreintegratedImuMeasurements preImu;

    NavState prevX;
    imuBias::ConstantBias prevB;

public:

    Graph() {

        // Initial pose
        Pose3 x0(Rot3(), Point3(0.0, 0.0, 0.0));
        Vector3 v0(0.0, 0.0, 0.0);
        imuBias::ConstantBias bias0;

        initial.insert(X(nImu), x0);
        initial.insert(V(nImu), v0);
        initial.insert(B(nImu), bias0);

        // Noise
        auto poseNoise = noiseModel::Diagonal::Sigmas((
            Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5) // rad, rad, rad, m, m, m
            .finished());  
        auto velNoise = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
        auto biasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);

        // Prior
        g.addPrior(X(nImu), x0, poseNoise);
        g.addPrior(V(nImu), v0, velNoise);
        g.addPrior(B(nImu), bias0, biasNoise);

        prevX = NavState(x0, v0);
        prevB = bias0;


        // ISAM2
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        solver = ISAM2(parameters);


        // Imu cov 
        auto params = PreintegrationParams::MakeSharedU(9.81); // U es de g Up D seria de Doww
        params->setAccelerometerCovariance(I_3x3 * 0.1);
        params->setGyroscopeCovariance(I_3x3 * 0.1);
        params->setIntegrationCovariance(I_3x3 * 0.1);
        params->setUse2ndOrderCoriolis(false);
        params->setOmegaCoriolis(Vector3(0, 0, 0));
        preImu = PreintegratedImuMeasurements(params, bias0);


        // imuNoise = Diag::Sigmas(Vector3(0.2, 0.2, 0.1));
        lidarNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    }


    void addImu(const Imu& imu, double dt) {

        preImu.integrateMeasurement(imu.lin_accel, imu.ang_vel, dt);

    }


    void addCones(const Cones& cones) {
        nImu++;

        ImuFactor iFact(X(nImu - 1), V(nImu - 1), X(nImu), V(nImu), B(nImu -1), preImu);
        g.add(iFact);
        
        
        imuBias::ConstantBias zeroBias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        auto biasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);
        g.add(BetweenFactor<imuBias::ConstantBias>(B(nImu - 1), B(nImu), zeroBias, biasNoise));
        
        for(const auto& c : cones) {

            Point3 cone(c(0), c(1), c(2));
            LandmarkFactor lFact(X(nImu), L(nLidar), cone, lidarNoise);
            g.add(lFact);
            
            nLidar++;
        }

        NavState nextX = preImu.predict(prevX, prevB);
        initial.insert(X(nImu), nextX.pose());
        initial.insert(V(nImu), nextX.v());
        initial.insert(B(nImu), prevB);


        Values result;

        solver.update(g, initial);
        result = solver.calculateEstimate();

        // Reset g & initial
        g.resize(0);
        initial.clear();

        // Overwrite the beginning of the preintegration for the next step.
        prevX = NavState(result.at<Pose3>(X(nImu)), result.at<Vector3>(V(nImu)));
        prevB = result.at<imuBias::ConstantBias>(B(nImu));

        // Reset the preintegration object.
        preImu.resetIntegrationAndSetBias(prevB);
    }




};