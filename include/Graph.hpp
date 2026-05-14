#pragma once


#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
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

    noiseModel::Diagonal::shared_ptr lidarNoise;

    Values initial;

    ISAM2 solver;

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

public:

    Graph() {}

    void init(){

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


        // ISAM2
        ISAM2Params parameters;
        parameters.optimizationParams;
        
        
        
        FastMap<char, Vector> thresholds;

        thresholds['x'] =
            (Vector(6) <<
                0.01, 0.01, 0.01,
                0.005, 0.005, 0.005).finished();

        thresholds['v'] =
            (Vector(3) <<
                0.05, 0.05, 0.05).finished();

        thresholds['b'] =
            (Vector(6) <<
                1e-3, 1e-3, 1e-3,
                1e-4, 1e-4, 1e-4).finished();

        thresholds['l'] =
            (Vector(3) <<
                1000., 1000., 1000.).finished();

        parameters.relinearizeThreshold = thresholds;
        // parameters.relinearizeThreshold = cfg.isam.th;
        
        
        parameters.relinearizeSkip = cfg.isam.skip;
        // parameters.enableRelinearization;
        // parameters.evaluateNonlinearError
        // parameters.factorization:
        // parameters.cacheLinearizedFactors;
        // cacheLinearizedFactors;
        parameters.enableDetailedResults = cfg.gtsam_debug;
        parameters.enablePartialRelinearizationCheck = cfg.isam.relCheck;
        // parameters.findUnusedFactorSlots;

        solver = ISAM2(parameters);


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

        preImu.integrateMeasurement(imu.lin_accel, imu.ang_vel, dt);
        state = preImu.predict(prevX, prevB);
        
    }

    // TODO: no afegir sempre que veig un factor, afegirlo cada cert temps/distancia, aixi no tens tants facotrs 
    void addCones(const Cones &cones) {
        PROFC_NODE("GTSAM")
        Config &cfg = Config::getInstance();

        nImu++;


        NavState nextX = preImu.predict(prevX, prevB);
        initial.insert(X(nImu), nextX.pose());
        initial.insert(V(nImu), nextX.v());
        initial.insert(B(nImu), prevB);

        Point3 prevPos = prevX.pose().translation();
        Point3 currPos =  nextX.pose().translation();

        // if(!loopClosed && 
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

        Cones toAdd;
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
                
                toAdd.push_back(p);
                nLidar++;
            } else {
                int nIdx = neighbors[0].idx;
                LandmarkFactor lFact(X(nImu), L(nIdx), c.toEigen(), lidarNoise);
                g.add(lFact);
            }
        }
        octree.update(toAdd);
        

        Values result;

        
        {
            PROFC_NODE("SOLVER")
            auto metrics = solver.update(g, initial);
            std::cout << "Relinarize: " <<  metrics.variablesRelinearized << std::endl;
            std::cout << "Reeliminated: " << metrics.variablesReeliminated << std::endl;
        }
        result = solver.calculateEstimate();
        
        if(cfg.gtsam_debug) 
            GTSAM_PRINT(result);

        // Matrix covX = solver.marginalCovariance(X(nImu));
        // std::cout << "Cov X" << nImu << ":\n" << covX << std::endl;

        

        // Reset g & initial
        g.resize(0);
        initial.clear();

        // Overwrite the beginning of the preintegration for the next step.
        prevX = NavState(result.at<Pose3>(X(nImu)), result.at<Vector3>(V(nImu)));
        prevB = result.at<imuBias::ConstantBias>(B(nImu));
        state = prevX;


        octree.clear();

        for(int i = 0; i < nLidar; i++) {
            Point3 p(result.at<Point3>(L(i)));
            Cone cone(p.x(), p.y(), p.z(), i);
            toAdd.push_back(cone);
        }
        octree.update(toAdd);



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