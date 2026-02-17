import gtsam
from gtsam import noiseModel, Pose2, BetweenFactorPose2, LevenbergMarquardtOptimizer, Marginals, BearingRangeFactor2D, Point2
from gtsam.symbol_shorthand import X, L
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np



ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
LANDMARK_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))

noise_step = 0.2
noise_rot = 0.01


def rmse_se2(gt, est):
    N = gt.shape[0]
    total = 0.0
    
    for i in range(N):
        p_gt = Pose2(*gt[i])
        p_est = Pose2(*est[i])
        
        error_pose = p_gt.between(p_est)
        
        e = np.array([
            error_pose.x(),
            error_pose.y(),
            error_pose.theta()
        ])
        
        total += np.dot(e, e)
        
    return np.sqrt(total / N)


def genData(
    N=40,
    radius=5.0,
    seed=10
):
    """
    Generates a circular trajectory that returns
    close to the initial position.

    Returns:
        gt_poses: (N,3) absolute GT
        odom_deltas: (N,3) relative noisy deltas
    """

    if seed is not None:
        np.random.seed(seed)

    gt_poses = np.zeros((N,3))
    odom_deltas = np.zeros((N,3))

    # Angular increment to complete one circle
    dtheta = 2*np.pi / N
    step = 2*np.pi*radius / N   # arc length approx

    # Ground truth pose
    pose_gt = Pose2(0,0,0)

    for i in range(N):

        # ---- TRUE DELTA ----
        dx = step
        dy = 0.0
        dth = dtheta

        delta_gt = Pose2(dx, dy, dth)

        pose_gt = pose_gt.compose(delta_gt)

        gt_poses[i] = [pose_gt.x(), pose_gt.y(), pose_gt.theta()]

        # ---- NOISY DELTA ----
        dx_n = dx + np.random.normal(0, noise_step)
        dth_n = dth + np.random.normal(0, noise_rot)

        odom_deltas[i] = [dx_n, 0.0, dth_n]

    return gt_poses, odom_deltas



def genData2(
    N=40,
    side_length=5.0,
    seed=10
):
    """
    Generates a circular trajectory that returns
    close to the initial position.

    Returns:
        gt_poses: (N,3) absolute GT
        odom_deltas: (N,3) relative noisy deltas
    """

    if seed is not None:
        np.random.seed(seed)

    gt_poses = np.zeros((N,3))
    odom_deltas = np.zeros((N,3))

    steps_per_side = N // 4
    step = side_length / steps_per_side

    # Ground truth pose
    pose_gt = Pose2(0,0,0)

    for i in range(N):

        # ---- TRUE DELTA ----
        dx = step
        dy = 0.0
        dth = 0
        if (i > 0) and (i % steps_per_side == 0):           
            dth = np.pi / 2
        

        delta_gt = Pose2(dx, dy, dth)

        pose_gt = pose_gt.compose(delta_gt)

        gt_poses[i] = [pose_gt.x(), pose_gt.y(), pose_gt.theta()]

        # ---- NOISY DELTA ----
        dx_n = dx + np.random.normal(0, noise_step)
        dth_n = dth + np.random.normal(0, noise_rot)

        odom_deltas[i] = [dx_n, 0.0, dth_n]

    return gt_poses, odom_deltas



def main():

    useLandMark = True
    Ls = 5
    quad = True

    N = 100
    dist = 5.0
    gt_data, odomData = genData(N, dist)
    if(quad):
        gt_data, odomData = genData2(N, dist)

    graph = gtsam.NonlinearFactorGraph()




    nImu = 1
    nOpt = 1

    priorMean = Pose2(0.0, 0.0, 0.0)
    graph.add(gtsam.PriorFactorPose2(X(nImu), priorMean, PRIOR_NOISE))

    curPose = Pose2(0.0, 0.0, 0.0)

    initial = gtsam.Values()
    initial.insert(X(nOpt), curPose)

    landmarks = np.empty(5, dtype=object) 
    landmarks[0] = Point2(0, dist)
    landmarks[1] = Point2(-dist, 0) 
    landmarks[2] = Point2(dist, 0) 
    landmarks[3] = Point2(-dist, 2 *dist)
    landmarks[4] = Point2(dist, 2 * dist)
    if(quad):
        landmarks[0] = Point2(0.5 * dist, 0.5 * dist)
        landmarks[1] = Point2(0.5 * dist, -0.5 * dist)
        landmarks[2] = Point2(1.5 * dist, 0.5 * dist)
        landmarks[3] = Point2(-0.5 * dist, 0.5 * dist)
        landmarks[4] = Point2(0.5 * dist, 1.5 * dist)

    if useLandMark:
        for i in range(Ls):
                initial.insert(L(i + 1), landmarks[i])
        
    

    for i in range(N - 1): 

        nImu += 1
        nOpt += 1
        
        odometry = Pose2(odomData[i])
        graph.add(BetweenFactorPose2(X(nImu - 1), X(nImu), odometry, ODOMETRY_NOISE))
        
        curPose = curPose.compose(odometry)
        initial.insert(X(nOpt), curPose)

        # if(i < (N // 4) and useLandMark):
        if(useLandMark):

            for j in range(Ls):
            
                pose_gt = Pose2(*gt_data[i])
                bearing = pose_gt.bearing(landmarks[j])
                dst = pose_gt.range(landmarks[j])

                # luego añadir ruido aquí
                # bearing_noisy = bearing + ruido
                # dist_noisy = dist + ruido
                if(dst < dist):
                    graph.add(BearingRangeFactor2D(X(nImu), L(j + 1), bearing, dst, LANDMARK_NOISE))




    print(graph)
        

    loop_noise = noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.02]))
    first = Pose2(*gt_data[0])
    last  = Pose2(*gt_data[N-1])
    loop_meas = first.between(last)


    graph.add(
        BetweenFactorPose2(
            X(1),          # primera pose
            X(N),        # última pose
            loop_meas,
            loop_noise
        )
    )

    params = gtsam.LevenbergMarquardtParams()
    optimizer = LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    print("\nFinal Result:\n{}".format(result))




    marginals = Marginals(graph, result)

    optimized = np.zeros((N, 3))

    for i in range(1, N + 1):
        # gtsam_plot.plot_pose2(0, result.atPose2(X(i)), 0.5,
        #                       marginals.marginalCovariance(X(i)))
        pose = result.atPose2(X(i))
        optimized[i-1] = [pose.x(), pose.y(), pose.theta()]

    # if(useLandMark):
    #     for i in range(Ls):
    #         x, y = result.atPoint2(L(i + 1))
    #         plt.scatter(x, y, c='r', marker='*', s=200, label="Landmark")
    
    print("Graph error before:", graph.error(initial))
    print("Graph error after:", graph.error(result))
    rmse_full = rmse_se2(gt_data, optimized)
    print("RMSE SE2:", rmse_full)

        

    plt.figure()

    plt.plot(gt_data[:,0], gt_data[:,1], 'g-', label="Ground Truth")
    plt.plot(optimized[:,0], optimized[:,1], 'b-', label="Optimized")

    if(useLandMark):
        for i in range(Ls):
            # x, y = result.atPoint2(L(i + 1))
            x, y = landmarks[i]
            plt.scatter(x, y, c='r', marker='*', s=200, label="Landmark")
    



    plt.axis('equal')
    # plt.legend()
    plt.show()

if __name__ == "__main__":
    main()