//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"


/*
 * PART 0. Init
 */

void Slam2ref::initOptimizer()
{
    // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
    // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step.
    // A parameter structure is available that allows the user to set various properties, such as the relinearization threshold and type of linear solver.
    // For this example, we set the relinearization threshold small so the iSAM2 result
    // will approach the batch result.  : https://github.com/devbharat/gtsam/blob/master/examples/VisualISAM2Example.cpp

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    if(print_ISAM_config_param)
        parameters.print();
    isam = new ISAM2(parameters);
}



void Slam2ref::initNoiseConstants()
{
    const double PriorNoiseValue = 1e-102; // originally 1e-12
    const double OdomNoiseValue = 1e-4;
    const double RobustNoiseScore = robust_noise_model_value_; // MV original 0.5

    // Variances Vector6 order means
    // : rad*rad, rad*rad, rad*rad, meter*meter, meter*meter, meter*meter
    {
        gtsam::Vector Vector6(6);
        // Vector6 << 1e-102, 1e-102, 1e-102, 1e-102, 1e-102, 1e-102;
        Vector6 << Eigen::VectorXd::Constant(6, PriorNoiseValue);
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        // Vector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
        Vector6 << Eigen::VectorXd::Constant(6, OdomNoiseValue);
        odomNoise = noiseModel::Diagonal::Variances(Vector6);
    }

    {
        gtsam::Vector Vector6(6);
        Vector6 << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e8, 1e8, 1e8;
        // Vector6 << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        largeNoise = noiseModel::Diagonal::Variances(Vector6);
    }

    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << Eigen::VectorXd::Constant(6, RobustNoiseScore);
//    robustNoiseVector6 << RobustNoiseScore, RobustNoiseScore, RobustNoiseScore, RobustNoiseScore, RobustNoiseScore, RobustNoiseScore;// MV defined in this way it definetely does not work correctly.
    if(mEstimator_ == "Cauchy")
    {
        robustNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Cauchy::Create( 1),            // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough. 
                gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    }
    else
    {
        robustNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create( 1.345*RobustNoiseScore),            // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough. //MV: for my data, Hubert with variance value = 0.00005  is better than Cauchy
                gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    }

}
