//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "0.Utils/utility.h"

#ifndef slam2ref_PCTRANSFORMATIONS_H
#define slam2ref_PCTRANSFORMATIONS_H


class PcTransformations {
public:
    static void transformPointCloudWithAngles(pcl::PointCloud<PointType>::Ptr cloud, float translation_x, float translation_y, float translation_z, float yaw, float pitch, float roll);


};


#endif //slam2ref_PCTRANSFORMATIONS_H
