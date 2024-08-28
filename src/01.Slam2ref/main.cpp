//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"

using namespace gtsam;

int main()//int argc, char** argv)
{
//    ros::init(argc, argv, "main");
    logInfo_cout("\033[1;32m----> Slam2ref starts.\033[0m");

    Slam2ref slam2Ref;
    slam2Ref.run();

    logInfo_cout("\033[1;32m----> Slam2ref done.\033[0m");
    return 0;
}
