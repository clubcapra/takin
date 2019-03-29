#include "ros/ros.h"
#include "std_srvs/Trigger.h"

bool estop = true;

bool toggleEstop(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
    const bool prev_estop = estop;

    estop = !estop;

    if(estop != prev_estop){
        res.success = static_cast<unsigned char>(true);
        res.message = "successfully toggle estop";
    } else {
        res.success = static_cast<unsigned char>(false);
        res.message = "failed toggle estop";
    }
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"takin_estop");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("takin_estop",toggleEstop);
    ros::spin();
    /*ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }*/
    return 0;
}


