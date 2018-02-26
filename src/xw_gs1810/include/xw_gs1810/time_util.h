//
// Created by junbochen on 17-2-23.
//

#include "ros/time.h"
#include "../../../../../../../../opt/ros/kinetic/include/ros/time.h"

#ifndef HAS_LOCAL_PLANNER_TIME_UTIL_H
#define HAS_LOCAL_PLANNER_TIME_UTIL_H

#endif //HAS_LOCAL_PLANNER_TIME_UTIL_H

    class Timer {
    public:
        Timer () {
            stime_ = ros::Time::now();
            etime_ = ros::Time::now();
        }
        void restart() {
            stime_ = ros::Time::now();
        }
        double stop() {
            etime_ = ros::Time::now();
            return (etime_ - stime_).toSec();
        }
        double dur() {
            return (etime_ - stime_).toSec();
        }
    private:
        ros::Time stime_;
        ros::Time etime_;
    };

