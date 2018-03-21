#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <math.h>
#include "Measurement.h"

const float distanceLimitLower = 0.4;
const float distanceLimitUpper = 1.5;
const int obstacleHoleMargin = 5;
const int standardPitch = 1500;
const int standardRoll = 1500;
const int maxPitchChange = 500;
const int maxRollChange = 500;
const int minPitchChange = 50;
const int minRollChange = 50;

class ObstacleAvoider {
private:
float limitLower;
bool activated;
ros::Subscriber sub;
Measurement data[500];
ros::Publisher pub_rc;
float rad2Deg(float rad);
public:
ObstacleAvoider();
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

ObstacleAvoider::ObstacleAvoider(){
        ROS_INFO("Version 1.0");
        limitLower = sin(M_PI/4) * distanceLimitLower;

        ros::NodeHandle n;
        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ObstacleAvoider::scanCallback, this);
        pub_rc= n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
        activated = false;

}

float ObstacleAvoider::rad2Deg(float rad){
        return ((rad)*180./M_PI);
}

void ObstacleAvoider::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
        float angle_tmp=scan->angle_min;
        float inc = scan->angle_increment;


        int n = 0;

        Measurement * obstacle = NULL;
        int margin = obstacleHoleMargin;

        // iterate over the entire scan
        for( std::vector<float>::const_iterator it=scan->ranges.begin(); it!=scan->ranges.end(); it++, angle_tmp=angle_tmp+inc )
        {
                float x = (*it) * sinf(angle_tmp);
                float y = (*it) * cosf(angle_tmp);

                if((fabs(x)>limitLower || fabs(y)>limitLower) && (*it)<distanceLimitUpper)
                {

                        data[n].distance = *it;
                        data[n].angle = angle_tmp;
                        data[n].obstacle = false;

                        if(obstacle == NULL) {
                                data[n].obstacle = true;
                                obstacle = &data[n];
                        } else {
                                if(*it < obstacle->distance) {
                                        data[n].obstacle = true;
                                        obstacle->obstacle=false;
                                        obstacle = &data[n];
                                }
                        }

                        margin=obstacleHoleMargin;

                } else {
                        data[n].distance = 0;
                        data[n].angle = 0;
                        data[n].obstacle = false;

                        if(obstacle!=NULL) {
                                if(margin>0) {
                                        margin--;
                                }else{
                                        obstacle = NULL;
                                }
                        }
                }
                n++;

        }

        int num = 1;
        float minRoll = 0, maxRoll = 0, minPitch = 0, maxPitch = 0;

        for(int i=0; i<360; i++) {

                if(data[i].obstacle) {
                        ROS_INFO("%d Obstacle detected! Angle: %f, Distance, %f", num, rad2Deg(data[i].angle), data[i].distance);
                        num++;


                        float proximity = ((distanceLimitUpper - data[i].distance)/(distanceLimitUpper-distanceLimitLower));

                        if(proximity>1) {
                                proximity = 1;
                        }

                        float pitch = proximity * maxPitchChange * cosf(data[i].angle);
                        float roll = proximity * maxRollChange * -sinf(data[i].angle);

                        maxPitch = fmax(pitch,maxPitch);
                        minPitch = fmin(pitch,minPitch);
                        maxRoll = fmax(roll,maxRoll);
                        minRoll = fmin(roll,minRoll);
                }
        }

        float roll = maxRoll+minRoll;
        float pitch = maxPitch+minPitch;

        if(fabs(roll)<minRollChange) roll=0;
        if(fabs(pitch)<minPitchChange) pitch=0;
        //ROS_INFO("Min distance: %f, Angle: %f", closest, angle);


        mavros_msgs::OverrideRCIn override_RC;
        for (int i = 0; i < 8; i++) {
                override_RC.channels[i] = 0;                 //65535;
        }

        if(roll>0 || pitch > 0) {

                override_RC.channels[0]= standardRoll+roll;
                override_RC.channels[1]= standardPitch+pitch;
                ROS_WARN("Publishing Overwriting Message %i, %i",override_RC.channels[0],override_RC.channels[1]);

                activated = true;
                pub_rc.publish(override_RC);
        } else if(activated) {
                pub_rc.publish(override_RC);
                activated=false;
        }



}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "main");
        ObstacleAvoider avoider;

        ros::spin();

        return 0;
}
