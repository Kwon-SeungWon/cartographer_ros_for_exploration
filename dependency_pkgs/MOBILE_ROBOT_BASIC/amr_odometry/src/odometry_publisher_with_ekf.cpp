#include "redbot02_odometry/odometry_publisher.h"

Redbot::ToOdom::ToOdom():pnh("~"), loop_rate(20) 
{
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    setSubscriber();
    setParam();
    init();
}

Redbot::ToOdom::~ToOdom()
{

}

void Redbot::ToOdom::setParam()
{
    robot_state = new RobotState{};
    robot_name = "";
    pnh.param("enc_vel_gain", enc_vel_gain,1.0);
    pnh.param("enc_wvel_gain", enc_wvel_gain,1.0);
    pnh.param("read_rate", read_rate,20);
}

void::Redbot::ToOdom::setSubscriber()
{
    enc_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("enc_vel", 10, &Redbot::ToOdom::encVelCallback, this);
}

void Redbot::ToOdom::init()
{

    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom_tf.header.stamp = ros::Time::now();;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);

    odom.pose.pose.orientation = odom_quat;
    odom_tf.transform.rotation = odom_quat;

    //motor = new Motor{0,};
    //read_thread = boost::thread(&Redbot::ToOdom::readThread, this, read_rate);
}

void Redbot::ToOdom::encVelCallback(const geometry_msgs::TwistConstPtr& msgs)
{
  enc_vel = *msgs; // enc_vel 변수에 메시지 할당
  //ROS_INFO("%f, %f", enc_vel.linear.x, enc_vel.angular.z);
  ros::Time odom_current_time = ros::Time::now();
  current_time = odom_current_time.toSec();
  dt = current_time - last_time;
 
  dx   = (enc_vel.linear.x * cos(robot_state->yaw) - enc_vel.linear.y * sin(robot_state->yaw)) * dt;
  dy   = (enc_vel.linear.x * sin(robot_state->yaw) + enc_vel.linear.y * cos(robot_state->yaw)) * dt;
  dyaw =  enc_vel.angular.z * dt;

  robot_state->x += dx;
  robot_state->y += dy;
  robot_state->yaw += dyaw;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_state->yaw);

  odom.header.stamp = odom_current_time;
  odom.header.frame_id = "odom";    

  odom.pose.pose.position.x = robot_state->x;
  odom.pose.pose.position.y = robot_state->y;
  odom.pose.pose.position.z = robot_state->z;
  odom.pose.pose.orientation = odom_quat;

  odom.pose.covariance = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01};

  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear = enc_vel.linear;
  odom.twist.twist.angular = enc_vel.angular;

  odom.twist.covariance = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01};

  odom_pub_.publish(odom);

//////////// If you use EKF, you have to disable below code /////////////

//   odom_tf.header.frame_id = "odom";
//   odom_tf.child_frame_id = "base_footprint";
//   odom_tf.header.stamp = odom_current_time;

//   odom_tf.transform.translation.x = robot_state->x;
//   odom_tf.transform.translation.y = robot_state->y;
//   odom_tf.transform.translation.z = robot_state->z;
//   odom_tf.transform.rotation = odom_quat;
//   odom_tf_broadcaster_.sendTransform(odom_tf);

  last_time = current_time;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
    ros::Time::init();
	Redbot::ToOdom toodom;
	
    while (ros::ok())
    {
        ros::spinOnce(); // 콜백 함수가 호출될 수 있도록 함
        toodom.loop_rate.sleep(); // 설정한 주기에 맞춰 대기
        //ROS_INFO("Odom Complete");
    }

    return 0;
}