#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

typedef struct{
    int id;
    double x, y, z;
    double qx, qy, qz, qw;
    double roll, pitch, yaw;
    double roll_deg, pitch_deg, yaw_deg;
}robot_state;

class robotWorker{
public:
    robotWorker();
    void odomCallback(const nav_msgs::Odometry& odom);
    void setMoveVector(float linear_x, int cnt);
    void timerCallback(const ros::TimerEvent&);

private:
    ros::Publisher twist_pub, signal_pub;
    ros::Subscriber odom_sub;
    ros::Timer timer;
    ros::NodeHandle nh;

    robot_state w1_state;
};

robotWorker::robotWorker(){
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    signal_pub = nh.advertise<std_msgs::Bool>("check_signal", 1000);
    odom_sub = nh.subscribe("base_pose_ground_truth", 1, &robotWorker::odomCallback, this);
    timer = nh.createTimer(ros::Duration(0.1), &robotWorker::timerCallback, this);

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twist_pub.publish(twist);

    //ロボットの番号をセット
    w1_state.id = 0;
}


void robotWorker::odomCallback(const nav_msgs::Odometry& odom){

    //クォータニオン宣言
    tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    //クォータニオンからオイラーオイラー角への変換
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //ラジアンから度への変換
    double roll_deg, pitch_deg, yaw_deg;
    roll_deg = roll * (180.0 / M_PI);
    pitch_deg = pitch * (180.0 / M_PI);
    yaw_deg = yaw * (180.0 / M_PI);

    //構造体へ代入
    w1_state.x = odom.pose.pose.position.x;
    w1_state.y = odom.pose.pose.position.y;
    w1_state.z = odom.pose.pose.position.z;
    w1_state.qx = odom.pose.pose.orientation.x;
    w1_state.qy = odom.pose.pose.orientation.y;
    w1_state.qz = odom.pose.pose.orientation.z;
    w1_state.qw = odom.pose.pose.orientation.w;
    w1_state.roll = roll;
    w1_state.pitch = pitch;
    w1_state.yaw = yaw;
    w1_state.roll_deg = roll_deg;
    w1_state.pitch_deg = pitch_deg;
    w1_state.yaw_deg = yaw_deg;

}

//一定周期で動作する関数
//この関数内でロボットを制御するプログラムを記述
void robotWorker::timerCallback(const ros::TimerEvent&){
    //ロボットの制御
    setMoveVector(0.2, 10);
    setMoveVector(-0.2, 10);

    //センサシグナルの配信
    std_msgs::Bool signal;
    signal.data = false;
    signal_pub.publish(signal);
}

void robotWorker::setMoveVector(float linear_x, int cnt){
    int i;
    geometry_msgs::Twist twist;
    ros::Rate loop_rate(10);

    for(i = 0;i < cnt; i++){
        twist.linear.x = linear_x;
        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    //ロボットのx, y, angleを出力
    ROS_INFO("x:%f", w1_state.x);
    ROS_INFO("y:%f", w1_state.y);
    ROS_INFO("yaw_deg:%f", w1_state.yaw_deg);

}

int main(int argc, char **argv){
    //ノードの登録
    ros::init(argc, argv, "control_robot");

    //stageシミュレーターが起動するまで待つ
    sleep(1);

    //オブジェクトの生成
    robotWorker worler1;

    ros::spin();
    return 0;
}
