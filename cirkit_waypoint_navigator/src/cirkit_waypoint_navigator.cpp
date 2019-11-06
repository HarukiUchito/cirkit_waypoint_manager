/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <laser_geometry/laser_geometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cirkit_waypoint_navigator/TeleportAbsolute.h>
#include <map_selector/change_map.h>
#include <map_selector/transform_gps_pose.h>

#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "ros_colored_msg.h" // FIXME: this header depend ROS, but exclude ros header. Now must be readed after #include"ros/ros.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

namespace RobotBehaviors
{
enum State
{
    WAYPOINT_NAV,
    WAYPOINT_REACHED_GOAL,
    INIT_NAV,
    WAYPOINT_NAV_PLANNING_ABORTED,
    WAYPOINT_REACHED_STOP_POINT,
    CHANGE_MAP_HERE,
};
}

class WayPoint
{
public:
    WayPoint();
    WayPoint(move_base_msgs::MoveBaseGoal goal, int area_type, double reach_threshold, int map_number)
        : goal_(goal), area_type_(area_type), reach_threshold_(reach_threshold), map_number_(map_number)
    {
    }
    ~WayPoint() {} // FIXME: Don't declare destructor!!
    bool isSearchArea()
    {
        if (area_type_ == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool isStopPoint()
    {
        if (area_type_ == 2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool isChangeMapPoint()
    {
        if (area_type_ == 3)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    move_base_msgs::MoveBaseGoal goal_;
    int area_type_;
    double reach_threshold_;
    int map_number_;
};

class CirkitWaypointNavigator
{
public:
    CirkitWaypointNavigator()
        : ac_("move_base", true),
          rate_(10)
    {
        robot_behavior_state_ = RobotBehaviors::INIT_NAV;
        std::string filename;

        ros::NodeHandle n("~");
        n.param<std::string>("waypointsfile",
                             filename,
                             ros::package::getPath("cirkit_waypoint_navigator") + "/waypoints/garden_waypoints.csv"); // FIXME: Don't find!

        n.param("start_waypoint", target_waypoint_index_, 0);

        ROS_INFO("[Waypoints file name] : %s", filename.c_str());
        next_waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/next_waypoint", 1);
        ROS_INFO("Reading Waypoints.");
        readWaypoint(filename.c_str());
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
    }

    ~CirkitWaypointNavigator()
    {
        this->cancelGoal();
    }

    void sendNewGoal(geometry_msgs::Pose pose)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose = pose;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ac_.sendGoal(goal);
        now_goal_ = goal.target_pose.pose;
    }

    void sendNextWaypointMarker(const geometry_msgs::Pose waypoint,
                                int target_object_mode)
    {
        visualization_msgs::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "map";
        waypoint_marker.header.stamp = ros::Time();
        waypoint_marker.id = 0;
        waypoint_marker.type = visualization_msgs::Marker::ARROW;
        waypoint_marker.action = visualization_msgs::Marker::ADD;
        waypoint_marker.pose = waypoint;
        waypoint_marker.pose.position.z = 0.3;
        waypoint_marker.scale.x = 0.8;
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.1;
        waypoint_marker.color.a = 0.7;
        waypoint_marker.color.r = 0.05 + 1.0 * (float)target_object_mode;
        waypoint_marker.color.g = 0.80;
        waypoint_marker.color.b = 0.05 + 1.0 * (float)target_object_mode;
        next_waypoint_marker_pub_.publish(waypoint_marker);
    }

    void cancelGoal()
    {
        ROS_INFO("cancelGoal() is called !!");
        ac_.cancelGoal();
    }

    int readWaypoint(std::string filename)
    {
        const int rows_num = 9; // x, y, z, Qx,Qy,Qz,Qw, area_type, reach_threshold
        boost::char_separator<char> sep(",", "", boost::keep_empty_tokens);
        std::ifstream ifs(filename.c_str());
        std::string line;
        while (ifs.good())
        {
            getline(ifs, line);
            if (line.empty())
                break;
            tokenizer tokens(line, sep);
            std::vector<double> data;
            tokenizer::iterator it = tokens.begin();
            for (; it != tokens.end(); ++it)
            {
                std::stringstream ss;
                double d;
                ss << *it;
                ss >> d;
                data.push_back(d);
            }
            if (data.size() < 9)
            {
                ROS_ERROR("Row size is mismatch!!");
                return -1;
            }
            else
            {
                // x y z qx qy qz qw area_type reach_th
                move_base_msgs::MoveBaseGoal waypoint;
                waypoint.target_pose.pose.position.x = data[0];
                waypoint.target_pose.pose.position.y = data[1];
                waypoint.target_pose.pose.position.z = data[2];
                waypoint.target_pose.pose.orientation.x = data[3];
                waypoint.target_pose.pose.orientation.y = data[4];
                waypoint.target_pose.pose.orientation.z = data[5];
                waypoint.target_pose.pose.orientation.w = data[6];
                int map_num = -1;
                if (data.size() >= 10)
                    map_num = data[9];
                waypoints_.push_back(WayPoint(waypoint, (int)data[7], data[8] / 2.0, map_num));
            }
        }
        return 0;
    }

    WayPoint getNextWaypoint()
    {
        ROS_INFO_STREAM("Next Waypoint : " << target_waypoint_index_);
        WayPoint next_waypoint = waypoints_[target_waypoint_index_];
        target_waypoint_index_++;
        return next_waypoint;
    }

    bool isFinalGoal()
    {
        if ((target_waypoint_index_) == ((int)waypoints_.size()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    double calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
    {
        return sqrt(pow((a.position.x - b.position.x), 2.0) + pow((a.position.y - b.position.y), 2.0));
    }

    geometry_msgs::Pose getMapPoseFromGPSPose(geometry_msgs::Pose g_pose)
    {
        map_selector::transform_gps_pose tr_gps;
        tr_gps.request.gps_pose = g_pose;
        if (tr_gps_cli_.call(tr_gps))
        {
            ROS_INFO("Succeeded to call transform_gps_pose");
            return tr_gps.response.map_pose;
        }
        else
        {
            ROS_ERROR("failed to call transform_gps_pose");
            exit(0);
        }
        /*
        geometry_msgs::PoseStamped m_pose;
        try
        {
            geometry_msgs::PoseStamped source;
            source.header.frame_id = "gps";
            source.header.stamp = ros::Time(0);
            source.pose = g_pose;
            listener_.transformPose("map", source, m_pose);
            ROS_INFO("gps pose is transformed to (x: %f, y: %f) in map frame",
                m_pose.pose.position.x,
                m_pose.pose.position.y
            );
        }
        catch (tf::TransformException& e)
        {
            ROS_ERROR("WaypointNavigator: failed to transform gps pose to map frame %s", e.what());
            exit(0);
        }
        return m_pose.pose;
        */
    }

    // 通常のwaypointの場合
    void setNextGoal(WayPoint waypoint)
    {
        reach_threshold_ = waypoint.reach_threshold_;
        geometry_msgs::Pose m_pose = getMapPoseFromGPSPose(waypoint.goal_.target_pose.pose);
        this->sendNextWaypointMarker(m_pose, 0); // 現在目指しているwaypointを表示する
        this->sendNewGoal(m_pose);
    }

    double getReachThreshold()
    {
        return reach_threshold_;
    }

    geometry_msgs::Pose getRobotCurrentPosition()
    {
        // tfを使ってロボットの現在位置を取得する
        tf::StampedTransform transform;
        geometry_msgs::Pose pose;
        try
        {
            listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        //ROS_INFO_STREAM("c)x :" << pose.position.x << ", y :" << pose.position.y);
        return pose;
    }

    geometry_msgs::Pose getNowGoalPosition()
    {
        //ROS_INFO_STREAM("g)x :" << now_goal_.position.x << ", y :" << now_goal_.position.y);
        return now_goal_;
    }

    void run()
    {
        ros::ServiceClient cli_ch_map = nh_.serviceClient<map_selector::change_map>("map_selector/change_map");
        tr_gps_cli_ = nh_.serviceClient<map_selector::transform_gps_pose>("map_selector/transform_gps_pose");
        robot_behavior_state_ = RobotBehaviors::INIT_NAV;
        while (ros::ok())
        {
            WayPoint next_waypoint = this->getNextWaypoint();
            ROS_GREEN_STREAM("Next WayPoint is got");
            ROS_INFO("gps frame, x: %f, y: %f",
                next_waypoint.goal_.target_pose.pose.position.x,
                next_waypoint.goal_.target_pose.pose.position.y
            );

            if (next_waypoint.isChangeMapPoint())
            {
                map_selector::change_map ch_map;
                ch_map.request.map_number = next_waypoint.map_number_;
                if (cli_ch_map.call(ch_map))
                    ROS_INFO("Succeeded to call change_map");
                else
                {
                    ROS_ERROR("failed to call change_map");
                }
            }
            
            ROS_INFO("Go next_waypoint.");
            this->setNextGoal(next_waypoint);
            robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
            
            ros::Time begin_navigation = ros::Time::now(); // 新しいナビゲーションを設定した時間
            ros::Time verbose_start = ros::Time::now();
            double last_distance_to_goal = 0;
            double delta_distance_to_goal = 1.0; // 0.1[m]より大きければよい
            while (ros::ok())
            {
                geometry_msgs::Pose robot_current_position = this->getRobotCurrentPosition();                 // 現在のロボットの座標
                geometry_msgs::Pose now_goal_position = this->getNowGoalPosition();                           // 現在目指している座標
                double distance_to_goal = this->calculateDistance(robot_current_position, now_goal_position); // 現在位置とwaypointまでの距離を計算
                // ここからスタック(Abort)判定。
                delta_distance_to_goal = last_distance_to_goal - distance_to_goal; // どれだけ進んだか
                if (delta_distance_to_goal < 0.1)
                { // 進んだ距離が0.1[m]より小さくて
                    ros::Duration how_long_stay_time = ros::Time::now() - begin_navigation;
                    if (how_long_stay_time.toSec() > 10.0)
                    { // 90秒間経過していたら
                        if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV)
                        {
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED; // プランニング失敗とする
                            break;
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    { // 30秒おきに進捗を報告する
                        ros::Duration verbose_time = ros::Time::now() - verbose_start;
                        if (verbose_time.toSec() > 30.0)
                        {
                            ROS_INFO_STREAM("Waiting Abort: passed 30s, Distance to goal: " << distance_to_goal);
                            verbose_start = ros::Time::now();
                        }
                    }
                }
                else
                { // 0.1[m]以上進んでいればOK
                    last_distance_to_goal = distance_to_goal;
                    begin_navigation = ros::Time::now();
                }
                // waypointの更新判定
                if (distance_to_goal < this->getReachThreshold())
                { // 目標座標までの距離がしきい値になれば
                    ROS_INFO_STREAM("Distance: " << distance_to_goal);
                    if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV)
                    {
                        if (next_waypoint.isStopPoint())
                        {
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_STOP_POINT;
                        }
                        else
                        {
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_GOAL;
                        }
                        break;
                    }
                    else
                    {
                        break;
                    }
                }
                rate_.sleep();
                ros::spinOnce();
            }
            switch (robot_behavior_state_)
            {
                case RobotBehaviors::WAYPOINT_REACHED_GOAL:
                {
                    ROS_INFO("WAYPOINT_REACHED_GOAL");
                    if (this->isFinalGoal())
                    {                       // そのwaypointが最後だったら
                        this->cancelGoal(); // ゴールをキャンセルして終了
                        return;
                    }
                    break;
                }
                case RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED:
                {
                    ROS_INFO("!! WAYPOINT_NAV_PLANNING_ABORTED !!");
                    this->cancelGoal(); // 今のゴールをキャンセルして
                    //this->tryBackRecovery(); // 1mくらい戻ってみて
                    target_waypoint_index_ -= 1; // waypoint indexを１つ戻す
                    break;
                }
                case RobotBehaviors::WAYPOINT_REACHED_STOP_POINT:
                {
                    ROS_INFO("WAYPOINT_REACHED_GOAL");
                    char c;
                    std::cin >> c;
                    break;
                }
                default:
                {
                    ROS_WARN_STREAM("!! UNKNOWN STATE !!");
                    break;
                }
            }
            rate_.sleep();
            ros::spinOnce();
        } // while(ros::ok())
    }

private:
    MoveBaseClient ac_;
    RobotBehaviors::State robot_behavior_state_;
    ros::Rate rate_;
    std::vector<WayPoint> waypoints_;
    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    ros::ServiceClient tr_gps_cli_;

    int target_waypoint_index_;                                        // 次に目指すウェイポイントのインデックス
    double reach_threshold_;                                           // 今セットされてるゴール（waypointもしくは探索対象）へのしきい値
    geometry_msgs::Pose now_goal_;                                     // 現在目指しているゴールの座標
    ros::Publisher cmd_vel_pub_;
    ros::Publisher next_waypoint_marker_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cirkit_waypoint_navigator");

    CirkitWaypointNavigator cirkit_waypoint_navigator;
    cirkit_waypoint_navigator.run();

    return 0;
}
