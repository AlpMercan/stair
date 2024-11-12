#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <vector>

namespace stair_detector
{

struct CostmapPoint {
    double x;
    double y;
    bool active;
    
    CostmapPoint(double _x, double _y) : x(_x), y(_y), active(true) {}
};

class StairLayer : public costmap_2d::Layer
{
public:
    StairLayer() : last_sensor_value_(0.0), navigation_active_(false) {}

    virtual void onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        // Get parameters
        nh.param("threshold", threshold_, 50.0);
        nh.param("costmap_value", costmap_value_, 254);
        nh.param("region_size", region_size_, 0.3);
        nh.param("forward_distance", forward_distance_, 0.5);

        // Subscribe to sensor value and move_base status
        sensor_sub_ = nh.subscribe("/stair_detection", 1, 
            &StairLayer::sensorCallback, this);
        status_sub_ = nh.subscribe("/move_base/status", 1,
            &StairLayer::statusCallback, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &StairLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                            double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
            return;

        last_robot_x_ = robot_x;
        last_robot_y_ = robot_y;
        last_robot_yaw_ = robot_yaw;

        if (last_sensor_value_ > threshold_ && navigation_active_)
        {
            // Calculate the point in front of the robot
            double front_x = robot_x + forward_distance_ * cos(robot_yaw);
            double front_y = robot_y + forward_distance_ * sin(robot_yaw);
            
            // Add new point if threshold exceeded
            costmap_points_.push_back(CostmapPoint(front_x, front_y));
        }

        // Update bounds to include all active costmap points
        bool first = true;
        for (const auto& point : costmap_points_)
        {
            if (!point.active) continue;
            
            if (first)
            {
                *min_x = point.x - region_size_;
                *min_y = point.y - region_size_;
                *max_x = point.x + region_size_;
                *max_y = point.y + region_size_;
                first = false;
            }
            else
            {
                *min_x = std::min(*min_x, point.x - region_size_);
                *min_y = std::min(*min_y, point.y - region_size_);
                *max_x = std::max(*max_x, point.x + region_size_);
                *max_y = std::max(*max_y, point.y + region_size_);
            }
        }
    }

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, 
                           int max_i, int max_j)
    {
        if (!enabled_)
            return;

        unsigned int size_x = master_grid.getSizeInCellsX();
        unsigned int size_y = master_grid.getSizeInCellsY();

        // Update costs for all active points
        for (const auto& point : costmap_points_)
        {
            if (!point.active) continue;

            unsigned int mx, my;
            if (master_grid.worldToMap(point.x, point.y, mx, my))
            {
                double resolution = master_grid.getResolution();
                int cell_radius = static_cast<int>(region_size_ / resolution);

                for (int j = -cell_radius; j <= cell_radius; j++)
                {
                    for (int i = -cell_radius; i <= cell_radius; i++)
                    {
                        unsigned int cur_mx = mx + i;
                        unsigned int cur_my = my + j;

                        if (cur_mx < size_x && cur_my < size_y)
                        {
                            double distance = hypot(i * resolution, j * resolution);
                            if (distance <= region_size_)
                            {
                                master_grid.setCost(cur_mx, cur_my, costmap_value_);
                            }
                        }
                    }
                }
            }
        }
    }

private:
    void sensorCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        last_sensor_value_ = msg->data;
        ROS_INFO("Received sensor value: %f", last_sensor_value_);
    }

    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        if (msg->status_list.empty())
        {
            navigation_active_ = false;
            costmap_points_.clear();
            return;
        }

        // Get the latest status
        const auto& status = msg->status_list.back();
        
        // Check if navigation is active
        navigation_active_ = (status.status == actionlib_msgs::GoalStatus::ACTIVE);
        
        // Clear points if navigation is completed or aborted
        if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED ||
            status.status == actionlib_msgs::GoalStatus::ABORTED)
        {
            costmap_points_.clear();
            ROS_INFO("Navigation completed or aborted, clearing costmap points");
        }
    }

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    double threshold_;
    int costmap_value_;
    double region_size_;
    double forward_distance_;
    double last_sensor_value_;
    double last_robot_x_, last_robot_y_, last_robot_yaw_;
    bool navigation_active_;
    std::vector<CostmapPoint> costmap_points_;
    
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
    ros::Subscriber sensor_sub_;
    ros::Subscriber status_sub_;
};

} // namespace stair_detector

PLUGINLIB_EXPORT_CLASS(stair_detector::StairLayer, costmap_2d::Layer)
