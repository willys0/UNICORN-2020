#ifndef __VELOCITY_LAYER_H_
#define __VELOCITY_LAYER_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <tracking_layers/VelocityPluginConfig.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>

#include <mutex>

namespace tracking_layers {
class VelocityLayer : public costmap_2d::CostmapLayer {
public:
    VelocityLayer();

    virtual void onInitialize();

    virtual void matchSize();

    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);


    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void reset() { onInitialize(); }


    virtual bool isDiscretized() {
        return false;
    }

protected:
    void velocityFill(double x1, double y1, double x2, double y2, double* min_x, double* min_y, double* max_x, double* max_y);

    void dynamicObjectCallback(const costmap_converter::ObstacleArrayMsgConstPtr& msg);

private:
    void reconfigureCB(tracking_layers::VelocityPluginConfig &config, uint32_t level);

    ros::NodeHandle nh_;
    ros::Subscriber dynObjSub_;

    dynamic_reconfigure::Server<tracking_layers::VelocityPluginConfig>* dsrv_;

    bool rolling_window_;

    std::vector<costmap_converter::ObstacleMsg> obstacles_;

    double a_;
    double width_;
    double mag_;

    ros::Duration keep_time_;

    std::mutex mutex_;


};
};

#endif // __VELOCITY_LAYER_H_
