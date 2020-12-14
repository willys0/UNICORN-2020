#include <tracking_layers/velocity_layer.h>

#include <pluginlib/class_list_macros.h>

namespace tracking_layers {

VelocityLayer::VelocityLayer() : keep_time_(0.5) {

}

void VelocityLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    nh_ = nh;

    VelocityLayer::matchSize();
    current_ = true;

    rolling_window_ = layered_costmap_->isRolling();

    default_value_ = costmap_2d::FREE_SPACE;
    matchSize();

    dynObjSub_ = nh_.subscribe("/obstacles", 1, &VelocityLayer::dynamicObjectCallback, this);

    dsrv_ = new dynamic_reconfigure::Server<tracking_layers::VelocityPluginConfig>(nh_);
    dsrv_->setCallback(boost::bind(&VelocityLayer::reconfigureCB, this, _1, _2));

    enabled_ = true;
}


void VelocityLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}

void VelocityLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y) {
    if(!enabled_)
        return;

    mutex_.lock();

    if(rolling_window_)
        updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);

    useExtraBounds(min_x, min_y, max_x, max_y);

    memset(costmap_, default_value_, getSizeInCellsX() * getSizeInCellsY() * sizeof(char));

    current_ = true;

    for(auto& o : obstacles_) {
        if(o.polygon.points.size() == 0)
            continue;

        if(ros::Time::now() - o.header.stamp > keep_time_)
            continue;

        double avg_x, avg_y;

        // for(auto& p : o.polygon.points) {
        //     avg_x += p.x;
        //     avg_y += p.y;
        // }
        // avg_x /= o.polygon.points.size();
        // avg_y /= o.polygon.points.size();

        avg_x = o.polygon.points[o.polygon.points.size() / 2].x;
        avg_y = o.polygon.points[o.polygon.points.size() / 2].y;
        double e_x = avg_x + o.velocities.twist.linear.x * mag_;
        double e_y = avg_y + o.velocities.twist.linear.y * mag_;

        velocityFill(avg_x, avg_y, e_x, e_y, min_x, min_y, max_x, max_y);
    }

    mutex_.unlock();
}

void VelocityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if(!enabled_)
        return;

    mutex_.lock();

    for(int j = min_j; j < max_j; j++) {
        for(int i = min_i; i < max_i; i++) {
            int index = getIndex(i, j);

            master_grid.setCost(i,j,std::max(master_grid.getCost(i, j), costmap_[index]));
                //master_grid.setCost(i,j,costmap_[index]);
        }
    }

    mutex_.unlock();
}

void VelocityLayer::velocityFill(double x1, double y1, double x2, double y2, double* min_x, double* min_y, double* max_x, double* max_y) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double mag = std::sqrt(dx*dx + dy*dy);

    double nx = dx / mag;
    double ny = dy / mag;

    double th = M_PI / 2.0;
    double lx = nx * std::cos(th) - ny * std::sin(th);
    double ly = nx * std::sin(th) + ny * std::cos(th);

    for(double p = 0; p < mag; p += getResolution()) {

        for(double r = -width_; r <= width_; r += getResolution()) {

            double x = x1 + nx * p + lx * r;
            double y = y1 + ny * p + ly * r;

            unsigned int mx, my;
            if(worldToMap(x,y,mx,my)) {
                double d = std::sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
                int cost = std::max(0, (int)std::round(a_ / ( (fabs(r) / width_) * std::exp(1.0*d+0.001)) - 1.0));
                cost = std::min(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 2, cost);
                //setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
                setCost(mx, my, cost);
            }

            *min_x = std::min(*min_x, x);
            *max_x = std::max(*max_x, x);
            *min_y = std::min(*min_y, y);
            *max_y = std::max(*max_y, y);

        }
    }
}

void VelocityLayer::dynamicObjectCallback(const costmap_converter::ObstacleArrayMsgConstPtr& msg) {
    obstacles_ = msg->obstacles;
}

void VelocityLayer::reconfigureCB(tracking_layers::VelocityPluginConfig &config, uint32_t level) {
    a_ = config.a;
    width_ = config.width;
    mag_ = config.length_factor;
    keep_time_ = ros::Duration(config.keep_time);
}
}

PLUGINLIB_EXPORT_CLASS(tracking_layers::VelocityLayer, costmap_2d::Layer)
