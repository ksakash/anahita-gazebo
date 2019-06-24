#include <anahita_plugins/AUVPlugin.hh>

using namespace std;

namespace gazebo
{
AUV::AUV() {
    std::cout << "AUV plugin started successfully" << std::endl;
    gate_center[0] = 2.686;
    gate_center[1] = -0.033;
    gate_center[2] = -1.72;
}
AUV::~AUV() {}

void AUV::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    this->model_ = _model;
    this->sdf_ = _sdf;
    
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle("anahita"));
    pub_ = nh_->advertise<geometry_msgs::Point>("/anahita/distance", 10);
    this->rosQueueThread = std::thread(std::bind(&AUV::QueueThread, this));
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AUV::Update, this));
}

/// \brief ROS helper function that processes messages
void AUV::QueueThread() {
  static const double timeout = 0.01;
  while (this->nh_->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void AUV::Update() {
    ignition::math::Vector3d vehicle_pose_ = this->model_->WorldPose().Pos();
    vehicle_pose.x = vehicle_pose_.X();
    vehicle_pose.y = vehicle_pose_.Y();
    vehicle_pose.z = vehicle_pose_.Z();
    // std::cout << "pose: " << vehicle_pose[0] << " " << vehicle_pose[1] << " " << vehicle_pose[2] << std::endl;
    pub_.publish(vehicle_pose);
}

GZ_REGISTER_MODEL_PLUGIN(AUV)

}