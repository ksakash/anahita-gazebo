#ifndef AUVPLUGIN_HH
#define AUVPLUGIN_HH

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/common/Plugin.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>

namespace gazebo
{
    class AUV : public ModelPlugin {
        public: AUV();
        public: ~AUV();

        public: virtual void Load(physics::ModelPtr, sdf::ElementPtr);
        public: virtual void Update();
        public: void QueueThread();

        // Pointer to the model
        private: physics::ModelPtr model_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // ROS node
        private: std::unique_ptr<ros::NodeHandle> nh_;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;

        // subscriber to the topics published by the pwm_publisher
        protected: ros::Subscriber sub_;
        protected: ros::Publisher pub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: event::ConnectionPtr update_connection_;

        private: Eigen::Vector3d gate_center;
        private: geometry_msgs::Point vehicle_pose;
    };
}

#endif // !AUVPLUGIN_HH
