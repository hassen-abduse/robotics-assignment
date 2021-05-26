#include <functional>
#include "gazebo/transport/transport.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "arm_lib/angles.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// Safety check
			if (_parent->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, MyRobot Plugin not loaded\n";
				return;
			}

			// Store the pointer to the model
			this->model = _parent;

			// // instantiate the joint controller
			this->jointController = this->model->GetJointController();

			// instantiate the angles publisher node
			ros::NodeHandle node;
			this->anglePublisher = node.advertise<arm_lib::angles>("arm/current_joint_angles", 1000);

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client",
						  ros::init_options::NoSigintHandler);
			}
			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
				ros::SubscribeOptions::create<arm_lib::angles>(
					"/" + this->model->GetName() + "/angle_cmd",
					1,
					boost::bind(&ModelPush::OnRosMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);

			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&ModelPush::QueueThread, this));

			this->publisherThread = std::thread(std::bind(&ModelPush::publishCurrentAngles, this));
		}

		/// \brief ROS helper function that processes messages
	private:
		void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

	private:
		void OnRosMsg(const arm_lib::anglesConstPtr _msg)
		{
			this->updateJointAngles(_msg->joint1, _msg->joint2, _msg->joint3, _msg->joint4);
		}

	private:
		void updateJointAngles(double j1, double j2, double j3, double j4)
		{

			std::string chasisArm1Joint = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			std::string arm1Arm2Joint = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2Arm3Joint = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3Arm4Joint = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			// change to radian
			j1 = j1 * M_PI / 180.0;
			j2 = j2 * M_PI / 180.0;
			j3 = j3 * M_PI / 180.0;
			j4 = j4 * M_PI / 180.0;

			common::PID joint1PID = common::PID(10.0, 1.0, 7.0);
			this->jointController->SetPositionPID(chasisArm1Joint, joint1PID);
			this->jointController->SetPositionTarget(chasisArm1Joint, j1);

			common::PID joint2PID = common::PID(15.0, 2.0, 10.0);
			this->jointController->SetPositionPID(arm1Arm2Joint, joint2PID);
			this->jointController->SetPositionTarget(arm1Arm2Joint, j2);

			common::PID joint3PID = common::PID(12.0, 1.5, 8.8);
			this->jointController->SetPositionPID(arm2Arm3Joint, joint3PID);
			this->jointController->SetPositionTarget(arm2Arm3Joint, j3);

			common::PID joint4PID = common::PID(10.0, 1.0, 7.4);
			this->jointController->SetPositionPID(arm3Arm4Joint, joint4PID);
			this->jointController->SetPositionTarget(arm3Arm4Joint, j4);

			this->jointController->Update();
		}

	private:
		void publishCurrentAngles()
		{
			while (ros::ok())
			{
				double joint1 = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position();

				double joint2 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position();

				double joint3 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position();

				double joint4 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position();

				// change to radian to degree
				joint1 = joint1 * 180.0 / M_PI;

				joint2 = joint2 * 180.0 / M_PI;

				joint3 = joint3 * 180.0 / M_PI;

				joint4 = joint4 * 180.0 / M_PI;

				arm_lib::angles joint_angles;
				joint_angles.joint1 = joint1;
				joint_angles.joint2 = joint2;
				joint_angles.joint3 = joint3;
				joint_angles.joint4 = joint4;
				ros::Rate loop_rate(10);
				anglePublisher.publish(joint_angles);
				ros::spinOnce();
			}
		}

	private:
		physics::ModelPtr model;

	private:
		physics::JointControllerPtr jointController;

		/// \brief A node use for ROS transport
	private:
		std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
	private:
		ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
	private:
		ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
	private:
		std::thread rosQueueThread;

	private:
		std::thread publisherThread;

	private:
		ros::Publisher anglePublisher;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}