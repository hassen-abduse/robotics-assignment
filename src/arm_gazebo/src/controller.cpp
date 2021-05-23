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

			ros::NodeHandle node;
			this->angle_publisher_ = node.advertise<arm_lib::angles>("current_joint_angles", 1000);
			
			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			std::string name1 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();

			this->jointController->SetPositionPID(name1, pid);

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

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			this->publish_joint_angles();
		}

	public:
		void OnRosMsg(const arm_lib::anglesConstPtr _msg)
		{
			this->update_joint_angles(_msg->joint1_z, _msg->joint1_x, _msg->joint2_x, _msg->joint3_x, _msg->joint4_x);
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

		void update_joint_angles(double z0, double x0, double x1, double x2, double x3)
		{

			std::string chasis_arm1_joint = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			std::string arm1_arm2_joint = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2_arm3_joint = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3_arm4_joint = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			// change to radian
			z0 = z0 * M_PI / 180.0;
			x0 = x0 * M_PI / 180.0;
			x1 = x1 * M_PI / 180.0;
			x2 = x2 * M_PI / 180.0;
			x3 = x3 * M_PI / 180.0;
			this->jointController->SetJointPosition(chasis_arm1_joint, x0, 0);

			this->jointController->SetJointPosition(chasis_arm1_joint, z0, 1);

			this->jointController->SetJointPosition(arm1_arm2_joint, x1, 0);

			this->jointController->SetJointPosition(arm2_arm3_joint, x2, 0);

			this->jointController->SetJointPosition(arm3_arm4_joint, x3, 0);
		}

		void publish_joint_angles()
		{

			// Get joint position by index.
			// 0 returns rotation accross X axis
			// 1 returns rotation accross Y axis
			// 2 returns rotation accross Z axis
			// If the Joint has only Z axis for rotation, 0 returns that value and 1 and 2 return nan
			ROS_INFO("INSIDE PUBLISHER");
			double joint1_z = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(0);

			double joint1_x = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(0);

			double joint2_x = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);

			double joint3_x = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			double joint4_x = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);

			// change to radian to degree
			joint1_z = joint1_z * 180.0 / M_PI;

			joint1_x = joint1_x * 180.0 / M_PI;

			joint2_x = joint2_x * 180.0 / M_PI;

			joint3_x = joint3_x * 180.0 / M_PI;

			joint4_x = joint4_x * 180.0 / M_PI;

			arm_lib::angles joint_angles;
			joint_angles.joint1_z = joint1_z;
			joint_angles.joint1_x = joint1_x;
			joint_angles.joint2_x = joint2_x;
			joint_angles.joint3_x = joint3_x;
			joint_angles.joint4_x = joint4_x;
			ros::Rate loop_rate(10);
			ROS_INFO("MSG: %f", joint_angles.joint4_x);
			angle_publisher_.publish(joint_angles);
			ros::spinOnce();
		}

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// 	// A joint controller object
		// 	// Takes PID value and apply angular velocity
		// 	//  or sets position of the angles
	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

		// // 	// PID object
	private:
		common::PID pid;

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
		ros::Publisher angle_publisher_;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}