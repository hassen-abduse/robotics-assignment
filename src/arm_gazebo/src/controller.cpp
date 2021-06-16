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
#include "arm_gazebo/pose.h"
#include "arm_gazebo/IK.h"
#include "arm_gazebo/FK.h"

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
			this->posePublisher = node.advertise<arm_gazebo::pose>("arm/current_pose", 1000);
			//this->posePublisher = node.advertise<arm_lib::angles>("arm/")

			std::string chasisArm1Joint = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			std::string arm1Arm2Joint = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2Arm3Joint = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3Arm4Joint = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();
			std::string arm4Arm5Joint = this->model->GetJoint("arm4_arm5_joint")->GetScopedName();
			std::string arm5PalmJoint = this->model->GetJoint("arm5_palm_joint")->GetScopedName();

			common::PID joint1PID = common::PID(30.0, 10.0, 7.0);
			common::PID joint2PID = common::PID(25.0, 8.0, 6.0);
			common::PID joint3PID = common::PID(20.0, 7.0, 5.0);
			common::PID joint4PID = common::PID(17.0, 6.0, 4.0);
			common::PID joint5PID = common::PID(15.0, 5.0, 3.0);
			common::PID joint6PID = common::PID(10.0, 4.0, 2.0);

			this->SetPID(chasisArm1Joint, joint1PID);
			this->SetPID(arm1Arm2Joint, joint2PID);
			this->SetPID(arm2Arm3Joint, joint3PID);
			this->SetPID(arm3Arm4Joint, joint4PID);
			this->SetPID(arm4Arm5Joint, joint5PID);
			this->SetPID(arm5PalmJoint, joint6PID);

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
				ros::SubscribeOptions::create<arm_gazebo::pose>(
					"/" + this->model->GetName() + "/pose_cmd",
					1,
					boost::bind(&ModelPush::OnRosMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);

			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&ModelPush::QueueThread, this));

			this->publisherThread = std::thread(std::bind(&ModelPush::PublishPose, this));

			std::vector<double> _pose = {0.0, 20.0, 30.0};
			this->CatchBox(_pose);
			//std::vector<double> rel = {20.0, 10.0, 2.0};
			//this->ReleaseBox(rel);
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
		void OnRosMsg(const arm_gazebo::poseConstPtr _pose)
		{
			if (_pose->option == 1)
			{
				CatchBox({_pose->x, _pose->y, _pose->z});
			}
			else if(_pose->option == 0)
			{
				ReleaseBox({_pose->x, _pose->y, _pose->z});
			}
		}

	private:
		void UpdateJointAngles(std::vector<double> _pose)
		{
			std::vector<double> joint_angles = this->InverseKinematics(_pose);

			std::string chasisArm1Joint = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			std::string arm1Arm2Joint = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2Arm3Joint = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3Arm4Joint = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();
			std::string arm4Arm5Joint = this->model->GetJoint("arm4_arm5_joint")->GetScopedName();
			std::string arm5PalmJoint = this->model->GetJoint("arm5_palm_joint")->GetScopedName();

			common::PID joint1PID = common::PID(30.0, 10.0, 7.0);
			common::PID joint2PID = common::PID(25.0, 8.0, 6.0);
			common::PID joint3PID = common::PID(20.0, 7.0, 5.0);
			common::PID joint4PID = common::PID(17.0, 6.0, 4.0);
			common::PID joint5PID = common::PID(15.0, 5.0, 3.0);
			common::PID joint6PID = common::PID(10.0, 4.0, 2.0);

			this->SetPID(chasisArm1Joint, joint1PID);
			this->SetAngle(chasisArm1Joint, joint_angles[0]);

			this->SetPID(arm1Arm2Joint, joint2PID);
			this->SetAngle(arm1Arm2Joint, joint_angles[1]);

			this->SetPID(arm2Arm3Joint, joint3PID);
			this->SetAngle(arm2Arm3Joint, joint_angles[2]);

			this->SetPID(arm3Arm4Joint, joint4PID);
			this->SetAngle(arm3Arm4Joint, joint_angles[3]);

			this->SetPID(arm4Arm5Joint, joint5PID);
			this->SetAngle(arm4Arm5Joint, joint_angles[4]);

			this->SetPID(arm5PalmJoint, joint6PID);
			this->SetAngle(arm5PalmJoint, joint_angles[5]);

			this->jointController->Update();
		}

	private:
		double GetAngle(std::string jointName)
		{
			return physics::JointState(this->model->GetJoint(jointName)).Position();
		}

	private:
		void SetAngle(std::string jointName, double _angle)
		{
			this->jointController->SetPositionTarget(jointName, _angle);
		}

	private:
		void SetPID(std::string jointName, common::PID _pid)
		{
			this->jointController->SetPositionPID(jointName, _pid);
		}

	private:
		void PublishPose()
		{
			while (ros::ok())
			{
				double joint1 = this->GetAngle("chasis_arm1_joint");
				double joint2 = this->GetAngle("arm1_arm2_joint");
				double joint3 = this->GetAngle("arm2_arm3_joint");
				double joint4 = this->GetAngle("arm3_arm4_joint");
				double joint5 = this->GetAngle("arm4_arm5_joint");
				double joint6 = this->GetAngle("arm5_palm_joint");

				// change to radian to degree
				joint1 = joint1 * 180.0 / M_PI;
				joint2 = joint2 * 180.0 / M_PI;
				joint3 = joint3 * 180.0 / M_PI;
				joint4 = joint4 * 180.0 / M_PI;
				joint5 = joint5 * 180.0 / M_PI;
				joint6 = joint6 * 180.0 / M_PI;

				std::vector<double> joint_angles = {joint1, joint2, joint3, joint4, joint5, joint6};
				std::vector<double> link_lengths = {0.1, 0.05, 2.0, 1.0, 0.5, 0.5, 0.5};
				ros::NodeHandle n;
				ros::ServiceClient fk_client = n.serviceClient<arm_gazebo::FK>("FK");
				arm_gazebo::FK fk_srv;

				fk_srv.request.joint_angles = joint_angles;
				fk_srv.request.link_lengths = link_lengths;

				if (fk_client.call(fk_srv))
				{
					std::vector<double> end_effector_pose = fk_srv.response.actuator_pose;
					arm_gazebo::pose _pose;
					_pose.x = end_effector_pose[0];
					_pose.y = end_effector_pose[1];
					_pose.y = end_effector_pose[2];
					ros::Rate loop_rate(10);
					posePublisher.publish(_pose);
					ros::spinOnce();
				}
				else
				{
					ROS_ERROR("Failed to call FK service.");
				}
			}
		}


	private:
		std::vector<double> InverseKinematics(std::vector<double> _pose)
		{
			ros::NodeHandle n;
			ros::ServiceClient ik_client = n.serviceClient<arm_gazebo::IK>("IK");
			arm_gazebo::IK ik_srv;

			ik_srv.request.desired_pose = {_pose[0], _pose[1], _pose[2]};
			std::vector<double> angles;
			if (ik_client.call(ik_srv))
			{
				angles = ik_srv.response.joint_angles;
			}
			else
			{
				ROS_ERROR("Failed to call service transform_vector");
			}
			return angles;
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
		ros::Publisher posePublisher;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}