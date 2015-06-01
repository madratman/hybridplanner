//TODO 
  	// add dOxygen comments. 
	// include folder.
	// include headers.
	// default constructoes.

class HybridPlanner
{
	// TODO separate out planning and appending. 
	// TODO convert into trajectory_msgs 

	// typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
 	// typedef TrajectoryVec::const_iterator TrajectoryIter;
	public:
		~HybridPlanner()  //virtual? baseclass?
		{
		 // moveit_group_.reset();
		 // kinematic_state.reset();
		}

		void appendFreeSpaceJointTrajectoryPoint(std::map<std::string, double> joints)
		{
			/**
		   * @brief 
		   * @param 
		   * @param 
		   */
		   group.setJointValueTarget(joints);
		}


		void appendFreeSpaceJointTrajectorySegment()
		{
			
		}

		void appendFreeSpaceJointTrajectoryPoint(Eigen::Translation3d position, Eigen::Quaterniond orientation)
		{
			Eigen::Affine3d pose = position*orientation;
			group.setPoseTarget(pose);
			//TODO 
		}

		void appendFreeSpaceCartesianTrajectorySegment(geometry_msgs::Pose start, geometry_msgs::Pose end)
		{
					
			// Eigen::Affine3d pose = Eigen::Translation3d(-0.358, 0.043, 0.865)*Eigen::Quaterniond(0.690, 0.108, 0.706, -0.112);
	 		// group.setPoseTarget(pose);

			// geometry_msgs::Pose target_pose1;
			// target_pose1.orientation.w = 1.0;
			// target_pose1.position.x = 0.28;
			// target_pose1.position.y = -0.7;
			// target_pose1.position.z = 1.0;
			// group.setPoseTarget(target_pose1);

			geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
			std::vector<geometry_msgs::Pose> waypoints;

			waypoints.push_back(start);
			waypoints.push_back(end);

		    moveit_msgs::RobotTrajectory trajectory;
		    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

		    // TODO 
			// defination of moveit_msgs/RobotTrajectory.msg:-
		 		// trajectory_msgs/JointTrajectory joint_trajectory
				// trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory

		    hybridTrajectory.points.push_back(trajectory.joint_trajectory) //TODO check if this is right

		 //    moveit_msgs::ExecuteKnownTrajectory srv;
			// srv.request.wait_for_execution = true;
			// ros::ServiceClient executeKnownTrajectoryServiceClient = node_handle.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
			// group.computeCartesianPath(waypoints, 0.005, 1000.0, srv.request.trajectory);
			// executeKnownTrajectoryServiceClient.call(srv);
			// executeKnownTrajectoryServiceClient.call(srv);

		}


		//TODO this is a reference for the next function
		// void populateTrajectoryMsg(const std::vector<descartes_core::TrajectoryPtPtr>& solution,
  //                            const descartes_core::RobotModel& robot_model,
  //                            double interpoint_delay,
  //                            double time_offset,
  //                            trajectory_msgs::JointTrajectory& trajectory)
		// {
		// 	//TOCHECK are joint_names required? 	
		//     typedef std::vector<descartes_core::TrajectoryPtPtr>::const_iterator JointSolutionIterator;

		//     // For calculating the time_from_start field of the trajectoryPoint
		//     ros::Duration time_from_start(time_offset);
		//     std::vector<double> dummy_seed (robot_model.getDOF(), 0.0);

		//     for (JointSolutionIterator it = solution.begin(); it != solution.end(); ++it)
		//     {
		//       // Retrieve actual target joint angles from the polymorphic interface function
		//       std::vector<double> sol;
		//       it->get()->getNominalJointPose(dummy_seed, robot_model, sol);
		      
		//       trajectory_msgs::JointTrajectoryPoint point;
		//       point.positions = sol;
		//       point.velocities.resize(sol.size(), 0.0); // Fill extra fields with zeroes for now
		//       point.accelerations.resize(sol.size(), 0.0);
		//       point.effort.resize(sol.size(), 0.0);
		//       point.time_from_start = time_from_start;

		//       // add trajectory point to array
		//       trajectory.points.push_back(point);

		//       // increment time so far by next duration
		//       time_from_start += ros::Duration(interpoint_delay);
		//     }
		//     return;
		// }

		void appendProcessPath(std::vector<descartes_core::TrajectoryPtPtr> descartesTrajectory)
		{
			// TODO have to think about the following. This is from ros-i indigo training class
			hybridTrajectory.header.stamp = ros::Time::now();
			hybridTrajectory.header.frame_id = config_.world_frame;
			hybridTrajectory.joint_names = config_.joint_names;
			// For keeping track of time-so-far in the trajectory
			double time_offset = 0.0;
			for (unsigned int i = 0; i < descartesTrajectory.size(); i++)
			  {
			    // Find nominal joint solution at this point
			    std::vector<double> joints;

			    // getting joint position at current point
			    const descartes_core::TrajectoryPtPtr& joint_point = descartesTrajectory[i];
			    //TODO check next line. use dummy seed as 1st parameter?
			    // std::vector<double> dummy_seed (robot_model.getDOF(), 0.0);
			    joint_point->getNominalJointPose(std::vector<double>(), *descartes_robot_model_ptr_, joints);
			    //TODO what exactly is a "nominal" pose. THere was discussion in a recent issur as well. 
			    //what are seed states? Isn't the first parameter wrong?


			    // Fill out a ROS trajectory point
			    trajectory_msgs::JointTrajectoryPoint pt;
			    pt.positions = joints;
			    // velocity, acceleration, and effort are given dummy values
			    // we'll let the controller figure them out
			    pt.velocities.resize(joints.size(), 0.0);
			    pt.accelerations.resize(joints.size(), 0.0);
			    pt.effort.resize(joints.size(), 0.0);
			    // TODO effort is?
			    // set the time into the trajectory
			    pt.time_from_start = ros::Duration(time_offset);
			    // increment time
			    time_offset += config_.time_delay;

			    hybridTrajectory.points.push_back(pt);
			  }
                                                      
		}

		void appendProcessPath(descartes_core::TrajectoryPtPtr* processPathWaypoints)
		{

		}

		void appendProcessPath()
		{

		}

		descartes_core::TrajectoryPtPtr makeCartesianPoint(const std::vector<Eigen::Affine3d&> pose)
		{	

			using namespace descartes_core;
  			using namespace descartes_trajectory;

  			return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );

			typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
			typedef TrajectoryVec::const_iterator TrajectoryIter;


			descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
   			points.push_back(pt);

		}

		descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
		{
			using namespace descartes_core;
			using namespace descartes_trajectory;
			return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
		}


		trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model, 
			const std::vector<std::string>& joint_names, double time_delay)
		{

		}

		bool executeHybridTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
		{
		  // Create a Follow Joint Trajectory Action Client
		  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
		  if (!ac.waitForServer(ros::Duration(2.0)))
		  {
		    ROS_ERROR("Could not connect to action server");
		    return false;
		  }

		  control_msgs::FollowJointTrajectoryGoal goal;
		  goal.trajectory = trajectory;
		  goal.goal_time_tolerance = ros::Duration(1.0);
		  
		  ac.sendGoal(goal);

		  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
		  {
		    ROS_INFO("Action server reported successful execution");
		    return true;
		  } 
		  else {
		    ROS_WARN("Action server could not execute trajectory");
		    return false;
		  }
		}

		//from https://github.com/ros-industrial/industrial_training/blob/hydro-devel/training/orig/demo_manipulation/src/collision_avoidance_pick_and_place/src/utilities/pick_and_place_utilities.cpp
		// TODO think of use cases and make functions for such cases to append traj segments
		// std::vector<geometry_msgs::Pose> create_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
		// {
		//   geometry_msgs::Pose start_pose, target_pose, end_pose;
		//   std::vector<geometry_msgs::Pose> poses;

		//   // creating start pose by applying a translation along +z by approach distance
		//   tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);

		//   // converting target pose
		//   tf::poseTFToMsg(target_tf,target_pose);

		//   // creating end pose by applying a translation along +z by retreat distance
		//   tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat_dis))*target_tf,end_pose);

		//   poses.clear();
		//   poses.push_back(start_pose);
		//   poses.push_back(target_pose);
		//   poses.push_back(end_pose);

		//   return poses;
		// }

		void planHybridPath()
		{

		}

		void debugFreeSpacePlan()
		{
			// move_group_interface::MoveGroup::Plan plan;
		    // if (!group.plan(plan))
		    // {
		      // ROS_FATAL("Unable to create motion plan.  Aborting.");
		      // exit(-1);
		    // }
		    

		    // do non-blocking move request
		  	// group.asyncExecute(plan);

		  	// // cancel motion after fixed time
		  	// sleep(1.0);
		  	// group.stop();
	  		// sleep(1.0);  // wait for stop command to be received

		}

	protected:
		HybridPlannerConfiguration config_ //make struct?
		ros::NodeHandle nh_;     
		// ros::Publisher marker_publisher_;        
  		ros::ServiceClient moveit_run_path_client_;
		descartes_core::RobotModelPtr descartes_corerobot_model_ptr_; 
		descartes_planner::SparsePlanner sparsePlanner_;     
		descartes_planner::DensePlanner densePlanner_;
		trajectory_msgs::JointTrajectoryPoint hybridTrajectory;
}


// ToThinkOver

/*
In ROS, the task of converting Cartesian goal positions into a stream of joint positions is traditionally described as the "planning" step. This task typically involves not only IK, but also collision detection and joint-space search algorithms to identify a valid trajectory between the "current" joint state and the specified goal state. Additionally, this planning step may include additional constraints (e.g. maintain desired end-effector orientation) and various trajectory-smoothing filters.

As a first step, you'll need to describe how much of this planning capability you want/need to utilize. I will outline 3 potential approaches below. I recommend #2, but have never used this method. In the past, I have used #1, but that approach requires you to manually provide some of the functionality that MoveIt does automatically in method #2.

1) Direct IK Solution
In this approach, you will run IK to calculate a joint position for each cartesian point. These joint points will be combined into a single trajectory and sent to the robot. The trajectory may require additional code support to be robust: velocity/accel smoothing, arm-configuration continuity, etc.

create a JointTrajectoryActionGoal message to hold the joint trajectory
create a MoveIt JointStateGroup to use for calculating IK (as shown here)
for each cartesian point:
call joint_state_group->setFromIK() to calc IK
NOTE: this can take an Eigen::Affine3d input
NOTE: may need to call joint_state_group->setVariableValues() to set initial joint state
call joint_state_group->getVariableValues() to get computed joint position
create a JointTrajectoryPoint from your joint position
NOTE: you'll need to compute the appropriate velocity/accel values yourself
add JointTrajectoryPoint to FollowJointTrajectoryActionGoal.goal.trajectory.points*
send the completed joint trajectory (FollowJointTrajectoryActionGoal) to the appropriate action. For ROS-I nodes, this is typically "joint_trajectory_action".
When you start the motoman ROS-I nodes, it typically brings up both the interface to the actual robot and a joint_trajectory_action node that manages sending the FollowJointTrajectoryActionGoal to the robot interface nodes.
check to make sure that action node is running: rosnode info joint_trajectory_action
2) MoveIt ComputeCartesianPath
In this approach, you pass the full sequence of cartesian points to MoveIt. MoveIt computes the full joint trajectory between points, while also providing additional functionality beyond the basic approach described above: It inserts additional points to maintain linear motion, automatically checks for collisions and arm-configuration "flips", and generates the velocities and accelerations needed for smooth motion.

create a MoveIt MoveGroupInterface object, as shown here
call move_group->ComputeCartesianPath() with your sequence of cartesian points
NOTE: this function accepts a std::vector<Pose>, so you'll need to convert your cartesian positions using tf::poseEigenToMsg()
this returns a RobotTrajectory.
send your trajectory to MoveIt
the easiest way I've found is to use the move_group node's ExecuteKnownTrajectory service, which is available on my setup at "/execute_kinematic_path".
alternatively, you could probably send it to the TrajectoryExecutionManager object directly, as shown here, but that seems messy
3) MoveIt Point-to-Point
This approach is probably the "easiest", but is unlikely to give you the motion you are looking for. You pass individual cartesian points to the MoveIt planning/execution pipeline, and MoveIt will handle everything from there. However, the robot will stop at each cartesian waypoint, and the motion between waypoints is not guaranteed to be linear.

Create a move_group_interface object, as above
for each cartesian point:
call group.setPoseTarget() with your cartesian point
call group.move() to plan and execute motion
*/



//TODO pass by reference!