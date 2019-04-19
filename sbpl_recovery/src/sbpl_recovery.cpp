/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <sbpl_recovery/sbpl_recovery.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(sbpl_recovery::SBPLRecovery, nav_core::RecoveryBehavior)

namespace sbpl_recovery
{
  SBPLRecovery::SBPLRecovery():
    global_costmap_(NULL),
    local_costmap_(NULL),
    recovery_costmap_(NULL),
    tf_(NULL),
    initialized_(false),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
  {
  }

  void SBPLRecovery::initialize (std::string n, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle p_nh = ros::NodeHandle("~/" + n);

    ROS_INFO("SBPLRecovery: Name is %s", n.c_str());

    std::string plan_topic;
    p_nh.param("plan_topic", plan_topic, std::string("NavfnROS/plan"));
    p_nh.param("control_frequency", control_frequency_, 10.0);
    p_nh.param("controller_patience", controller_patience_, 5.0);
    p_nh.param("planning_attempts", planning_attempts_, 3);
    p_nh.param("attempts_per_run", attempts_per_run_, 3);
    p_nh.param("use_local_frame", use_local_frame_, true);
    p_nh.param("use_pose_follower", use_pose_follower_, true);
    p_nh.param("use_sbpl_planner", use_sbpl_planner_, true);
    p_nh.param("use_recovery_costmap", use_recovery_costmap_, false);

  ROS_INFO_STREAM("plan_topic: --" << plan_topic << "--.");
  ROS_INFO_STREAM("control_frequency_: --" << control_frequency_ << "--.");
  ROS_INFO_STREAM("controller_patience_: --" << controller_patience_ << "--.");
  ROS_INFO_STREAM("planning_attempts_: --" << planning_attempts_ << "--.");
  ROS_INFO_STREAM("attempts_per_run_: --" << attempts_per_run_ << "--.");
  ROS_INFO_STREAM("use_local_frame_: --" << use_local_frame_ << "--.");
  ROS_INFO_STREAM("use_pose_follower_: --" << use_pose_follower_ << "--.");
  ROS_INFO_STREAM("use_sbpl_planner: --" << use_sbpl_planner_ << "--.");
  ROS_INFO_STREAM("use_recovery_costmap_: --" << use_recovery_costmap_ << "--.");

    double planning_distance;
    p_nh.param("planning_distance", planning_distance, 2.0);
    sq_planning_distance_ = planning_distance * planning_distance;
  ROS_INFO_STREAM("planning_distance: --" << planning_distance << "--.");


    //we need to initialize our costmaps
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    tf_ = tf;

    // initialize the extended global costmap
    recovery_costmap_ = new costmap_2d::Costmap2DROS("recovery_costmap", *tf_);
    recovery_costmap_->pause();

    if (use_recovery_costmap_)
    {
      global_costmap_ = recovery_costmap_;
      global_costmap_->start();
    }

    std::string planner_type;
    std::string planner_name;

    if(use_sbpl_planner_)
    {
      planner_type = "sbpl_lattice_planner::SBPLLatticePlanner";
      planner_name = n + "/sbpl_lattice_planner";
    }
    else
    {
      planner_type = "nav_core_adapter::GlobalPlannerAdapter";
      planner_name = n + "/GlobalPlannerAdapter";
    }

    // //we need to initialize our local and global planners
    // if(use_local_frame_)
    //   global_planner_.initialize(n + "/sbpl_lattice_planner", local_costmap_);
    // else
    //   global_planner_.initialize(n + "/sbpl_lattice_planner", global_costmap_);

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(planner_type);

      if(use_local_frame_)
        planner_->initialize(planner_name, local_costmap_);
      else
        planner_->initialize(planner_name, global_costmap_);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Failed to create the planner " << planner_name << ". Exception: " << ex.what());
      exit(1);
    }

    if (use_pose_follower_)
    {
      ROS_INFO_STREAM("----sbpl_recover: create pose_follower:");
      local_planner_.initialize(n + "/pose_follower", tf, local_costmap_);
    }
    else
    {
      ROS_INFO_STREAM("----sbpl_recover: create collision planner:");
      collision_planner_.initialize(n + "/collision_planner", tf, local_costmap_);
    }

    //we'll need to subscribe to get the latest plan information
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::NodeHandle node_nh("~/");
    plan_sub_ = node_nh.subscribe<nav_msgs::Path>(plan_topic, 1, boost::bind(&SBPLRecovery::planCB, this, _1));
    initialized_ = true;
  }

  void SBPLRecovery::planCB(const nav_msgs::Path::ConstPtr& plan)
  {
    //just copy the plan data over

    tf::Stamped<tf::Pose> global_pose;
    local_costmap_->getRobotPose(global_pose);

    costmap_2d::Costmap2D costmap;
    costmap = *(local_costmap_->getCostmap());

    if(use_local_frame_)
    {
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      if(base_local_planner::transformGlobalPlan(*tf_, plan->poses, global_pose, costmap, local_costmap_->getGlobalFrameID(), transformed_plan))
      {
        boost::mutex::scoped_lock l(plan_mutex_);
        if(!transformed_plan.empty())
          plan_.header = transformed_plan[0].header;
        plan_.poses = transformed_plan;
      }
      else
        ROS_WARN("Could not transform to frame of the local recovery");
    }
    else
    {
      boost::mutex::scoped_lock l(plan_mutex_);
      plan_ = *plan;
    }
  }

  double SBPLRecovery::sqDistance(const geometry_msgs::PoseStamped& p1,
      const geometry_msgs::PoseStamped& p2)
  {
    return (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x) +
      (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
  }

  std::vector<geometry_msgs::PoseStamped> SBPLRecovery::makePlan()
  {
    boost::mutex::scoped_lock l(plan_mutex_);
    std::vector<geometry_msgs::PoseStamped> sbpl_plan;

    tf::Stamped<tf::Pose> global_pose;
    if(use_local_frame_)
    {
      if(!local_costmap_->getRobotPose(global_pose))
      {
        ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
        return sbpl_plan;
      }
    }
    else
    {
      if(!global_costmap_->getRobotPose(global_pose))
      {
        ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
        return sbpl_plan;
      }
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //first, we want to walk far enough along the path that we get to a point
    //that is within the recovery distance from the robot. Otherwise, we might
    //move backwards along the plan
    unsigned int index = 0;
    for(index=0; index < plan_.poses.size(); ++index)
    {
      if(sqDistance(start, plan_.poses[index]) < sq_planning_distance_)
        break;
    }

    //next, we want to find a goal point that is far enough away from the robot on the
    //original plan and attempt to plan to it
    int unsuccessful_attempts = 0;
    for(unsigned int i = index; i < plan_.poses.size(); ++i)
    {
      ROS_DEBUG("SQ Distance: %.2f,  spd: %.2f, start (%.2f, %.2f), goal (%.2f, %.2f)",
          sqDistance(start, plan_.poses[i]),
          sq_planning_distance_,
          start.pose.position.x, start.pose.position.y,
          plan_.poses[i].pose.position.x,
          plan_.poses[i].pose.position.y);
      if(sqDistance(start, plan_.poses[i]) >= sq_planning_distance_ || i == (plan_.poses.size() - 1))
      {
        ROS_INFO("Calling sbpl planner with start (%.2f, %.2f), goal (%.2f, %.2f)",
            start.pose.position.x, start.pose.position.y,
            // plan_.poses[i].pose.position.x,
            // plan_.poses[i].pose.position.y);
            plan_.poses.back().pose.position.x,
            plan_.poses.back().pose.position.y);

        // if(global_planner_.makePlan(start, plan_.poses[i], sbpl_plan) && !sbpl_plan.empty())
        if(planner_->makePlan(start, plan_.poses.back(), sbpl_plan) && !sbpl_plan.empty())
        {
          ROS_INFO("Got a valid plan");
          return sbpl_plan;
        }
        sbpl_plan.clear();

        //make sure that we don't spend forever planning
        unsuccessful_attempts++;
        if(unsuccessful_attempts >= attempts_per_run_)
          return sbpl_plan;
      }
    }

    return sbpl_plan;
  }

  void SBPLRecovery::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("Please initialize this recovery behavior before attempting to run it.");
      return;
    }

    ROS_INFO("Starting the sbpl recovery behavior");

    for(int i=0; i <= planning_attempts_; ++i)
    {
      std::vector<geometry_msgs::PoseStamped> sbpl_plan = makePlan();

      if(sbpl_plan.empty())
      {
        ROS_ERROR("Unable to find a valid pose to plan to on the global plan.");
        return;
      }

      //ok... now we've got a plan so we need to try to follow it
      if (use_pose_follower_)
      {
        local_planner_.setPlan(sbpl_plan);

        ros::Rate r(control_frequency_);
        ros::Time last_valid_control = ros::Time::now();
        while (ros::ok() && last_valid_control + ros::Duration(controller_patience_) >= ros::Time::now() && !local_planner_.isGoalReached())
        {
          geometry_msgs::Twist cmd_vel;
          bool valid_control = local_planner_.computeVelocityCommands(cmd_vel);

          if (valid_control)
            last_valid_control = ros::Time::now();

          ROS_INFO_STREAM("sbpl recovery pose follower: pub cmd: " << cmd_vel.linear.x << " , "<< cmd_vel.linear.y << " , "<< cmd_vel.angular.z << " , ");

          vel_pub_.publish(cmd_vel);
          r.sleep();
        }

        if (local_planner_.isGoalReached())
          ROS_INFO("The sbpl recovery behavior made it to its desired goal");
        else
          ROS_WARN("The sbpl recovery behavior failed to make it to its desired goal");
      }
      else
      {
        collision_planner_.setPlan(sbpl_plan);

        ros::Rate r(control_frequency_);
        ros::Time last_valid_control = ros::Time::now();
        while (ros::ok() && last_valid_control + ros::Duration(controller_patience_) >= ros::Time::now() && !collision_planner_.isGoalReached())
        {
          geometry_msgs::Twist cmd_vel;
          bool valid_control = collision_planner_.computeVelocityCommands(cmd_vel);

          if (valid_control)
            last_valid_control = ros::Time::now();

          vel_pub_.publish(cmd_vel);
          r.sleep();
        }

        if(collision_planner_.isGoalReached())
          ROS_INFO("The sbpl recovery behavior made it to its desired goal");
        else
          ROS_WARN("The sbpl recovery behavior failed to make it to its desired goal");
      }
    }
  }
};
