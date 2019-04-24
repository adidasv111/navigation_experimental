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
    controller_(nullptr),
    planner_(nullptr),
    replan_(true),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")//,
    // blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
  {
  }

  void SBPLRecovery::initialize (std::string n, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle p_nh = ros::NodeHandle("~/" + n);

    ROS_INFO("SBPLRecovery: Name is %s", n.c_str());

    std::string plan_topic, goal_topic;
    p_nh.param("plan_topic", plan_topic, std::string("NavfnROS/plan"));
    p_nh.param("goal_topic", goal_topic, std::string("/move_base/result"));
    p_nh.param("controller_frequency", control_frequency_, 10.0);
    p_nh.param("controller_patience", controller_patience_, 5.0);
    p_nh.param("planning_attempts", planning_attempts_, 3);
    p_nh.param("attempts_per_run", attempts_per_run_, 3);
    p_nh.param("use_local_frame", use_local_frame_, true);
    p_nh.param("use_pose_follower", use_pose_follower_, true);
    p_nh.param("use_sbpl_planner", use_sbpl_planner_, true);
    p_nh.param("use_recovery_costmap", use_recovery_costmap_, false);
    p_nh.param("abort_time", abort_time_, 30.0);

    if (use_recovery_costmap_)
    {
      p_nh.param("clear_recovery_map", clear_recovery_map_, false);
      if (clear_recovery_map_)
      {
        std::vector<std::string> clearable_layers_default = {"obstacle_layer", "inflation_layer"};
        p_nh.param("clear_recovery_costmap_layers", clearable_layers_recovery_costmap_, clearable_layers_default);
      }
    }

    double planner_frequency;
    p_nh.param("planner_frequency", planner_frequency, 1.0);
    if (planner_frequency <= 0)
      replan_ = false;
    else
    {
      replan_ = true;
      planner_period_ = 1.0/planner_frequency;
    }

  ROS_INFO_STREAM("plan_topic: --" << plan_topic << "--.");
  ROS_INFO_STREAM("goal_result_topic: --" << goal_topic << "--.");
  ROS_INFO_STREAM("controller_frequency_: --" << control_frequency_ << "--.");
  ROS_INFO_STREAM("controller_patience_: --" << controller_patience_ << "--.");
  ROS_INFO_STREAM("planning_attempts_: --" << planning_attempts_ << "--.");
  ROS_INFO_STREAM("attempts_per_run_: --" << attempts_per_run_ << "--.");
  ROS_INFO_STREAM("use_local_frame_: --" << use_local_frame_ << "--.");
  ROS_INFO_STREAM("use_pose_follower_: --" << use_pose_follower_ << "--.");
  ROS_INFO_STREAM("use_sbpl_planner: --" << use_sbpl_planner_ << "--.");
  ROS_INFO_STREAM("use_recovery_costmap_: --" << use_recovery_costmap_ << "--.");
  ROS_INFO_STREAM("planner_frequency: --" << planner_frequency << "--. period: --" << planner_period_ << "--.");
  ROS_INFO_STREAM("abort_time_: --" << abort_time_ << "--.");

    double planning_distance;
    p_nh.param("planning_distance", planning_distance, 2.0);
    sq_planning_distance_ = planning_distance * planning_distance;
  ROS_INFO_STREAM("planning_distance: --" << planning_distance << "--.");

    double abort_distance;
    p_nh.param("abort_distance", abort_distance, 10.0);
    sq_abort_distance_ = abort_distance * abort_distance;
  ROS_INFO_STREAM("abort_distance: --" << abort_distance << "--.");

    double prune_distance;
    p_nh.param("prune_distance", prune_distance, 0.4);
    sq_prune_distance_ = prune_distance * prune_distance;
  ROS_INFO_STREAM("prune_distance: --" << prune_distance << "--.");


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

    std::string planner_type = "";
    std::string planner_name = "";

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

      ROS_INFO_STREAM("sbpl_recovery: using planner " << planner_name << " - " << planner_type);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Failed to create the planner " << planner_name << ". Exception: " << ex.what());
      exit(1);
    }

  //   std::string controller_type = "";
  //   std::string controller_name = "";

  //   if(use_pose_follower_)
  //   {
  //     controller_type = "pose_follower/PoseFollower";
  //     controller_name = n + "/pose_follower";
  //   }
  //   else
  //   {
  //     controller_type = "base_local_planner/TrajectoryPlannerROS";
  //     controller_name = n + "/collision_planner";
  //   }

  //   //initialize the controller
  //   try {
  //     controller_ = blp_loader_.createInstance(controller_type);
  //     controller_->initialize(controller_name, tf, local_costmap_);
  //     ROS_INFO_STREAM("sbpl_recovery: using controller " << controller_name << " - " << controller_type);
  //  }
  //   catch (const pluginlib::PluginlibException& ex)
  //   {
  //     ROS_FATAL_STREAM("Failed to create the controller " << controller_name << ". Exception: " << ex.what());
  //     exit(1);
  //   }

    if (use_pose_follower_)
    {
      // local_planner_.initialize(n + "/pose_follower", tf, local_costmap_);
      controller_ = boost::make_shared<pose_follower::PoseFollower>();
      controller_->initialize(n + "/pose_follower", tf, local_costmap_);
      ROS_INFO_STREAM("sbpl_recovery: using controller pose_follower");
    }
    else
    {
      // collision_planner_.initialize(n + "/collision_planner", tf, local_costmap_);
      controller_ = boost::make_shared<base_local_planner::TrajectoryPlannerROS>();
      controller_->initialize(n + "/collision_planner", tf, local_costmap_);
      ROS_INFO_STREAM("sbpl_recovery: using controller TrajectoryPlannerROS");
    }

    //we'll need to subscribe to get the latest plan information
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::NodeHandle node_nh("~/");
    plan_sub_ = node_nh.subscribe<nav_msgs::Path>(plan_topic, 1, boost::bind(&SBPLRecovery::planCB, this, _1));
    goal_sub_ = node_nh.subscribe<move_base_msgs::MoveBaseActionResult>(goal_topic, 1, boost::bind(&SBPLRecovery::goalResultCB, this, _1));

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

  void SBPLRecovery::goalResultCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal_result)
  {
    uint status = goal_result->status.status;
    if (status == 2 || status == 4 || status == 8 || status == 9)
      received_abort_goal_.store(true);
  }

  double SBPLRecovery::sqDistance(const geometry_msgs::PoseStamped& p1,
      const geometry_msgs::PoseStamped& p2)
  {
    return (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x) +
      (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
  }

  bool SBPLRecovery::getGlobalPose(geometry_msgs::PoseStamped& global_pose)
  {
    tf::Stamped<tf::Pose> global_pose_tf;

    if(use_local_frame_)
    {
      if(!local_costmap_->getRobotPose(global_pose_tf))
      {
        ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
        return false;
      }
    }
    else
    {
      if(!global_costmap_->getRobotPose(global_pose_tf))
      {
        ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
        return false;
      }
    }

    tf::poseStampedTFToMsg(global_pose_tf, global_pose);

    return true;
  }


  std::vector<geometry_msgs::PoseStamped> SBPLRecovery::makePlan()
  {
    boost::mutex::scoped_lock l(plan_mutex_);
    std::vector<geometry_msgs::PoseStamped> sbpl_plan;

    geometry_msgs::PoseStamped start;
    // tf::Stamped<tf::Pose> global_pose;
    if (getGlobalPose(start) == false)
    {
      return sbpl_plan;
    }
    // if(use_local_frame_)
    // {
    //   if(!local_costmap_->getRobotPose(global_pose))
    //   {
    //     ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
    //     return sbpl_plan;
    //   }
    // }
    // else
    // {
    //   if(!global_costmap_->getRobotPose(global_pose))
    //   {
    //     ROS_ERROR("SBPL recovery behavior could not get the current pose of the robot. Doing nothing.");
    //     return sbpl_plan;
    //   }
    // }

    // geometry_msgs::PoseStamped start;
    // tf::poseStampedTFToMsg(global_pose, start);

    if(plan_.poses.size() <= 0)
    {
      ROS_ERROR("SBPL recovery got an empty reference path");
      return sbpl_plan;
    }

    //first, we want to walk far enough along the path that we get to a point
    //that is within the recovery distance from the robot. Otherwise, we might
    //move backwards along the plan
    // => go through the reference plan and find the first point withing planning distance from the robot,
    // so if the robot already moved along the reference plan, we would not plan to points that are already behind the robot
    unsigned int index = 0;
    for(index=0; index < plan_.poses.size(); ++index)
    {
      if(sqDistance(start, plan_.poses[index]) < sq_planning_distance_)
        break;
    }

    // if no starting point found, plan to the last reference point
    if (index >= plan_.poses.size())
      index = plan_.poses.size() - 1;

    if (index < 0)
    {
      ROS_ERROR("sbpl recovery: negative index!");
      return sbpl_plan;
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
            //plan_.poses[i].pose.position.x,
            //plan_.poses[i].pose.position.y);
            plan_.poses.back().pose.position.x,
            plan_.poses.back().pose.position.y);

        {
          // we need to lock the map when we plan
          boost::recursive_mutex* map_mutex;
          if (use_local_frame_)
            map_mutex = local_costmap_->getCostmap()->getMutex();
          else
            map_mutex = global_costmap_->getCostmap()->getMutex();

          boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(map_mutex));

//          if (planner_->makePlan(start, plan_.poses[i], sbpl_plan) && !sbpl_plan.empty())
          if (planner_->makePlan(start, plan_.poses.back(), sbpl_plan) && !sbpl_plan.empty())
          {
            prunePlan(start, sbpl_plan);
            ROS_INFO("Got a valid plan");
            return sbpl_plan;
          }
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
    received_abort_goal_.store(false);

    if (clear_recovery_map_)
      clearCostmapLayers(recovery_costmap_,  clearable_layers_recovery_costmap_);

    ros::Time start_time = ros::Time::now();

    for(int i=0; i < planning_attempts_; ++i)
    {
      geometry_msgs::PoseStamped start_pose;
      geometry_msgs::PoseStamped current_pose;
      if (getGlobalPose(start_pose) == false)
      {
        ROS_ERROR("SBPLRecovery::runBehavior failed to get start pose");
        return;
      }

      // plan first time
      std::vector<geometry_msgs::PoseStamped> sbpl_plan = makePlan();

      if(sbpl_plan.empty())
      {
        ROS_ERROR("Unable to find a valid pose to plan to on the global plan.");
        return;
      }
      ros::Time last_valid_plan = ros::Time::now();

      //ok... now we've got a plan so we need to try to follow it
      controller_->setPlan(sbpl_plan);

      ros::Rate r(control_frequency_);

      ros::Time last_valid_control = ros::Time::now();
      bool abort_distance_reached = false;

      while(ros::ok() &&
          last_valid_control + ros::Duration(controller_patience_) >= ros::Time::now() &&
          !controller_->isGoalReached() &&
          start_time + ros::Duration(abort_time_) >= ros::Time::now() &&
          abort_distance_reached == false &&
          received_abort_goal_.load() != true)
      {
        if (getGlobalPose(current_pose) == false)
        {
          ROS_ERROR("SBPLRecovery::runBehavior failed to get current pose");
          return;
        }
        if(sqDistance(start_pose, current_pose) > sq_abort_distance_)
          abort_distance_reached = true;

        if (replan_ && (last_valid_plan + ros::Duration(planner_period_) < ros::Time::now()))
        {
          sbpl_plan = makePlan();
          if (!sbpl_plan.empty())
          {
            controller_->setPlan(sbpl_plan);
            last_valid_plan = ros::Time::now();
          }
        }

        geometry_msgs::Twist cmd_vel;
        bool valid_control = controller_->computeVelocityCommands(cmd_vel);

        if(valid_control)
          last_valid_control = ros::Time::now();

        vel_pub_.publish(cmd_vel);
        r.sleep();
      }

      if(controller_->isGoalReached())
        ROS_INFO("The sbpl recovery behavior made it to its desired goal");
      else
        ROS_WARN("The sbpl recovery behavior failed to make it to its desired goal");
    }
}

  void SBPLRecovery::clearCostmapLayers(costmap_2d::Costmap2DROS* costmap, std::vector<std::string> layer_to_clear)
  {
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp)
    {
      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
      std::string name = plugin->getName();
      int slash = name.rfind('/');
      if( slash != std::string::npos ){
          name = name.substr(slash+1);
      }

      std::vector<std::string>::iterator it = find(layer_to_clear.begin(), layer_to_clear.end(), name);
      if (it != layer_to_clear.end())
      {
        ROS_INFO_STREAM("clearing layer --" << name << "--.");

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(costmap->getCostmap()->getMutex()));
        plugin->reset();
      }
    }

    // update map once
    costmap->updateMap();
  }

  void SBPLRecovery::prunePlan(const geometry_msgs::PoseStamped& pose, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // find the 1st point of the plan which is prune distance away
    // (plan should always start from robot pose)
    unsigned int index = 0;
    for(index = 0; index < plan.size(); ++index)
    {
      if(sqDistance(pose, plan[index]) > sq_prune_distance_)
        break;
    }

    if (index >= plan.size())
    {
      ROS_WARN("sbpl recovery: prune plan: didn't find any points outside prune distance. don't prune");
      return;
    }

    if (index <= 0)
    {
      ROS_WARN("sbpl recovery: prune plan: negative index! don't prune");
      return;
    }

    // erase all points until this point
    plan.erase(plan.begin(),plan.begin()+index);
  }
};
