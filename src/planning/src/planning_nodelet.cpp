#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <fstream>
#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>
#include <iostream>
#include "matplotlibcpp.h" 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Geometry> 


namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
namespace plt = matplotlibcpp;
////MOJ KOD
class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Publisher pose_array_pub;

  ros::Timer plan_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  ///MOJ KOD 
  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::PoseStamped drone_pose;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;

/////MOJ KOD
  void saveVectorsToCSV(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z, const std::string& filename) {
    if (x.size() != y.size() || y.size() != z.size()) {
        std::cerr << "Error: Vectors x, y, and z must have the same size." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << "X,Y,Z\n"; // Write the CSV header
        for (size_t i = 0; i < x.size(); ++i) {
            file << x[i] << "," << y[i] << "," << z[i] << "\n";
        }
        file.close();
        std::cout << "Vectors saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
  }


  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  void triger_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (msg->poses.size() < 2) {
        ROS_WARN("Received less than 2 poses in /triger topic!");
        return;
    }

    triger_received_ = true;

    geometry_msgs::Pose goal, drone;
    goal = msg->poses[0];  
    drone = msg->poses[1];  

    goal_pose.header = msg->header;
    drone_pose.header = msg->header;

    goal_pose.pose.position.x = goal.position.x;
    goal_pose.pose.position.y = goal.position.y;
    goal_pose.pose.position.z = goal.position.z;
    goal_pose.pose.orientation.x = goal.orientation.x;
    goal_pose.pose.orientation.y = goal.orientation.y;
    goal_pose.pose.orientation.z = goal.orientation.z;
    goal_pose.pose.orientation.w = goal.orientation.w;

    drone_pose.pose.position.x = drone.position.x;
    drone_pose.pose.position.y = drone.position.y;
    drone_pose.pose.position.z = drone.position.z;
    drone_pose.pose.orientation.x = drone.orientation.x;
    drone_pose.pose.orientation.y = drone.orientation.y;
    drone_pose.pose.orientation.z = drone.orientation.z;
    drone_pose.pose.orientation.w = drone.orientation.w;

    ROS_INFO("Received Goal Pose: [%.2f, %.2f, %.2f]", 
             goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    ROS_INFO("Received Init Pose: [%.2f, %.2f, %.2f]", 
             drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z);
}

  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!triger_received_) {
      return;
    }
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    bool generate_new_traj_success = false;
    Trajectory traj;
    Eigen::Vector3d target_p, target_v;
    Eigen::Quaterniond target_q;
    Eigen::Quaterniond land_q(1, 0, 0, 0);

    iniState.setZero();
    iniState.col(0).x() = drone_pose.pose.position.x;
    iniState.col(0).y() = drone_pose.pose.position.y;
    iniState.col(0).z() = drone_pose.pose.position.z;
    iniState.col(1) = perching_v_;
    //target_p = perching_p_;
    target_p << goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            goal_pose.pose.position.z;
    target_v = perching_v_; //OSTAVIT?
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0;

    //Eigen::Vector3d axis = perching_axis_.normalized(); MOJ KOD !!!!
    Eigen::Quaterniond quaternion(
        goal_pose.pose.orientation.w,
        goal_pose.pose.orientation.x,
        goal_pose.pose.orientation.y,
        goal_pose.pose.orientation.z
    );

    quaternion.normalize();
    Eigen::AngleAxisd angle_axis(quaternion);

    Eigen::Vector3d axis = angle_axis.axis(); // Get the axis of rotation

    std::cout << "Axis of rotation: [" 
              << axis.x() << ", " << axis.y() << ", " << axis.z() << "]" << std::endl;
    
    double theta = perching_theta_ * 0.5;
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;

    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;

    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(traj, "traj");

      Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
      Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
      visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");
    }
    if (!generate_new_traj_success) {
      triger_received_ = false;
      return;
      // assert(false);
    }

    // NOTE run vis
    // hopf fiberation
    auto v2q = [](const Eigen::Vector3d& v, Eigen::Quaterniond& q) -> bool {
      double a = v.x();
      double b = v.y();
      double c = v.z();
      if (c == -1) {
        return false;
      }
      double d = 1.0 / sqrt(2.0 * (1 + c));
      q.w() = (1 + c) * d;
      q.x() = -b * d;
      q.y() = a * d;
      q.z() = 0;
      return true;
    };

    auto f_DN = [](const Eigen::Vector3d& x) {
      double x_norm_2 = x.squaredNorm();
      return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    };
    // auto f_D2N = [](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    //   double x_norm_2 = x.squaredNorm();
    //   double x_norm_3 = x_norm_2 * x.norm();
    //   Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    //   return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
    // };

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    //MOJ KOD
    double totalDur = traj.getTotalDuration(); 
    std::cout << "TOTAL DUUR " << totalDur << "  " << dt << std::endl;
  
    //MOJ NOVI KOD!!!!!!!!!!!

    Eigen::Matrix3Xd positions = traj.getPositions();
    Eigen::Vector3d velocitiez = traj.getVel(totalDur-dt);
    std::cout << "Velocity: " << velocitiez.transpose() << std::endl;

    // Print the dimensions of the positions matrix
    std::cout << "Matrix dimensions: " << positions.rows() << "x" << positions.cols() << std::endl;


    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "world"; // Set frame of reference
    pose_array_msg.header.stamp = ros::Time::now();

    // Extract x, y, z components
    std::vector<double> x, y, z;
    for (int i = 0; i < positions.cols(); ++i) { // Iterate over columns
        x.push_back(positions(0, i)); // X component
        y.push_back(positions(1, i)); // Y component
        z.push_back(positions(2, i)); // Z component

        geometry_msgs::Pose pose;
        pose.position.x = positions(0, i);
        pose.position.y = positions(1, i);
        pose.position.z = positions(2, i);
        // Set orientation (identity quaternion for no rotation)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        pose_array_msg.poses.push_back(pose);
    }

    saveVectorsToCSV(x,y,z, "/home/luka/catkin_ws/src/Fast-Perching/trajectory_positions.csv");
    std::cout << "Publishing PoseArray with poses   " << pose_array_msg << std::endl;
    pose_array_pub.publish(pose_array_msg);


    for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
      ros::Duration(dt).sleep();
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      // std::cout << p.x() << " , " << p.z() << " , ";

      Eigen::Vector3d zb = thrust.normalized();
      {
        // double a = zb.x();
        // double b = zb.y();
        // double c = zb.z();
        Eigen::Vector3d zb_dot = f_DN(thrust) * j;
        double omega12 = zb_dot.norm();
        // if (omega12 > 3.1) {
        //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
        // }
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
        // double a_dot = zb_dot.x();
        // double b_dot = zb_dot.y();
        // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
        // std::cout << "jer: " << j.transpose() << std::endl;
        // std::cout << "omega12: " << zb_dot.norm() << std::endl;
        // std::cout << "omega3: " << omega3 << std::endl;
        // std::cout << thrust.x() << " , " << thrust.z() << " , ";
        // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
        // std::cout << omega2 << std::endl;
        // std::cout << zb_dot.norm() << std::endl;
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q);
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt;
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1);
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) {
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom");
      }
      if (std::abs(t - 2.9709) < 1e-6) { // Adjust tolerance as needed
          std::cout << "The last point" << std::endl;
          // std::cout << msg.pose.pose.position.x << " " 
          //           << msg.pose.pose.position.y << " " 
          //           << msg.pose.pose.position.z << std::endl;
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      target_p = target_p + target_v * dt;
      
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = ros::Time::now();


      //MOJ KOD

                  
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omegaa: " << max_omega << std::endl;
      //////////////////////////////////
      //MOJ KOD
        int pieceNum = traj.getPieceNum();
        std::cout << "Number of pieces in the trajectory: " << pieceNum << std::endl;

        // Retrieve the positions from the trajectory
        Eigen::Matrix3d positions = traj.getPositions();

        // Extract x, y, z components
        // std::vector<double> x, y, z;
        // for (int i = 0; i < positions.rows(); ++i) {
        //     x.push_back(positions(i, 0));
        //     y.push_back(positions(i, 1));
        //     z.push_back(positions(i, 2));
        // }

        //saveVectorsToCSV(positions, "trajectory_positions.csv");

        // Plot the trajectory in 3D
        // plt::figure();
        // plt::plot3(x, y, z, { {"label", "Trajectory"} });
        // plt::xlabel("X");
        // plt::ylabel("Y");
        // plt::set_zlabel("Z");
        // plt::title("3D Trajectory Plot");
        // plt::legend();
        // plt::show();  

      //////////////////////////////////

      }
      
    }
    std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    std::cout << "max omegga: " << max_omega << std::endl;

    int pieceNum = traj.getPieceNum();
    std::cout << "Number of pieces in the trajectory: " << pieceNum << std::endl;

    

    //Plot the trajectory in 3D
    // plt::figure();
    // plt::plot3(x, y, z, { {"label", "Trajectory"} });
    // plt::xlabel("X");
    // plt::ylabel("Y");
    // plt::set_zlabel("Z");
    // plt::title("3D Trajectory Plot");
    // plt::legend();
    // plt::show();  
  

    triger_received_ = false;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    nh.getParam("replan", debug_replan_);

    // NOTE once
    nh.getParam("perching_px", perching_p_.x());
    nh.getParam("perching_py", perching_p_.y());
    nh.getParam("perching_pz", perching_p_.z());
    nh.getParam("perching_vx", perching_v_.x());
    nh.getParam("perching_vy", perching_v_.y());
    nh.getParam("perching_vz", perching_v_.z());
    nh.getParam("perching_axis_x", perching_axis_.x());
    nh.getParam("perching_axis_y", perching_axis_.y());
    nh.getParam("perching_axis_z", perching_axis_.z());
    nh.getParam("perching_theta", perching_theta_);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);

    triger_sub_ = nh.subscribe<geometry_msgs::PoseArray>(
    "triger/array", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());

    pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/fastp_trajectory", 10);
    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);