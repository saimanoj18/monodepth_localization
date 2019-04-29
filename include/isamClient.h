#pragma once

#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <isam/isam.h>
#include <isam/slam3d.h>
#include <thread>
#include <mutex>
#include <queue>

#include "lib_irp_common/error.h"
#include "irp_slam_msgs/f_pose_partial.h"
#include "irp_slam_msgs/f_pose_pose.h"
#include "irp_slam_msgs/graph_vis.h"
#include "irp_slam_msgs/node.h"

#include "user_factors.h"
#include <Eigen/Dense>

#ifndef DTOR
#define DTOR M_PI/180.0 //  (M_PI / 180.) in units.h
#endif

#ifndef RTOD
#define RTOD 180.0/M_PI //  (180. / M_PI) in units.h
#endif

using namespace std;
using namespace isam;
using namespace Eigen;


struct RelativePose
{
  double dx;
  double dy;
  double dz;
  double droll;
  double dpitch;
  double dyaw;
};
struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static double
dcs (double chi2, double phi)
{
    double tmp = (2*phi) / (phi + chi2);
    return tmp > 1 ? 1 : tmp;
}

static double
pratik (double chi2)
{
    /* return 1; */
    double phi = 5.;
    return dcs (chi2, phi);
}

class isamClient
{
  public:
    isamClient (void);
    ~isamClient (void);

    void RosInit (ros::NodeHandle &n);
    void isamAddPosePartial(irp_slam_msgs::f_pose_partial f_prior);
    void isamAddPosePartialForRestore(irp_slam_msgs::f_pose_partial f_pose);
    void isamAddPosePose(irp_slam_msgs::f_pose_pose f_pose);
    void isamAddPosePoseForRestore(irp_slam_msgs::f_pose_pose f_pose);
    irp_slam_msgs::graph_vis GetGraphVis();
    void UpdateVisNode (irp_slam_msgs::graph_vis& gv, Node *node, int64_t id);
    int NodeNameToType(const char *name);
    vector<double> NodeVectorReorder(const Node *node);
    MatrixXd NodeMarginalReorder(const Node *node, MatrixXd m);
    Pose3d vec2isam_pose3d (const double z[6]);
    Pose3db vec2isam_pose3db (const double z[5]);
    MatrixXd vec2isam_sqrtinf3d (const double R[36]);
    MatrixXd vec2isam_sqrtinf3db (const double R[25]);
    void RunBatchOptimization();
    void RunUpdate();
    void PrintGraphState();
    void PrintGraph();
    void BatchProcess();

  private:

    Slam slam_;
    std::thread batch_thread_;
    bool batch_processing_flag_;
    std::mutex mutex_;
    std::queue<irp_slam_msgs::f_pose_pose> pose_pose_queue_;
    std::queue<irp_slam_msgs::f_pose_partial> pose_partial_queue_;

    int update_count_;
    int batch_interval_;
    ros::NodeHandle nh_;
    bool initialized_;

    Pose3d_Node *node_;
    Pose3d_Node *pre_node_;
    map<int64_t, Pose3d_Node*> nodes_;

    isam::Selector s_;

    double (*pratik_dcs_function) (double) = pratik;

};
