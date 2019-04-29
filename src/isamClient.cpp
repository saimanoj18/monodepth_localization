#include <iostream>

#include "isamClient.h"

#define DEBUG 1

static double
_dcs_func (double chi2, double phi)
{
    double tmp = (2*phi) / (phi + chi2);
    return tmp > 1 ? 1 : tmp;
}

static double
_dcs_wrapper (double chi2)
{
    double phi = 5.;
    return _dcs_func (chi2, phi);
}

isamClient::isamClient(void)
{
    initialized_ = false;
    nodes_.clear();
    s_ = isam::ESTIMATE;
    update_count_ = 0;
    batch_interval_ = 30;
    batch_processing_flag_ = false;
}

isamClient::~isamClient(void)
{

  if(batch_thread_.joinable()) batch_thread_.join();

}

void isamClient::RosInit (ros::NodeHandle &n)
{
    nh_ = n;
}

void isamClient::isamAddPosePartial(irp_slam_msgs::f_pose_partial f_pose)
{
    mutex_.lock();
    if(batch_processing_flag_ == true){
      cout << "pose partial batch queue" << endl;
      pose_partial_queue_.push(f_pose);
      mutex_.unlock();
      return;
    }
    mutex_.unlock();

//    cout << "Add pose partial" << endl;
    if(initialized_ == false){      //Add initial node
        node_ = new Pose3d_Node ();
        slam_.add_node (node_);
        nodes_[f_pose.node_id] = node_;

        Pose3d priorZ     = vec2isam_pose3d (&(f_pose.z[0]));
        MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose.R[0]));
        Pose3d_Factor *prior = new Pose3d_Factor (node_, priorZ, SqrtInformation(sqrtinf));
        if(f_pose.use_dcs == 1){
            prior->set_scale_function(&pratik_dcs_function);
        }

        slam_.add_factor (prior);

        initialized_ = true;
    }else{  //Add pose factor

        if(nodes_.find(f_pose.node_id) == nodes_.end()){ //new node
            node_ = new Pose3d_Node ();
            nodes_[f_pose.node_id] = node_;
        }else{
            node_ = nodes_[f_pose.node_id];
        }

        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_XY_PARTIAL_P3D){

          VectorXd delta(2);
          delta << f_pose.z[0], f_pose.z[1];
          MatrixXd cov(2,2);
          for (int i=0;i<2; i++)
              for (int j=0;j<2; j++)
                  cov(i,j) = f_pose.R[i*2+j];
          MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
          Factor *posem = new Pose3d_xy_Factor(dynamic_cast<Pose3d_Node*>(node_), delta, SqrtInformation(sqrtinf));

          if(f_pose.use_dcs == 1){
              posem->set_scale_function(&pratik_dcs_function);
          }
          slam_.add_factor (posem);

        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_XYZ_PARTIAL_P3D){

          VectorXd delta(3);
          delta << f_pose.z[0], f_pose.z[1], f_pose.z[2];
          MatrixXd cov(3,3);
          for (int i=0;i<3; i++)
              for (int j=0;j<3; j++)
                  cov(i,j) = f_pose.R[i*3+j];
          MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
          Factor *posem = new Pose3d_xyz_Factor(dynamic_cast<Pose3d_Node*>(node_), delta, SqrtInformation(sqrtinf));

          if(f_pose.use_dcs == 1){
              posem->set_scale_function(&pratik_dcs_function);
          }
          slam_.add_factor (posem);

        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_FULL_STATE_P3D){

            Pose3d poseZ     = vec2isam_pose3d (&(f_pose.z[0]));
            MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose.R[0]));
            Pose3d_Factor *posem = new Pose3d_Factor (node_, poseZ, SqrtInformation(sqrtinf));
            if(f_pose.use_dcs == 1){
                posem->set_scale_function(&pratik_dcs_function);
            }
            slam_.add_factor (posem);
        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_H_PARTIAL_P3D){

            Matrix1d poseZ;
            poseZ(0,0) = f_pose.z[0];
            Matrix1d cov;
            cov(0,0) = f_pose.R[0];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

            Pose3d_h_Factor *posem = new Pose3d_h_Factor (node_, poseZ, SqrtInformation(sqrtinf));
            if(f_pose.use_dcs == 1){
                posem->set_scale_function(&pratik_dcs_function);
            }
            slam_.add_factor (posem);
        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_XYH_PARTIAL_P3D){
          VectorXd delta(3);
          delta << f_pose.z[0], f_pose.z[1], f_pose.z[2];
          MatrixXd cov(3,3);
          for (int i=0;i<3; i++)
              for (int j=0;j<3; j++)
                  cov(i,j) = f_pose.R[i*3+j];
          MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
          Factor *posem = new Pose3d_xyh_Factor(dynamic_cast<Pose3d_Node*>(node_), delta, SqrtInformation(sqrtinf));

          if(f_pose.use_dcs == 1){
              posem->set_scale_function(&pratik_dcs_function);
          }
          slam_.add_factor (posem);
        }


//        PrintGraph();
        RunUpdate();
    }
}

void isamClient::isamAddPosePartialForRestore(irp_slam_msgs::f_pose_partial f_pose)
{

    if(initialized_ == false){      //Add initial node
        node_ = new Pose3d_Node ();
        slam_.add_node (node_);
        nodes_[f_pose.node_id] = node_;

        Pose3d priorZ     = vec2isam_pose3d (&(f_pose.z[0]));
        MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose.R[0]));
        Pose3d_Factor *prior = new Pose3d_Factor (node_, priorZ, SqrtInformation(sqrtinf));
        if(f_pose.use_dcs == 1){
            prior->set_scale_function(&pratik_dcs_function);
        }

        slam_.add_factor (prior);

        initialized_ = true;
    }else{  //Add pose factor

        if(nodes_.find(f_pose.node_id) == nodes_.end()){ //new node
            node_ = new Pose3d_Node ();
            nodes_[f_pose.node_id] = node_;
        }else{
            node_ = nodes_[f_pose.node_id];
        }

        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_XY_PARTIAL_P3D){

          VectorXd delta(2);
          delta << f_pose.z[0], f_pose.z[1];
          MatrixXd cov(2,2);
          for (int i=0;i<2; i++)
              for (int j=0;j<2; j++)
                  cov(i,j) = f_pose.R[i*2+j];
          MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
          Factor *posem = new Pose3d_xy_Factor(dynamic_cast<Pose3d_Node*>(node_), delta, SqrtInformation(sqrtinf));

          if(f_pose.use_dcs == 1){
              posem->set_scale_function(&pratik_dcs_function);
          }
          slam_.add_factor (posem);

        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_XYZ_PARTIAL_P3D){

          VectorXd delta(3);
          delta << f_pose.z[0], f_pose.z[1], f_pose.z[2];
          MatrixXd cov(3,3);
          for (int i=0;i<3; i++)
              for (int j=0;j<3; j++)
                  cov(i,j) = f_pose.R[i*3+j];
          MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
          Factor *posem = new Pose3d_xyz_Factor(dynamic_cast<Pose3d_Node*>(node_), delta, SqrtInformation(sqrtinf));

          if(f_pose.use_dcs == 1){
              posem->set_scale_function(&pratik_dcs_function);
          }
          slam_.add_factor (posem);

        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_FULL_STATE_P3D){

            Pose3d poseZ     = vec2isam_pose3d (&(f_pose.z[0]));
            MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose.R[0]));
            Pose3d_Factor *posem = new Pose3d_Factor (node_, poseZ, SqrtInformation(sqrtinf));
            if(f_pose.use_dcs == 1){
                posem->set_scale_function(&pratik_dcs_function);
            }
            slam_.add_factor (posem);
        }
        if(f_pose.sub_type == irp_slam_msgs::f_pose_partial::SUB_TYPE_H_PARTIAL_P3D){

            Matrix1d poseZ;
            poseZ(0,0) = f_pose.z[0];
            Matrix1d cov;
            cov(0,0) = f_pose.R[0];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

            Pose3d_h_Factor *posem = new Pose3d_h_Factor (node_, poseZ, SqrtInformation(sqrtinf));
            if(f_pose.use_dcs == 1){
                posem->set_scale_function(&pratik_dcs_function);
            }
            slam_.add_factor (posem);
        }
    }
}

void isamClient::isamAddPosePose(irp_slam_msgs::f_pose_pose f_pose_pose)
{
    mutex_.lock();
    if(batch_processing_flag_ == true){
      cout << "pose pose batch queue" << endl;
      pose_pose_queue_.push(f_pose_pose);
      mutex_.unlock();
      return;
    }
    mutex_.unlock();

//    cout << "Add pose pose" << endl;
    if(initialized_ == false){
        cout << "Need to initialize node before adding pose pose factor" << endl;
        return;
    }

    pre_node_ = (nodes_.find(f_pose_pose.node_id1)==nodes_.end()) ? NULL : nodes_[f_pose_pose.node_id1];
    node_ = (nodes_.find(f_pose_pose.node_id2)==nodes_.end()) ? NULL : nodes_[f_pose_pose.node_id2];

    if(pre_node_ == NULL){
        cout << "Can't find previous node. Please check your prvious node" << endl;
        return;
    }

    if(node_ == NULL){
        node_ = new Pose3d_Node ();
        slam_.add_node (node_);
        nodes_[f_pose_pose.node_id2] = node_;
    }

    Pose3d delta     = vec2isam_pose3d (&(f_pose_pose.z[0]));
    MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose_pose.R[0]));
    Pose3d_Pose3d_Factor *odom = new Pose3d_Pose3d_Factor (pre_node_, node_, delta, SqrtInformation(sqrtinf));
    if(f_pose_pose.use_dcs == 1){
        odom->set_scale_function(&pratik_dcs_function);
    }

    slam_.add_factor (odom);
//        PrintGraph();
    update_count_++;
}

void isamClient::isamAddPosePoseForRestore(irp_slam_msgs::f_pose_pose f_pose_pose)
{
    if(initialized_ == false){
        cout << "Need to initialize node before adding pose pose factor" << endl;
        return;
    }

    pre_node_ = (nodes_.find(f_pose_pose.node_id1)==nodes_.end()) ? NULL : nodes_[f_pose_pose.node_id1];
    node_ = (nodes_.find(f_pose_pose.node_id2)==nodes_.end()) ? NULL : nodes_[f_pose_pose.node_id2];

    if(pre_node_ == NULL){
        cout << "Can't find previous node. Please check your prvious node" << endl;
        return;
    }

    if(node_ == NULL){
        node_ = new Pose3d_Node ();
        slam_.add_node (node_);
        nodes_[f_pose_pose.node_id2] = node_;
    }

    Pose3d delta     = vec2isam_pose3d (&(f_pose_pose.z[0]));
    MatrixXd sqrtinf = vec2isam_sqrtinf3d (&(f_pose_pose.R[0]));
    Pose3d_Pose3d_Factor *odom = new Pose3d_Pose3d_Factor (pre_node_, node_, delta, SqrtInformation(sqrtinf));
    if(f_pose_pose.use_dcs == 1){
        odom->set_scale_function(&pratik_dcs_function);
    }

    slam_.add_factor (odom);
    update_count_++;
}

int isamClient::NodeNameToType(const char *name)
{
    if (0 == strcmp(name, "Point2d")) {
        return irp_slam_msgs::node::TYPE_POINT2D;
    } else if (0 == strcmp(name, "Point3d")) {
        return irp_slam_msgs::node::TYPE_POINT3D;
    } else if (0 == strcmp(name, "Pose2d")) {
        return irp_slam_msgs::node::TYPE_POSE2D;
    } else if (0 == strcmp(name, "Pose3d")) {
        return irp_slam_msgs::node::TYPE_POSE3D;
    } else if (0 == strcmp(name, "Anchor2d")) {
        return irp_slam_msgs::node::TYPE_ANCHOR2D;
    } else if (0 == strcmp(name, "Anchor3d")) {
        return irp_slam_msgs::node::TYPE_ANCHOR3D;
    } else {
        ERROR("[isamu]\tUnknown node name: %s\n",name);
        return -1;
    }
}

vector<double> isamClient::NodeVectorReorder(const Node *node)
{
    int dim = node->dim();
    VectorXd x = node->vector(s_);
    vector<double> x_out;
    x_out.resize(dim);

    switch (NodeNameToType(node->name())) {
    case irp_slam_msgs::node::TYPE_ANCHOR3D:
    case irp_slam_msgs::node::TYPE_POSE3D: {
        x_out[0] = x(0);
        x_out[1] = x(1);
        x_out[2] = x(2);
        x_out[3] = x(5);
        x_out[4] = x(4);
        x_out[5] = x(3);
        break;
    }
    case irp_slam_msgs::node::TYPE_POINT2D:
    case irp_slam_msgs::node::TYPE_POINT3D:
    case irp_slam_msgs::node::TYPE_POSE2D:
    default: // no reordering nesscarry
        copy (x.data(), x.data()+dim, x_out.begin());
    }
    return x_out;
}

void isamClient::UpdateVisNode (irp_slam_msgs::graph_vis& gv, Node *node, int64_t id)
{
    gv.nn++;
    gv.node_id.push_back(id);
    gv.node_type.push_back(NodeNameToType(node->name()));
    gv.dim.push_back(node->dim());
    gv.n += node->dim();
    vector<double> tmp = NodeVectorReorder(node);
    gv.mu.insert(gv.mu.end(), tmp.begin(), tmp.end());
}

MatrixXd isamClient::NodeMarginalReorder(const Node *node, MatrixXd m)
{
    switch (NodeNameToType(node->name())) {
    case irp_slam_msgs::node::TYPE_POSE3D: {

        Eigen::MatrixXd out(m.rows(), m.cols());

        int row, col;
        for (int i=0; i<m.rows(); i++) {
            for (int j=0; j<m.cols(); j++) {
                row = i, col = j;
                if (i % 6 == 3) row = i+2;
                if (i % 6 == 5) row = i-2;
                if (j % 6 == 3) col = j+2;
                if (j % 6 == 5) col = j-2;
                out(i,j) = m(row,col);
            }
        }
        return out;
    }
    case irp_slam_msgs::node::TYPE_POINT2D:
    case irp_slam_msgs::node::TYPE_POINT3D:
    case irp_slam_msgs::node::TYPE_POSE2D:
    default: // no reordering nesscarry
        return m;
        break;
    }
}


irp_slam_msgs::graph_vis isamClient::GetGraphVis()
{
    irp_slam_msgs::graph_vis gv;
    map<int64_t, Pose3d_Node*>::iterator it;
    int32_t offset = 0;
    for (it = nodes_.begin(); it != nodes_.end(); it++) {
        UpdateVisNode (gv, it->second, it->first);
        gv.mu_ind.push_back (offset);
        offset += it->second->dim ();
    }

    if (gv.nn == 0) return gv;

    gv.utime = ros::Time::now().toNSec();

    return gv;

}

// conversion between rph to hpr
Pose3d isamClient::vec2isam_pose3d (const double z[6])
{
    return Pose3d (z[0], z[1], z[2], z[5], z[4], z[3]);
}

Pose3db isamClient::vec2isam_pose3db (const double z[5])
{
    return Pose3db (z[0], z[1], z[4], z[3], z[2]);
}

MatrixXd isamClient::vec2isam_sqrtinf3d (const double R[36])
{
    MatrixXd cov(6,6);
    int row, col;
    for (int i=0;i<6; i++) {
        for (int j=0;j<6; j++) {
            row = i; col = j;
            if (i==3) row = 5;
            if (i==5) row = 3;
            if (j==3) col = 5;
            if (j==5) col = 3;
            cov(i,j) = R[row*6+col];
        }
    }
    MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

    return sqrtinf;
}

MatrixXd isamClient::vec2isam_sqrtinf3db (const double R[25])
{
    MatrixXd cov(5,5);
    int row, col;
    for (int i=0;i<5; i++) {
        for (int j=0;j<5; j++) {
            row = i; col = j;
            if (i==2) row = 4;
            if (i==4) row = 2;
            if (j==2) col = 4;
            if (j==4) col = 2;
            cov(i,j) = R[row*5+col];
        }
    }
    MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

    return sqrtinf;
}

void isamClient::RunBatchOptimization()
{
    cout << "Run batch optimization" << endl;
    if(batch_thread_.joinable()) batch_thread_.join();
    batch_thread_ = std::thread(&isamClient::BatchProcess,this);
}

void isamClient::RunUpdate()
{
//    cout << "Run update optimization" << endl;
    if(batch_processing_flag_ == false) slam_.update();
}
void isamClient::PrintGraphState()
{
    if(batch_processing_flag_ == false) slam_.print_stats ();
}

void isamClient::PrintGraph()
{
    if(batch_processing_flag_ == false) slam_.write (cout);
}
void isamClient::BatchProcess()
{
  mutex_.lock();
  batch_processing_flag_ = true;
  mutex_.unlock();
  cout << "batch start" << endl;
  slam_.batch_optimization();
  cout << "batch finish" << endl;
  //queue process
  mutex_.lock();
  while(pose_pose_queue_.size() > 0){
    cout << "pose pose restore" << endl;
    auto data = pose_pose_queue_.front();
    pose_pose_queue_.pop();
    isamAddPosePoseForRestore(data);

  }
  while(pose_partial_queue_.size() > 0){
    cout << "pose partial restore" << endl;
    auto data = pose_partial_queue_.front();
    pose_partial_queue_.pop();
    isamAddPosePartialForRestore(data);
  }
  batch_processing_flag_ = false;
  mutex_.unlock();
  cout << "batch process all done" << endl;
}
