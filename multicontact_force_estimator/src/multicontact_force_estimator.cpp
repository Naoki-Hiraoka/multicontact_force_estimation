#include <algorithm>

#include <Eigen/Sparse>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/JointPath>
#include <cnoid/SceneGraph>
#include <cnoid/src/Body/InverseDynamics.h>

#include <prioritized_qp/PrioritizedQPSolver.h>

#include <multicontact_force_estimator_msgs/MultiContactForceEstimatorConfig.h>
#include <multicontact_force_estimator_msgs/WrenchStampedArray.h>
#include <multicontact_force_estimator_msgs/ContactPointArray.h>
#include <multicontact_force_estimator/ForceSensorJointModel.h>
#include <multicontact_force_estimator/Jacobian.h>


namespace multicontact_force_estimator {
  class multicontact_force_estimator {
  public:
    multicontact_force_estimator() {
      ros::NodeHandle nh;
      ros::NodeHandle pnh("~");

      // ロボットモデルをロード
      {
        std::string filename;
        if (!pnh.getParam("robotmodel", filename)) {
          ROS_FATAL_STREAM("Failed to get robotmodel param");
          exit(1);
        }
        // package://に対応
        std::string packagestr = "package://";
        if(filename.size()>packagestr.size() && filename.substr(0,packagestr.size()) == packagestr){
          filename = filename.substr(packagestr.size());
          int pos = filename.find("/");
          filename = ros::package::getPath(filename.substr(0,pos)) + filename.substr(pos);
        }
        // ファイルをロード
        cnoid::BodyLoader loader;
        cnoid::BodyPtr vrml_robot = loader.load(filename);
        if(!vrml_robot){
          ROS_FATAL_STREAM("Failed to load " << filename.c_str());
          exit(1);
        }
        // 力センサを関節に変換
        robot_ = forcesensor_joint_model::convert2forcesensor_joint_model(vrml_robot);
      }

      // setup subscribers
      jointStateSub_ = nh.subscribe<sensor_msgs::JointState>("joint_states",
                                                             100, // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可
                                                             [&](const sensor_msgs::JointState::ConstPtr& msg){
                                                               isTopicUpdated_ = true;
                                                               jointStateMsg_ = msg;
                                                             });
      imuSub_ = nh.subscribe<sensor_msgs::Imu>("imu",
                                               1,
                                               [&](const sensor_msgs::Imu::ConstPtr& msg){
                                                 isTopicUpdated_ = true;
                                                 imuMsg_ = msg;
                                               });
      {
        cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
        forceSensorMsg_.resize(forceSensors.size());
        for(size_t i=0;i<forceSensors.size();i++){
          forceSensorSub_.push_back(nh.subscribe<geometry_msgs::WrenchStamped>(forceSensors[i]->name(),
                                                                               1,
                                                                               [&,i] // iのみコピーで渡す
                                                                               (const geometry_msgs::WrenchStamped::ConstPtr& msg){
                                                                                 isTopicUpdated_ = true;
                                                                                 forceSensorMsg_[i] = msg;
                                                                               }));
        }
      }
      contactPointSub_ = pnh.subscribe<multicontact_force_estimator_msgs::ContactPointArray>
        ("contact_points",
         1,
         [&](const multicontact_force_estimator_msgs::ContactPointArray::ConstPtr& msg){
          isTopicUpdated_ = true;
          contactPointMsg_ = msg;
         });

      // setup dymamic reconfigure
      {
        config_.debug_print = false;
        pnh.param("offset_update_rate", config_.offset_update_rate, 1.0); // 1 hz
        pnh.param("force_offset_update_thre", config_.force_offset_update_thre, 0.5); //0.5[N]
        pnh.param("moment_offset_update_thre", config_.moment_offset_update_thre, 0.1); //0.1 [Nm]
        pnh.param("marker_scale", config_.marker_scale, 0.01); // 0.01[m/N]
        cfgServer_.setConfigDefault(config_);
        cfgServer_.updateConfig(config_);
        cfgServer_.setCallback([&](multicontact_force_estimator_msgs::MultiContactForceEstimatorConfig& config, int32_t level){config_ = config; });//setCallbackの中でcallbackが呼ばれるので、その前にupdateConfigを呼ぶ必要がある
      }

      // setup publisher
      {
        cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
        for(size_t i=0;i<forceSensors.size();i++){
          offsetForcePub_[forceSensors[i]->name()] = pnh.advertise<geometry_msgs::WrenchStamped>(forceSensors[i]->name()+"_offset", 1000);
        }
      }
      rootMassOffsetPub_ = pnh.advertise<std_msgs::Float64>("root_mass_offset", 1000);
      estimatedForcePub_ = pnh.advertise<multicontact_force_estimator_msgs::WrenchStampedArray>("estimated_force", 1000);
      estimatedForceMarkerPub_ = pnh.advertise<visualization_msgs::MarkerArray>("estimated_force_marker", 1000);

      // setup variables
      originalRootMass_ = robot_->rootLink()->m();
      {
        cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
        for(size_t i=0;i<forceSensors.size();i++){
          forceSensorOffsets_[forceSensors[i]->name()] = cnoid::Vector6::Zero();
        }
      }
      {
        qpForceSensorTask_ = std::make_shared<prioritized_qp::Task>();
        qpForceSensorTask_->name() = "Force Sensor Task";
        qpForceSensorTask_->solver().settings()->setMaxIteraction(4000);
        qpForceSensorTask_->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpForceSensorTask_->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpForceSensorTask_->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        qpForceSensorTask_->toSolve() = true;
      }
      {
        qpRootTask_ = std::make_shared<prioritized_qp::Task>();
        qpRootTask_->name() = "Root Force Task";
        qpRootTask_->solver().settings()->setMaxIteraction(4000);
        qpRootTask_->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpRootTask_->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpRootTask_->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        qpRootTask_->toSolve() = true;
      }
      {
        qpJointTask_ = std::make_shared<prioritized_qp::Task>();
        qpJointTask_->name() = "Joint Torque Task";
        qpJointTask_->solver().settings()->setMaxIteraction(4000);
        qpJointTask_->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpJointTask_->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        qpJointTask_->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        qpJointTask_->toSolve() = true;
      }

      // main loop
      {
        int rate;
        pnh.param("rate", rate, 50); // 50 hz
        periodicTimer_ = pnh.createTimer(ros::Duration(1.0 / rate), &multicontact_force_estimator::periodicTimerCallback, this);
      }

    }

    void periodicTimerCallback(const ros::TimerEvent& event) {
      if (!isTopicUpdated_) return;
      isTopicUpdated_ = false;
      seq_++;

      const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());

      // センサ値を反映
      if (jointStateMsg_) {
        for(size_t i=0;i<jointStateMsg_->name.size();i++){
          cnoid::Link* joint = robot_->link(jointStateMsg_->name[i]);
          if(!joint) {
            ROS_WARN_STREAM("joint " << jointStateMsg_->name[i] << " is not found");
            continue;
          }
          if(jointStateMsg_->position.size() == jointStateMsg_->name.size()) joint->q() = jointStateMsg_->position[i];
          if(jointStateMsg_->velocity.size() == jointStateMsg_->name.size()) joint->dq() = jointStateMsg_->velocity[i];
          if(jointStateMsg_->effort.size() == jointStateMsg_->name.size()) joint->u() = jointStateMsg_->effort[i];
        }
      }
      if (imuMsg_) {
        // rootのvel, accを反映させていない TODO
        cnoid::Device* device = robot_->findDevice(imuMsg_->header.frame_id);
        if(!device) {
          ROS_WARN_STREAM("imu " << imuMsg_->header.frame_id << " is not found");
        } else {
          robot_->calcForwardKinematics(false,false);//static
          cnoid::Matrix3 currentdeviceR = device->link()->R() * device->T_local().linear();
          Eigen::Quaterniond q;
          tf::quaternionMsgToEigen(imuMsg_->orientation,q);
          cnoid::Matrix3 realdeviceR = q.normalized().toRotationMatrix();
          cnoid::Matrix3 realRootR = realdeviceR * currentdeviceR.inverse() * robot_->rootLink()->R();
          robot_->rootLink()->R() = realRootR;
        }
      }
      for (size_t i=0;i<forceSensorMsg_.size();i++){
        if(forceSensorMsg_[i]){
          if(!robot_->findDevice<cnoid::ForceSensor>(forceSensorMsg_[i]->header.frame_id)) {
            ROS_WARN_STREAM("force sensor " << forceSensorMsg_[i]->header.frame_id << " not found");
            continue;
          }
          cnoid::Vector6 F;
          tf::wrenchMsgToEigen(forceSensorMsg_[i]->wrench,F);
          F -= forceSensorOffsets_[forceSensorMsg_[i]->header.frame_id];
          // 符号が逆であることに注意(センサはロボットが受ける力. 関節トルクはロボットが発揮する力)
          robot_->link(forceSensorMsg_[i]->header.frame_id+"X")->u() = - F(0);
          robot_->link(forceSensorMsg_[i]->header.frame_id+"Y")->u() = - F(1);
          robot_->link(forceSensorMsg_[i]->header.frame_id+"Z")->u() = - F(2);
          robot_->link(forceSensorMsg_[i]->header.frame_id+"Roll")->u() = - F(3);
          robot_->link(forceSensorMsg_[i]->header.frame_id+"Pitch")->u() = - F(4);
          robot_->link(forceSensorMsg_[i]->header.frame_id+"Yaw")->u() = - F(5);
        }
      }
      robot_->calcForwardKinematics(false,false);//static

      // estimate
      std::vector<cnoid::Vector6> estimatedForce;
      if (contactPointMsg_){
        estimatedForce.resize(contactPointMsg_->contactpoints.size(),cnoid::Vector6::Zero());

        unsigned int totalEstimatedForceDim=0;
        for (size_t i=0;i<contactPointMsg_->contactpoints.size();i++){
          switch(contactPointMsg_->contactpoints[i].type){
          case multicontact_force_estimator_msgs::ContactPoint::SIXAXIS :
            totalEstimatedForceDim += 6;
            break;
          case multicontact_force_estimator_msgs::ContactPoint::POINT:
            totalEstimatedForceDim += 3;
            break;
          default:
            ROS_WARN_STREAM("undefined contact type " << contactPointMsg_->contactpoints[i].type);
            break;
          }
        }

        /*
          g = sensors + J^f
          f: estimated force
        */
        cnoid::VectorX sensors = cnoid::VectorX::Zero(6+robot_->numJoints());
        for(size_t i=0;i<robot_->numJoints();i++){
          sensors(6+i) = robot_->joint(i)->u();
        }
        cnoid::VectorX g = cnoid::VectorX::Zero(6+robot_->numJoints());
        {
          cnoid::Vector3 gravity(0, 0, 9.80665);
          robot_->rootLink()->dv() = gravity;//static
          robot_->rootLink()->dw().setZero();//static
          cnoid::Vector6 f = cnoid::calcInverseDynamics(robot_->rootLink()); //原点周り
          f.tail<3>() -= robot_->rootLink()->p().cross(f.head<3>()); // world系, rootlink周り
          g.head<6>() = f;
          for(size_t i=0;i<robot_->numJoints();i++){
            g(6+i) = robot_->joint(i)->u();
          }
        }
        Eigen::SparseMatrix<double,Eigen::RowMajor> J(totalEstimatedForceDim,6+robot_->numJoints());//contactpoint系、contactpoint周り. root6dofはworld系, rootlink周り
        {
          unsigned int idx = 0;
          for(size_t i=0;i<contactPointMsg_->contactpoints.size();i++){
            cnoid::LinkPtr parentLink;
            // chorenoidのリンクの名前はURDFの関節の名前になっている. URDFのリンクの名前はchorenoidの各リンクのジオメトリ名になっている
            for(size_t j=0;j<robot_->links().size();j++){
              cnoid::Affine3 tmp;
              if(robot_->links()[j]->visualShape()){
                cnoid::SgNodePath visualpath = robot_->links()[j]->visualShape()->findNode(contactPointMsg_->contactpoints[i].pose.header.frame_id,tmp);
                if(visualpath.size()!=0){
                  parentLink = robot_->links()[j];
                }
              }
            }
            Eigen::SparseMatrix<double,Eigen::RowMajor> J_this(6,6+robot_->numJoints());//contactpoint系、contactpoint周り. root6dofはworld系, rootlink周り
            if(parentLink){
              cnoid::JointPathPtr path(new cnoid::JointPath(parentLink));
              Eigen::Affine3d T_local;
              tf::poseMsgToEigen(contactPointMsg_->contactpoints[i].pose.pose,T_local);
              J_this = jacobian::calcJacobian(path,T_local,true);
            }
            switch(contactPointMsg_->contactpoints[i].type){
            case multicontact_force_estimator_msgs::ContactPoint::SIXAXIS :
              J.middleRows(idx,6) = J_this;
              idx += 6;
              break;
            case multicontact_force_estimator_msgs::ContactPoint::POINT:
              J.middleRows(idx,3) = J_this.topRows(3);
              idx += 3;
              break;
            default:
              break;
            }
          }
        }
        Eigen::SparseMatrix<double,Eigen::RowMajor> Jt = J.transpose();

        /*
          solve QP
         */
        cnoid::VectorX result = cnoid::VectorX::Zero(totalEstimatedForceDim);
        // 接触力制約は与えないほうが良い. 現実の力が制約を侵していた場合、制約の範囲内で運動方程式を無理に満たそうとして、制約の範囲内で大きく変動するような挙動を示す
        {
          // 力センサの値
          qpForceSensorTask_->solver().settings()->setVerbosity(config_.debug_print);

          qpForceSensorTask_->A().resize(6*forceSensors.size(),totalEstimatedForceDim);
          qpForceSensorTask_->b().resize(6*forceSensors.size());
          qpForceSensorTask_->wa().resize(6*forceSensors.size());
          qpForceSensorTask_->C().resize(0,totalEstimatedForceDim);
          qpForceSensorTask_->dl().resize(0);
          qpForceSensorTask_->du().resize(0);
          qpForceSensorTask_->wc().resize(0);
          qpForceSensorTask_->w().resize(totalEstimatedForceDim);

          for(size_t i=0;i<forceSensors.size();i++){
            const std::string& name = forceSensors[i]->name();
            int id = 6+robot_->link(name+"X")->jointId();
            qpForceSensorTask_->A().middleRows(i*6,6) = Jt.middleRows(id,6);
            qpForceSensorTask_->b().segment<6>(i*6) = g.segment<6>(id) - sensors.segment<6>(id);
            for(size_t j=0;j<3;j++) qpForceSensorTask_->wa()(i*6+j) = 1;
            for(size_t j=0;j<3;j++) qpForceSensorTask_->wa()(i*6+3+j) = 1;
          }

          for(size_t i=0;i<totalEstimatedForceDim;i++) qpForceSensorTask_->w()(i) = 1e-6;
        }
        {
          // rootのつりあい
          qpRootTask_->solver().settings()->setVerbosity(config_.debug_print);

          qpRootTask_->A().resize(6,totalEstimatedForceDim);
          qpRootTask_->b().resize(6);
          qpRootTask_->wa().resize(6);
          qpRootTask_->C().resize(0,totalEstimatedForceDim);
          qpRootTask_->dl().resize(0);
          qpRootTask_->du().resize(0);
          qpRootTask_->wc().resize(0);
          qpRootTask_->w().resize(totalEstimatedForceDim);

          qpRootTask_->A() = Jt.topRows(6);
          qpRootTask_->b() = g.head<6>() - sensors.head<6>();
          for(size_t j=0;j<3;j++) qpRootTask_->wa()(j) = 1;
          for(size_t j=0;j<3;j++) qpRootTask_->wa()(3+j) = 1;

          for(size_t i=0;i<totalEstimatedForceDim;i++) qpRootTask_->w()(i) = 1e-6;
        }
        {
          // 関節トルクの値
          qpJointTask_->solver().settings()->setVerbosity(config_.debug_print);

          qpJointTask_->A().resize(robot_->numJoints()-6*forceSensors.size(),totalEstimatedForceDim);
          qpJointTask_->b().resize(robot_->numJoints()-6*forceSensors.size());
          qpJointTask_->wa().resize(robot_->numJoints()-6*forceSensors.size());
          qpJointTask_->C().resize(0,totalEstimatedForceDim);
          qpJointTask_->dl().resize(0);
          qpJointTask_->du().resize(0);
          qpJointTask_->wc().resize(0);
          qpJointTask_->w().resize(totalEstimatedForceDim);

          for(size_t i=0;i<robot_->numJoints()-6*forceSensors.size();i++){
            int id = 6+i;
            qpJointTask_->A().innerVector(i) = Jt.innerVector(id);
            qpJointTask_->b()(i) = g(id) - sensors(id);
            qpJointTask_->wa()(i) = 1;
          }

          for(size_t i=0;i<totalEstimatedForceDim;i++) qpJointTask_->w()(i) = 1e-6;
        }
        {
          // solve
          std::vector<std::shared_ptr<prioritized_qp::Task> > tasks{qpForceSensorTask_,qpRootTask_, qpJointTask_};
          bool solved = prioritized_qp::solve(tasks,result,config_.debug_print);
          if(!solved){
            ROS_ERROR_STREAM("QP failed");
            return;
          }
        }

        /*
          結果の反映
         */
        {
          estimatedForce.resize(contactPointMsg_->contactpoints.size(),cnoid::Vector6::Zero());
          size_t idx = 0;
          for (size_t i=0;i<contactPointMsg_->contactpoints.size();i++){
            switch(contactPointMsg_->contactpoints[i].type){
            case multicontact_force_estimator_msgs::ContactPoint::SIXAXIS :
              estimatedForce[i] = result.segment<6>(idx);
              idx += 6;
              break;
            case multicontact_force_estimator_msgs::ContactPoint::POINT:
              estimatedForce[i].head<3>() = result.segment<3>(idx);
              idx += 3;
              break;
            default:
              break;
            }
          }
        }
        if(config_.offset_update_rate > 0) {
          cnoid::VectorX forceOffsets = qpForceSensorTask_->A() * result - qpForceSensorTask_->b();
          for(size_t i=0;i<forceSensors.size();i++){
            // 符号が逆であることに注意(センサはロボットが受ける力. 関節トルクはロボットが発揮する力)
            cnoid::Vector6 offset = - forceOffsets.segment<6>(i*6);
            if(offset.head<3>().norm() > config_.force_offset_update_thre) {
              forceSensorOffsets_[forceSensors[i]->name()].head<3>() += 1.0/config_.offset_update_rate*(event.current_real-event.last_real).toSec() * offset.head<3>();
            }
            if(offset.tail<3>().norm() > config_.moment_offset_update_thre) {
              forceSensorOffsets_[forceSensors[i]->name()].tail<3>() += 1.0/config_.offset_update_rate*(event.current_real-event.last_real).toSec() * offset.tail<3>();
            }
            }
        }
        if(config_.offset_update_rate > 0) {
          cnoid::Vector6 rootForceOffset = qpRootTask_->A() * result - qpRootTask_->b();
          if(std::abs(rootForceOffset[2]) > config_.force_offset_update_thre) {
            robot_->rootLink()->setMass(robot_->rootLink()->m() + 1.0/config_.offset_update_rate*(event.current_real-event.last_real).toSec() * rootForceOffset[2]);
          }
        }
      }

      // publish
      if (contactPointMsg_){
        multicontact_force_estimator_msgs::WrenchStampedArray msg;
        for(size_t i=0; i<contactPointMsg_->contactpoints.size();i++){
          geometry_msgs::WrenchStamped wrenchStamped;
          wrenchStamped.header.seq = seq_;
          wrenchStamped.header.stamp = event.current_real;
          wrenchStamped.header.frame_id = contactPointMsg_->contactpoints[i].header.frame_id;
          tf::wrenchEigenToMsg(estimatedForce[i], wrenchStamped.wrench);
          msg.wrenches.push_back(wrenchStamped);
        }
        estimatedForcePub_.publish(msg);
      }
      if (contactPointMsg_){
        if(estimatedForceMarkerMsg_.markers.size() < contactPointMsg_->contactpoints.size()) estimatedForceMarkerMsg_.markers.resize(contactPointMsg_->contactpoints.size());
        for(size_t i=0; i<contactPointMsg_->contactpoints.size();i++){
          estimatedForceMarkerMsg_.markers[i].header.seq = seq_;
          estimatedForceMarkerMsg_.markers[i].header.stamp = event.current_real;
          estimatedForceMarkerMsg_.markers[i].header.frame_id = contactPointMsg_->contactpoints[i].pose.header.frame_id;
          estimatedForceMarkerMsg_.markers[i].id = i;
          estimatedForceMarkerMsg_.markers[i].type = visualization_msgs::Marker::ARROW;
          estimatedForceMarkerMsg_.markers[i].action = visualization_msgs::Marker::ADD;
          estimatedForceMarkerMsg_.markers[i].pose.orientation.w = 1;
          estimatedForceMarkerMsg_.markers[i].scale.x = 0.01;
          estimatedForceMarkerMsg_.markers[i].scale.y = 0.02;
          estimatedForceMarkerMsg_.markers[i].color.r = 1.0;
          estimatedForceMarkerMsg_.markers[i].color.a = 1.0;
          estimatedForceMarkerMsg_.markers[i].points.resize(2);
          estimatedForceMarkerMsg_.markers[i].points[0] = contactPointMsg_->contactpoints[i].pose.pose.position;
          estimatedForceMarkerMsg_.markers[i].points[1] = contactPointMsg_->contactpoints[i].pose.pose.position;
          Eigen::Quaterniond localR;
          tf::quaternionMsgToEigen(contactPointMsg_->contactpoints[i].pose.pose.orientation,localR);
          cnoid::Vector3 localf = localR * estimatedForce[i].head<3>();
          estimatedForceMarkerMsg_.markers[i].points[1].x += localf[0] * config_.marker_scale;
          estimatedForceMarkerMsg_.markers[i].points[1].y += localf[1] * config_.marker_scale;
          estimatedForceMarkerMsg_.markers[i].points[1].z += localf[2] * config_.marker_scale;
        }
        for(size_t i=contactPointMsg_->contactpoints.size();i<estimatedForceMarkerMsg_.markers.size(); i++){
          estimatedForceMarkerMsg_.markers[i].id = i;
          estimatedForceMarkerMsg_.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        estimatedForceMarkerPub_.publish(estimatedForceMarkerMsg_);
      }
      {
        for(size_t i=0;i<forceSensors.size();i++){
          geometry_msgs::WrenchStamped msg;
          msg.header.seq = seq_;
          msg.header.stamp = event.current_real;
          msg.header.frame_id = forceSensors[i]->name();
          tf::wrenchEigenToMsg(forceSensorOffsets_[forceSensors[i]->name()], msg.wrench);
          offsetForcePub_[forceSensors[i]->name()].publish(msg);
        }
      }
      {
        std_msgs::Float64 msg;
        msg.data = robot_->rootLink()->m() - originalRootMass_;
        rootMassOffsetPub_.publish(msg);
      }

      event.last_real;
      event.current_real;
    }

  protected:
    ros::Timer periodicTimer_;
    unsigned int seq_ = 0;

    bool isTopicUpdated_ = false;

    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState::ConstPtr jointStateMsg_;
    ros::Subscriber imuSub_;
    sensor_msgs::Imu::ConstPtr imuMsg_;
    std::vector<ros::Subscriber> forceSensorSub_;
    std::vector<geometry_msgs::WrenchStamped::ConstPtr> forceSensorMsg_;
    ros::Subscriber contactPointSub_;
    multicontact_force_estimator_msgs::ContactPointArray::ConstPtr contactPointMsg_;
    dynamic_reconfigure::Server<multicontact_force_estimator_msgs::MultiContactForceEstimatorConfig> cfgServer_;
    std::map<std::string,ros::Publisher> offsetForcePub_;
    ros::Publisher rootMassOffsetPub_;
    ros::Publisher estimatedForcePub_;
    ros::Publisher estimatedForceMarkerPub_;
    visualization_msgs::MarkerArray estimatedForceMarkerMsg_;

    multicontact_force_estimator_msgs::MultiContactForceEstimatorConfig config_;
    cnoid::BodyPtr robot_;
    std::map<std::string,cnoid::Vector6> forceSensorOffsets_;
    double originalRootMass_;
    std::shared_ptr<prioritized_qp::Task> qpForceSensorTask_;
    std::shared_ptr<prioritized_qp::Task> qpRootTask_;
    std::shared_ptr<prioritized_qp::Task> qpJointTask_;
  };
};


int main(int argc,char** argv){
  ros::init(argc,argv,"multicontact_force_estimator");

  multicontact_force_estimator::multicontact_force_estimator contactForceEstimator;

  ros::spin();

  return 0;
}
