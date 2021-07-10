#include <multicontact_force_estimator/Jacobian.h>

namespace multicontact_force_estimator{
  namespace jacobian{
    //world系(local=false)/local系(local=true),エンドエフェクタ周り.
    //J = [root6dof joints]. root6dofはworld系,rootlink周り
    Eigen::SparseMatrix<double,Eigen::RowMajor> calcJacobian(const cnoid::JointPathPtr& path,
                                                             const cnoid::Position& T_local,
                                                             bool local){
      Eigen::SparseMatrix<double,Eigen::RowMajor> J = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6+path->endLink()->body()->numJoints());

      const cnoid::Position target_position = path->endLink()->T() * T_local;
      const cnoid::Vector3 target_p = target_position.translation();

      //root 6dof
      for(size_t j=0;j<6;j++){
        J.coeffRef(j,j) = 1;
      }
      cnoid::Vector3 dp = target_p - path->endLink()->body()->rootLink()->p();
      //  0     p[2] -p[1]
      // -p[2]  0     p[0]
      //  p[1] -p[0]  0
      J.coeffRef(0,4)=dp[2];
      J.coeffRef(0,5)=-dp[1];
      J.coeffRef(1,3)=-dp[2];
      J.coeffRef(1,5)=dp[0];
      J.coeffRef(2,3)=dp[1];
      J.coeffRef(2,4)=-dp[0];

      //joints
      for(size_t j=0;j<path->numJoints();j++){
        int col = 6+path->joint(j)->jointId();
        cnoid::Vector3 omega = path->joint(j)->R() * path->joint(j)->a();
        if(!path->isJointDownward(j)) omega = -omega;
        if(path->joint(j)->jointType() == cnoid::Link::JointType::PRISMATIC_JOINT ||
           path->joint(j)->jointType() == cnoid::Link::JointType::SLIDE_JOINT){
          J.coeffRef(0,col)=omega[0];
          J.coeffRef(1,col)=omega[1];
          J.coeffRef(2,col)=omega[2];
          J.coeffRef(3,col)=0;
          J.coeffRef(4,col)=0;
          J.coeffRef(5,col)=0;
        }
        if(path->joint(j)->jointType() == cnoid::Link::JointType::ROTATIONAL_JOINT ||
           path->joint(j)->jointType() == cnoid::Link::JointType::REVOLUTE_JOINT){
          cnoid::Vector3 dp = omega.cross(target_p - path->joint(j)->p());
          J.coeffRef(0,col)=dp[0];
          J.coeffRef(1,col)=dp[1];
          J.coeffRef(2,col)=dp[2];
          J.coeffRef(3,col)=omega[0];
          J.coeffRef(4,col)=omega[1];
          J.coeffRef(5,col)=omega[2];
        }
      }

      if (local) {
        Eigen::SparseMatrix<double,Eigen::RowMajor> Rinv(6,6);
        cnoid::Matrix3 Rtrans = ( path->endLink()->R() * T_local.linear() ).transpose();
        for(size_t j=0;j<3;j++){
          for(size_t k=0;k<3;k++){
            Rinv.coeffRef(j,k) = Rtrans(j,k);
            Rinv.coeffRef(3+j,3+k) = Rtrans(j,k);
          }
        }
        J = (Rinv * J).eval();
      }

      return J;
    }
  };
};
