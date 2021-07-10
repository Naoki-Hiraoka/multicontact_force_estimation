#include <cnoid/Body>
#include <cnoid/JointPath>
#include <Eigen/Sparse>

namespace multicontact_force_estimator{
  namespace jacobian{
    //world系(local=false)/local系(local=true),エンドエフェクタ周り
    Eigen::SparseMatrix<double,Eigen::RowMajor> calcJacobian(const cnoid::JointPathPtr& path,
                                                             const cnoid::Position& T_local,
                                                             bool local=false);
  };
};
