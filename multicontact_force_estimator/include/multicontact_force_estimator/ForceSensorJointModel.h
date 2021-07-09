#ifndef MULTICONTACT_FORCE_ESTIMATOR_FORCESENSOR_JOINT_MODEL_H
#define MULTICONTACT_FORCE_ESTIMATOR_FORCESENSOR_JOINT_MODEL_H

#include <cnoid/Body>

namespace multicontact_force_estimator {
  namespace forcesensor_joint_model {
    cnoid::BodyPtr convert2forcesensor_joint_model(const cnoid::BodyPtr robot);
    cnoid::LinkPtr getLinkByURDFLinkName(cnoid::BodyPtr robot, const std::string& linkname);
  };
};
#endif
