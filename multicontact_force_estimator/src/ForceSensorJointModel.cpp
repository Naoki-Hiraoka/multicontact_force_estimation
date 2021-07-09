#include "multicontact_force_estimator/ForceSensorJointModel.h"

#include <cnoid/ForceSensor>
#include <cnoid/SceneGraph>
#include <iostream>

/*
  力センサがついてるリンクの力をちゃんと推定できない気がする．接触点やマスパラは力センサの子側についている想定になっている．
 */

namespace multicontact_force_estimator {
  namespace forcesensor_joint_model {
    cnoid::BodyPtr convert2forcesensor_joint_model(const cnoid::BodyPtr robot) {

      cnoid::BodyPtr converted_robot = robot->clone();
      int nj = converted_robot->numJoints();
      const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(converted_robot->devices());
      for(size_t i=0; i<forceSensors.size(); i++){
	/*
	  <BEFORE>
	  parent_link - mass - geometry - forcesensor
	  | Tb
	  child_link
	  
	  <AFTER>
	  parent_link - forcesensor
	  | local_p
	  X
	  |
	  Y
	  |
	  Z
	  |
	  Roll
	  |
	  Pitch
	  |
	  Yaw - mass - geometry 
	  | local_p^-1 * Tb
	  child_link
	  
	  注: TbのR成分はcalcForwardKinematicsで考慮されないので、代わりに回転軸を傾ける
	*/
	const std::string& name = forceSensors[i]->name();
	cnoid::LinkPtr parent_link = forceSensors[i]->link();
	std::vector<cnoid::LinkPtr> child_links;
	for(cnoid::LinkPtr link=forceSensors[i]->link()->child();link;link=link->sibling()){
	  child_links.push_back(link);
	}
	for(size_t i=0;i<child_links.size();i++){
	  parent_link->removeChild(child_links[i]);//childのsiblingが代わりにparentのchildになることに注意
	}

	cnoid::LinkPtr linkX = new cnoid::Link();
	linkX->setName(name+"X");
	parent_link->appendChild(linkX);
	linkX->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
	linkX->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitX());
	linkX->setJointId(nj++);//jointIdがないとcalcMassMatrixで考慮されない
	linkX->setOffsetTranslation(forceSensors[i]->p_local());
	cnoid::LinkPtr linkY = new cnoid::Link();
	linkY->setName(name+"Y");
	linkX->appendChild(linkY);
	linkY->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
	linkY->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitY());
	linkY->setJointId(nj++);
	cnoid::LinkPtr linkZ = new cnoid::Link();
	linkZ->setName(name+"Z");
	linkY->appendChild(linkZ);
	linkZ->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
	linkZ->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitZ());
	linkZ->setJointId(nj++);
	cnoid::LinkPtr linkRoll = new cnoid::Link();
	linkRoll->setName(name+"Roll");
	linkZ->appendChild(linkRoll);
	linkRoll->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
	linkRoll->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitX());
	linkRoll->setJointId(nj++);
	cnoid::LinkPtr linkPitch = new cnoid::Link();
	linkPitch->setName(name+"Pitch");
	linkRoll->appendChild(linkPitch);
	linkPitch->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
	linkPitch->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitY());
	linkPitch->setJointId(nj++);
	cnoid::LinkPtr linkYaw = new cnoid::Link();
	linkYaw->setName(name+"Yaw");
	linkPitch->appendChild(linkYaw);
	linkYaw->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
	linkYaw->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitZ());
	linkYaw->setJointId(nj++);

	for(size_t i=0;i<child_links.size();i++){
	  linkYaw->appendChild(child_links[i]);
	  child_links[i]->setOffsetTranslation(linkX->Tb().inverse() * child_links[i]->b());
	}

	linkYaw->setCenterOfMass(parent_link->centerOfMass() - forceSensors[i]->p_local());
	linkYaw->setMass(parent_link->mass());
	linkYaw->setInertia(parent_link->I());
	cnoid::SgPosTransformPtr transform(new cnoid::SgPosTransform);
	transform->setTranslation((- forceSensors[i]->p_local()).cast<Eigen::Vector3f::Scalar>());
	transform->addChild(parent_link->visualShape());
	linkYaw->setVisualShape(transform);
	parent_link->setVisualShape(new cnoid::SgGroup());
	parent_link->setCenterOfMass(cnoid::Vector3::Zero());
	parent_link->setMass(0);
	parent_link->setInertia(cnoid::Matrix3::Zero());	
      }

      converted_robot->setRootLink(converted_robot->rootLink());

      return converted_robot;
    }

    cnoid::LinkPtr getLinkByURDFLinkName(cnoid::BodyPtr robot, const std::string& linkname) {
      for(size_t i=0;i<robot->links().size();i++){
        cnoid::Affine3 tmp;
        cnoid::SgNodePath path = robot->links()[i]->visualShape()->findNode(linkname,tmp);
        if(path.size()!=0){
          return robot->links()[i];
        }
      }
      std::cerr << "[multicontact_force_estimator/forcesensor_joint_model/getLinkByURDFLinkName] link " << linkname << " not found" << std::endl;
      return nullptr;
    }
  };
};
