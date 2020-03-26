#pragma once

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/BaseController.h>

#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>
#include <SofaSimulationTree/GNode.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/simulation/Node.h>
#include <SofaLoader/MeshObjLoader.h>
#include <SofaBoundaryCondition/TorsionForceField.h>
#include <sofa/core/objectmodel/Link.h>


#include "../controller/PIDController.h"

namespace sofa {
namespace component {
namespace controller {

template <typename ObjALinkType, typename ObjCLinkType>
class TorqueController : public Controller {

public:

    SOFA_CLASS(TorqueController, Controller);

    using MeshOBJLoader = component::loader::MeshObjLoader;

    using MechanicalObjectRig3 = component::container::MechanicalObject<defaulttype::Rigid3dTypes>;
    using MechanicalObjectVec3 = component::container::MechanicalObject<defaulttype::Vec3dTypes>;

//    using LinkNode = core::objectmodel::SingleLink< TorqueController, simulation::Node, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    using LinkNode = core::objectmodel::SingleLink< TorqueController, MechanicalObjectRig3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    using LinkRig3 = core::objectmodel::SingleLink< TorqueController, MechanicalObjectRig3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    using LinkVec3 = core::objectmodel::SingleLink< TorqueController, MechanicalObjectVec3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;

    using LinkObjA = core::objectmodel::SingleLink< TorqueController, MechanicalObjectRig3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    using LinkObjC = core::objectmodel::SingleLink< TorqueController, MechanicalObjectRig3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;

//    using LinkObjA = core::objectmodel::SingleLink< TorqueController, ObjALinkType, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
//    using LinkObjC = core::objectmodel::SingleLink< TorqueController, ObjCLinkType, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;

    typedef enum {
        NEUTRAL_TO_PRONATION,
        PRONATION_TO_NEUTRAL,
        NEUTRAL_TO_SUPNATION,
        SUPNATION_TO_NEUTRAL
    } Direction;

protected:

    TorqueController();

    virtual ~TorqueController();

    virtual void init() override;

    virtual void handleEvent(sofa::core::objectmodel::Event *) override;

private:

    double computeBoneAngle();
    void computeTorsionFieldTorque();
    void computeTorsionFieldVectors();
    void computeTorsionFieldVectors_init();
    void computeTorsionFieldDirection();
    void computeTorsionFieldDirection_init();

    void updateTorsionForceField();
    void getVec(const defaulttype::Rigid3dTypes::Coord&, sofa::defaulttype::Vector3&);
    void getVec(const defaulttype::Vec3dTypes::Coord&, sofa::defaulttype::Vector3&);

    sofa::defaulttype::Vector3 rotation_axis;
    sofa::defaulttype::Vector3 origin;
    sofa::defaulttype::Vector3 axis_origin;
    sofa::defaulttype::Vector3 neutral_direction;

    PID controller;

    double computed_torque;

    simulation::Node::SPtr context;
    simulation::Node::SPtr root;

    forcefield::TorsionForceField<defaulttype::Vec3dTypes>::SPtr forceField;
    simulation::Node::SPtr torque_node;
    MechanicalObjectVec3::SPtr torque_mo;

    Direction dir;
    sofa::defaulttype::Vector3 dirVec_origin;

    const std::string torque_node_name = "torque_node";
    const std::string torque_point_string = "torque_mo";

public:
    Data<double> maxTorque;
    Data<double> kp;
    Data<double> kd;
    Data<double> ki;
    Data<unsigned int> indexA;
    Data<unsigned int> indexB;
    Data<unsigned int> indexC;
    Data<unsigned int> indexD;
    Data<sofa::defaulttype::Vector3> offsetA;
    Data<sofa::defaulttype::Vector3> offsetB;
    LinkObjA objectA;
    LinkObjA objectB;
    LinkObjC objectC;
    LinkObjC objectD;
    LinkNode targetNode;
    Data<int> indexTarget;
    Data<bool> debug;
    Data<bool> initDir;

};


} // end namespace controller
} // end namespace component
} // end namespace sofa
