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

#include <SofaBoundaryCondition/ConstantForceField.h>
#include <SofaBoundaryCondition/LinearForceField.h>

#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/objectmodel/Data.h>
#include <SofaValidation/EvalPointsDistance.h>
#include <SofaBaseTopology/TopologySubsetData.h>


namespace sofa {
namespace component {
namespace controller {

template <typename DataTypes>
class StressStrainComponent : public Controller {

public:
    SOFA_CLASS(StressStrainComponent, Controller);
    using MeshOBJLoader = component::loader::MeshObjLoader;
    using MechanicalObjectRig3 = component::container::MechanicalObject<defaulttype::Rigid3dTypes>;
    using MechanicalObjectVec3 = component::container::MechanicalObject<defaulttype::Vec3dTypes>;
    using LinkNode = core::objectmodel::SingleLink< StressStrainComponent, simulation::Node, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    //using LinkRig3 = core::objectmodel::SingleLink< StressStrainComponent, MechanicalObjectRig3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    //using LinkVec3 = core::objectmodel::SingleLink< StressStrainComponent, MechanicalObjectVec3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;
    using LinkObjA = core::objectmodel::SingleLink< StressStrainComponent, MechanicalObjectVec3, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK>;

    using Vector3 = sofa::defaulttype::Vector3;
    using Deriv = sofa::defaulttype::Vec3dTypes::Deriv;

protected:
    StressStrainComponent();
    virtual ~StressStrainComponent();
    virtual void init() override;
    virtual void handleEvent(sofa::core::objectmodel::Event *) override;

private:
    simulation::Node::SPtr context;
    simulation::Node::SPtr root;
    simulation::Node::SPtr ligament_node;

    forcefield::LinearForceField<defaulttype::Vec3dTypes>::SPtr l_forceField;
    forcefield::ConstantForceField<defaulttype::Vec3dTypes>::SPtr forceField;
    simulation::Node::SPtr force_node;
    MechanicalObjectVec3::SPtr force_mo;

    const std::string force_node_name = "force_node";
    const std::string force_point_string = "force_mo";

    void getVec(const defaulttype::Vec3dTypes::Coord&, sofa::defaulttype::Vector3&);

public:
//    typedef typename DataTypes::VecCoord VecCoord;
//    typedef typename DataTypes::VecDeriv VecDeriv;
    //typedef typename defaulttype::Vec3dTypes Coord;
//    typedef typename DataTypes::Deriv Deriv;
    //typedef typename Coord::Real Real;
//    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
//    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    typedef typename DataTypes::Coord::value_type Real;
    typedef helper::vector<unsigned int> SetIndexArray;
    typedef sofa::component::topology::PointSubsetData<SetIndexArray> SetIndex;
    typedef typename DataTypes::VecDeriv VecDeriv;


    Data<Real> m_force;
    SetIndex m_points;
    Data<VecDeriv> m_keyForces;
    Data< helper::vector< Real > > m_keyTimes;

    Data<double> incrementalForce;
    Data<double> maxForce;
    Data<double> maxDisplacement;
    Data<sofa::defaulttype::Vec3dTypes::Deriv> ForceDirection;
    //Data<Vector3> ForceDirection;
    LinkNode targetNode;
    LinkObjA MO_ligament;
    Data<unsigned int> indexForce;
    Data<bool> debug;
    Data<bool> initDir;
    Data<std::string> indices;

    //void reinitSimulation();


};


} // end namespace controller
} // end namespace component
} // end namespace sofa
