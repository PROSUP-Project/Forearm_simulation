#include "TorqueController.h"

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/helper/logging/Messaging.h>

#include <SofaSimulationGraph/SimpleApi.h>
using sofa::simpleapi::str ;
using sofa::simpleapi::createObject ;
using sofa::simpleapi::createChild ;


namespace sofa {
namespace component {
namespace controller {

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

template <typename ObjALinkType, typename ObjCLinkType>
TorqueController<ObjALinkType, ObjCLinkType>::TorqueController()
        : maxTorque(initData(&maxTorque, 1.0, "maxTorque", "limits the maximal torque being applied"))
        , kp(initData(&kp, 0.5, "kp", "factor for the proportional factor in the PID"))
        , kd(initData(&kd, 0.3, "kd", "factor for the differentlial factor in the PID"))
        , ki(initData(&ki, 0.0, "ki", "factor for the integral factor in the PID"))
        , indexA(initData(&indexA, 0u, "indexA", "represents the index of the vertex in objectA"))
        , indexB(initData(&indexB, 0u, "indexB", "represents the index of the vertex in objectB"))
        , indexC(initData(&indexC, 0u, "indexC", "represents the index of the vertex in objectC"))
        , indexD(initData(&indexD, 0u, "indexD", "represents the index of the vertex in objectD"))
        , offsetA(initData(&offsetA, sofa::defaulttype::Vector3(0,0,0), "offsetA", "Vec3 for a offset"))
        , offsetB(initData(&offsetB, sofa::defaulttype::Vector3(0,0,0), "offsetB", "Vec3 for a offset"))
        , objectA(initLink("objectA", "reference to the mechanical object of object A"))
        , objectB(initLink("objectB", "reference to the mechanical object of object B"))
        , objectC(initLink("objectC", "reference to the mechanical object of object C"))
        , objectD(initLink("objectD", "reference to the mechanical object of object D"))
        , targetNode(initLink("targetNode", "reference to the node containing the MechanicalObject, which has to be moved"))
        , indexTarget(initData(&indexTarget, 0, "indexTarget", "represents the index of the vertex of the mechanical target object"))
        , debug(initData(&debug, false, "debug", "gives the starting direction"))
        , initDir(initData(&initDir, true, "initDir", "gives the starting direction"))
{}

template <typename ObjALinkType, typename ObjCLinkType>
TorqueController<ObjALinkType, ObjCLinkType>::~TorqueController() {
};

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::init() {

    context = dynamic_cast<simulation::Node *>(this->getContext());
    root = dynamic_cast<simulation::Node *>(context->getRootContext());

    if (!objectA) {
        msg_error("TQ : " + name.getValue()) << "objectA doesn't exists";
    }
    if (!objectB) {
        msg_error("TQ : " + name.getValue()) << "objectB doesn't exists";
    }
    if (!objectC) {
        msg_error("TQ : " + name.getValue()) << "objectC doesn't exists";
    }
    if (!objectD) {
        msg_error("TQ : " + name.getValue()) << "objectD doesn't exists";
    }

    // check if indexes are available
    const auto num_elem_a = objectA->x.getValue().size();
    std::cout<<"num_elem_a= "<<num_elem_a<<std::endl;
    const auto num_elem_b = objectB->x.getValue().size();
    std::cout<<"num_elem_b= "<<num_elem_b<<std::endl;

    const auto num_elem_c = objectC->x.getValue().size();
    std::cout<<"num_elem_c= "<<num_elem_c<<std::endl;

    const auto num_elem_d = objectD->x.getValue().size();
    std::cout<<"num_elem_d= "<<num_elem_d<<std::endl;

    if ((unsigned long) indexA.getValue() >= num_elem_a) {
        if(debug.getValue()) msg_error("TQ : " + name.getValue()) << "indexA doesn't exist in objectA";
    }

    if ((unsigned long) indexB.getValue() >= num_elem_b) {
        if(debug.getValue()) msg_error("TQ : " + name.getValue()) << "indexB doesn't exist in objectB";
    }

    if ((unsigned long) indexC.getValue() >= num_elem_c) {
        if(debug.getValue()) msg_error("TQ : " + name.getValue()) << "indexC doesn't exist in objectC";
    }

    if ((unsigned long) indexD.getValue() >= num_elem_d) {
        if(debug.getValue()) msg_error("TQ : " + name.getValue()) << "indexD doesn't exist in objectD";
    }


    // set the origin
    const auto& orig_rig3 = objectA->x.getValue()[indexA.getValue()];
    getVec(orig_rig3, origin);
    std::cout<<"origin is: "<< origin<<std::endl;

    computeTorsionFieldVectors_init();
    computeTorsionFieldDirection_init();
    // create the TorsionForceField

    // create a node containing a MechanicalObject inside with a Vec3d template on which the torque will be applied

    // get the string of all components
    auto torque_full_path = targetNode.getPath().substr(1);

    // split the according to the delimiter
    auto split_tokens = split(torque_full_path, '/');

    // get the node including the bone
    auto torque_bone_node = root->getChild(split_tokens[0]);

    if (!torque_bone_node) {
        msg_error("TQ : ") << "Bone node not found!!!!";
    }

    // create a node containing the MechObj for the torque
    torque_node = torque_bone_node->createChild(torque_node_name);

    auto torque_bone_surface_node = torque_bone_node->getChild(split_tokens[1]);
    if (!torque_bone_surface_node) {
        msg_error("TQ : ") << "SurfaceBone node not found!!!!";
    }

    auto torque_bone_surface_mo = torque_bone_surface_node->template get<MechanicalObjectRig3>();

    if (!torque_bone_surface_mo) {
        msg_error("TQ : ") << "MechanicalObject inside surface node not found!!!!";
    }

    auto torque_bone_surface_point = torque_bone_surface_mo->x.getValue()[indexTarget.getValue()];

    if (torque_bone_surface_point.ptr() == nullptr) {
        msg_error("TQ : ") << "point with given indexTarget not existing!!!!";
    }

    const auto torque_bone_surface_center = torque_bone_surface_point.getCenter();

    const std::string torque_point_coord_string =
                    str(torque_bone_surface_center(0)) + " " +
                    str(torque_bone_surface_center(1)) + " " +
                    str(torque_bone_surface_center(2));

    // create the MechanicalObject on which the torque will be applied to
    simpleapi::createObject(torque_node, "MechanicalObject", {
            {"name",torque_point_string},
            {"template","Vec3"},
            {"position", torque_point_coord_string}
    });

    // create the mapping for the MechanicalObject
    simpleapi::createObject(torque_node, "RigidMapping", {
            {"name", "torque_mapping"}
    });

    torque_mo = torque_node->template get<MechanicalObjectVec3>();

    if (!torque_mo) {
        msg_error("TQ : ") << "something went terribly wrong with the MechanicalObject creation for the torque";
    }

    // create the shared pointer for it and register it to the simulation
    sofa::core::objectmodel::BaseObjectDescription objDiscr;
    forcefield::TorsionForceField<defaulttype::Vec3dTypes> tmp_forcefield;
    forceField = sofa::core::objectmodel::BaseObject::create<forcefield::TorsionForceField<defaulttype::Vec3dTypes>>(&tmp_forcefield, torque_node->getContext(), &objDiscr);

    // set the parameters for the TorsionForceField
    forceField->name.setValue("Prosup_TorsionForceField");
    forceField->m_indices.setValue(helper::vector<unsigned int>(1, 0));
    forceField->m_origin.setValue(origin);
    forceField->m_torque.setValue(0);
    forceField->m_axis.setValue(rotation_axis);
    forceField->bwdInit();

    // compute the neutral position
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectB->x.getValue()[indexB.getValue()];

    defaulttype::Vector3 p1_vec, p2_vec;
    getVec(p1, p1_vec);
    getVec(p2, p2_vec);

    neutral_direction = ((p2_vec - offsetA.getValue()) - (p1_vec - offsetB.getValue())).normalized();

    // set the beginning direction
    dirVec_origin = neutral_direction;
    if (initDir.getValue()) {
        dir = Direction::NEUTRAL_TO_PRONATION;
    } else {
        dir = Direction::NEUTRAL_TO_SUPNATION;
    }

    // set up the PID controller
    const double dt = 0.1;
    const double max = maxTorque.getValue();
    const double min = -maxTorque.getValue();

    controller.initPID(dt, max, min, kp.getValue(), kd.getValue(), ki.getValue(), debug.getValue());
    controller.setPreError(0);

}




template <typename ObjALinkType, typename ObjCLinkType>
double TorqueController<ObjALinkType, ObjCLinkType>::computeBoneAngle() {

    // compute distance with dot_Product
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectB->x.getValue()[indexB.getValue()];

    defaulttype::Vector3 p1_vec, p2_vec;
    getVec(p1, p1_vec);
    getVec(p2, p2_vec);

    defaulttype::Vec<3, double> p;

    p = (p2_vec - p1_vec).normalized();

    const auto dotProduct = std::min(defaulttype::dot(dirVec_origin, p), 1.0);
    const auto angle = asin(dotProduct);
    //const auto angle = acos(dotProduct); // TODO: à revoir avec Markus

    double distance = angle * 2.0 / 3.1415;

    // overshoot correction
    if(defaulttype::dot(neutral_direction, p) < 0) {
        std::cout<<"overshooting!!"<<"Distance is: "<<distance<<"============================"<<std::endl;
        distance = 2 - distance;
    }

    // increase distance according to position of the bone
    // if the distance computation is only considering the first half of the 180 deg rotation
    if(dir == PRONATION_TO_NEUTRAL || dir == SUPNATION_TO_NEUTRAL) {
        distance += 1;
    }

    // if the bone is in backward movement, set the value to '-'
    if(dir == SUPNATION_TO_NEUTRAL || dir == NEUTRAL_TO_PRONATION) {
        distance *= -1;
    }

    if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "distance -> " << distance;

    return distance;
}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::computeTorsionFieldTorque() {

    auto distance = computeBoneAngle();

    // compute the force to reach the other end
    auto torque = controller.calculate(0, distance);

    if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "torque -> " << torque;

    computed_torque = torque;
}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::computeTorsionFieldVectors() {

    // get the points
//    const auto& p1 = objectB->x.getValue()[indexB.getValue()];
//    const auto& p2 = objectC->x.getValue()[indexC.getValue()];
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectA->x.getValue()[indexA.getValue()];
    const auto& p3 = objectB->x.getValue()[indexB.getValue()];
    const auto& p4 = objectC->x.getValue()[indexC.getValue()];

//    std::cout<<"D = "<<p1<<std::endl;
//    std::cout<<"A = "<<p2<<std::endl;
//    std::cout<<"B = "<<p3<<std::endl;
//    std::cout<<"C = "<<p4<<std::endl;

//    std::cout<<"B - D = "<<p3 - p1<<std::endl;
//    std::cout<<"A - C = "<<p2 - p4<<std::endl;

    defaulttype::Vector3 p1_vec, p2_vec;

    getVec(p1, p1_vec);
    getVec(p2, p2_vec);


    //p1_vec= p1_vec + offsetA.getValue();

    rotation_axis = p2_vec - p1_vec;

    //rotation_axis.at(0)=-26.7338;rotation_axis.at(1)= -30.9023;rotation_axis.at(2)= -196.567;
    //rotation_axis.normalize();
 //   std::cout<<"RA --> "<<rotation_axis<<std::endl;
}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::computeTorsionFieldVectors_init() {
    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

    simulation::Node *prul = root->getChild("prul");
    //OBJExporter* obj  ;

    // get the points
//    const auto& p1 = objectB->x.getValue()[indexB.getValue()];
//    const auto& p2 = objectC->x.getValue()[indexC.getValue()];
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectA->x.getValue()[indexA.getValue()];
    const auto& p3 = objectB->x.getValue()[indexB.getValue()];
    const auto& p4 = objectC->x.getValue()[indexC.getValue()];

//    std::cout<<"D = "<<p1<<std::endl;
//    std::cout<<"A = "<<p2<<std::endl;
//    std::cout<<"B = "<<p3<<std::endl;
//    std::cout<<"C = "<<p4<<std::endl;

//    std::cout<<"B - D = "<<p3 - p1<<std::endl;
//    std::cout<<"A - C = "<<p2 - p4<<std::endl;

    defaulttype::Vector3 p1_vec, p2_vec;

    getVec(p1, p1_vec);
    getVec(p2, p2_vec);


    p1_vec= p1_vec + offsetA.getValue();

    rotation_axis = p2_vec - p1_vec;

    //rotation_axis.at(0)=-26.7338;rotation_axis.at(1)= -30.9023;rotation_axis.at(2)= -196.567;
    //rotation_axis.normalize();
    //std::cout<<"RA ----> "<<rotation_axis<<std::endl;
}

void wait(int seconds){
    clock_t endwait;
    endwait=clock()+seconds*CLOCKS_PER_SEC;
    while (clock()<endwait);
}


template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::computeTorsionFieldDirection() {

    // compute the axis
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectB->x.getValue()[indexB.getValue()];



    defaulttype::Vector3 p1_vec, p2_vec;
    getVec(p1, p1_vec);
    getVec(p2, p2_vec);

    defaulttype::Vec<3, double> p;

    p = (p2_vec - p1_vec).normalized();
    // get the vector, on which to compute the dotProduct on
    if (defaulttype::dot(dirVec_origin, p) < 0.0)  {
        // time to change the state
        switch (dir) {
            case NEUTRAL_TO_PRONATION : {
                if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "DIRECTION CHANGE : N_T_P -> P_T_N -------------------------------------------------------";
                dir = PRONATION_TO_NEUTRAL;
                //std::cout<<"B - D = "<<p2 - p1<<std::endl;
                std::cout<<"waiting 3 seconds..."<<std::endl;
                wait(3);
                // reset some value from the PID controller
                controller.resetIntegralValue();
                break;
            }
            case PRONATION_TO_NEUTRAL : {
                if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "DIRECTION CHANGE : P_T_N -> N_T_S-------------------------------------------------------";
                dir = NEUTRAL_TO_SUPNATION;
                //std::cout<<"B - D = "<<p2 - p1<<std::endl;
                std::cout<<"waiting 3 seconds..."<<std::endl;
                wait(3);

                break;
            }
            case NEUTRAL_TO_SUPNATION : {
                if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "DIRECTION CHANGE : N_T_S -> S_T_N-------------------------------------------------------";
                dir = SUPNATION_TO_NEUTRAL;
                //std::cout<<"B - D = "<<p2 - p1<<std::endl;
                std::cout<<"waiting 3 seconds..."<<std::endl;
                wait(3);

                // reset some value from the PID controller
                controller.resetIntegralValue();
                break;
            }
            case SUPNATION_TO_NEUTRAL : {
                if(debug.getValue()) msg_info("TQ : " + name.getValue()) << "DIRECTION CHANGE : S_T_N -> N_T_P-------------------------------------------------------";
                dir = NEUTRAL_TO_PRONATION;
                //std::cout<<"B - D = "<<p2 - p1<<std::endl;
                std::cout<<"waiting 3 seconds..."<<std::endl;
                wait(3);

                break;
            }
        }

        dirVec_origin = p;

    }

    //std::cout<<"dirVec_origin"<<dirVec_origin<<std::endl;


}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::computeTorsionFieldDirection_init() {

    // compute the axis
    const auto& p1 = objectD->x.getValue()[indexD.getValue()];
    const auto& p2 = objectB->x.getValue()[indexB.getValue()];

    std::cout<<"computeTorsionFieldDirection_init"<<std::endl;
    std::cout<<"D = "<<p1<<std::endl;
    std::cout<<"B = "<<p2<<std::endl;


    defaulttype::Vector3 p1_vec, p2_vec;
    getVec(p1, p1_vec);
    getVec(p2, p2_vec);

    p1_vec= p1_vec + offsetA.getValue();
    p2_vec= p2_vec + offsetB.getValue();

    defaulttype::Vec<3, double> p;

    p = (p2_vec - p1_vec).normalized();




    dirVec_origin = p;
    //std::cout<<"dirVec_origin -->"<<dirVec_origin<<std::endl;


}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::updateTorsionForceField() {

    computeTorsionFieldVectors();
    forceField->m_axis.setValue(rotation_axis);

    computeTorsionFieldDirection();

    computeTorsionFieldTorque();
    forceField->m_torque.setValue(computed_torque);

}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::getVec(const defaulttype::Rigid3dTypes::Coord& elem, defaulttype::Vector3& vec) {
    vec(0) = elem.getCenter()(0);
    vec(1) = elem.getCenter()(1);
    vec(2) = elem.getCenter()(2);
}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::getVec(const defaulttype::Vec3dTypes::Coord& elem, defaulttype::Vector3& vec) {
    vec(0) = elem(0);
    vec(1) = elem(1);
    vec(2) = elem(2);
}

template <typename ObjALinkType, typename ObjCLinkType>
void TorqueController<ObjALinkType, ObjCLinkType>::handleEvent(sofa::core::objectmodel::Event *event) {

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        // compute the vectors for the TosionForcefield
        updateTorsionForceField();

    }
}

} // end namespace controller
} // end namespace component
} // end namespace sofa
