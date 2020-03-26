#include "StressStrainComponent.h"

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/helper/logging/Messaging.h>
#include <SofaSimulationGraph/SimpleApi.h>
#include <sofa/simulation/Simulation.h>
#include <iostream>
#include <fstream>
#include <numeric>

using sofa::simpleapi::str ;
using sofa::simpleapi::createObject ;
using sofa::simpleapi::createChild ;
using namespace sofa::component::misc;

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



//        , indices(initData(&indices, "", "points where the force has to be applied"))

template <typename DataTypes>
StressStrainComponent<DataTypes>::StressStrainComponent()
        : m_force(initData(&m_force, 0.0, "force", "initial applied strecth force"))
        , incrementalForce(initData(&incrementalForce, 5.0, "incrementalForce", "incremental force to be interpolated"))
        , maxForce(initData(&maxForce, 1000.0, "maxForce", "maximum applied stretch force"))
        , maxDisplacement(initData(&maxDisplacement, 30.0, "maxDisplacement", "maximum allowed displacement of the experimental appartus"))
        , ForceDirection(initData(&ForceDirection, "forceDirection", "direction of the applied force"))
        , MO_ligament(initLink("MO_ligament", "reference to the mechanical object of the ligament"))
        , indexForce(initData(&indexForce, 0u, "indexForce", "represents the index of the vertex in objectA"))
        , targetNode(initLink("targetNode", "reference to the node containing the MechanicalObject, which has to be moved"))
        , debug(initData(&debug, false, "debug", "gives the starting direction"))
        , initDir(initData(&initDir, true, "initDir", "gives the starting direction"))
        , m_points(initData(&m_points, "points", "points where the force is applied"))
        , m_keyTimes(initData(&m_keyTimes, "times", "key times for the interpolation"))
        , m_keyForces(initData(&m_keyForces, "forces", "forces corresponding to the key times"))



{}
template <typename DataTypes>
StressStrainComponent<DataTypes>::~StressStrainComponent() {
};

template <typename DataTypes>
void StressStrainComponent<DataTypes>::getVec(const defaulttype::Vec3dTypes::Coord& elem, sofa::defaulttype::Vector3& vec) {
    vec(0) = elem(0);
    vec(1) = elem(1);
    vec(2) = elem(2);
}


template <typename DataTypes>
void StressStrainComponent<DataTypes>::init() {
    context = dynamic_cast<simulation::Node *>(this->getContext());
    root = dynamic_cast<simulation::Node *>(context->getRootContext());

    if (!MO_ligament) {
        msg_error("TQ : " + name.getValue()) << "MO_ligament doesn't exists";
    }


    const auto num_vertices = MO_ligament->x.getValue().size();
    std::cout<<"Number of Vertices= "<<num_vertices<<std::endl;

    if ((unsigned long) indexForce.getValue() >= num_vertices) {
        if(debug.getValue()) msg_error("TQ : " + name.getValue()) << "indexForce doesn't exist in object";
    }



    std::cout<<"force : "<< m_force<<std::endl;
    std::cout<<"Force Direction : "<< ForceDirection.getValue()<<std::endl;

    const std::string force_direction_string =str(ForceDirection.getValue());
    const std::string force_points_string = str(indices);
    const std::string force_global_linear_string = str(m_force);
    auto force_ligament_node = root->getChild("cuboid");
    force_node = force_ligament_node->createChild(force_node_name);



/* ============================== Linear Force Field ================================*/
    // Method 1
//    simpleapi::createObject(force_node, "LinearForceField", {
//            {"points", "@Force_box_roi.indices" },
//            {"force", force_global_linear_string},
//            {"times", "0 0.1  0.2  0.3 0.4 0.5  0.6 0.7 0.8 0.9 1.0 1.1 1.2 1.3 1.4 1.5 1.6 1.7 1.8 1.9 2.0 "},
//            {"forces", "0 0 0  0 10 0  0 20 0  0 30 0  0 40 0  0 50 0  0 60 0  0 70 0  0 80 0  0 90 0  0 100 0  0 110 0  0 120 0  0 130 0  0 140 0  0 150 0  0 160 0  0 170 0  0 180 0  0 190 0 0 200 0 "},
//    });
//Method 2

    sofa::core::objectmodel::BaseObjectDescription lff_objDiscr;
    forcefield::LinearForceField<defaulttype::Vec3dTypes> tmp_lff;
    l_forceField = sofa::core::objectmodel::BaseObject::create<forcefield::LinearForceField<sofa::defaulttype::Vec3dTypes>>(&tmp_lff, force_node->getContext(), &lff_objDiscr);
    //l_forceField->d_force.setValue(sofa::defaulttype::Vec3dTypes::Coord::value_type(1.0));

    std::cout<<"m_force"<<m_force.getValue()<<std::endl;
    std::cout<<"m_points"<<m_points.getValue()<<std::endl;
    std::cout<<"m_keyForces"<<m_keyForces.getValue()<<std::endl;
    std::cout<<"m_keyTimes"<<m_keyTimes.getValue()<<std::endl;

      l_forceField->d_force.setValue(m_force.getValue());
      l_forceField->points.setValue(m_points.getValue());
      l_forceField->d_keyTimes.setValue(m_keyTimes.getValue());
      l_forceField->d_keyForces.setValue(m_keyForces.getValue());

/* ============================== Constant Force Field ================================*/
    // Method 1
//    simpleapi::createObject(force_node, "ConstantForceField", {
//            {"indices", "@Force_box_roi.indices" },
//            {"totalForce", force_direction_string},
//            {"arrowSizeCoef", "0.001"},
//    });



    //Method 2
    // create the shared pointer for it and register it to the simulation
//    sofa::core::objectmodel::BaseObjectDescription objDiscr;
//    forcefield::ConstantForceField<defaulttype::Vec3dTypes> tmp_ff;
//    forceField = sofa::core::objectmodel::BaseObject::create<forcefield::ConstantForceField<defaulttype::Vec3dTypes>>(&tmp_ff, force_node->getContext(), &objDiscr);
//    forceField->d_indices.setValue(helper::vector<unsigned int>(0));
//    forceField->d_force.setValue(sofa::defaulttype::Vec3dTypes::Deriv(0,50,0));
//    forceField->d_arrowSizeCoef.setValue(1);


};



void wait(int seconds){
    clock_t endwait;
    endwait=clock()+seconds*CLOCKS_PER_SEC;
    while (clock()<endwait);
}


template <typename DataTypes>
void StressStrainComponent<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event) {

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {

        context = dynamic_cast<simulation::Node *>(this->getContext());
        root = dynamic_cast<simulation::Node *>(context->getRootContext());

        auto dt = context->getDt();
        auto temps = context->getTime();
        std::cout<<"dt = "<<dt<<"  |    time = "<<temps<<" | ";

        ligament_node = root->getChild("cuboid");
        sofa::component::misc::EvalPointsDistance<defaulttype::Vec3dTypes> *EPD = dynamic_cast<sofa::component::misc::EvalPointsDistance<sofa::defaulttype::Vec3dTypes>*>(ligament_node->getObject("dist")) ;
        auto mean_distance = EPD->distMean.getValue();
        std::cout<<"mean_d = "<<mean_distance<<" | ";
        auto max_distance = EPD->distMax.getValue();
        std::cout<<"            max_d = "<<max_distance<<" mm.";




        auto all_distances = EPD->dist.getValue();

//        std::cout<<" -----------  All distances Before sorting  ---------------------\n";
//        std::cout<<all_distances;
//        std::cout<<"\n --------------------------------------------------------\n";
//        std::cout<<"first val: "<<all_distances[0]<<std::endl;
//        std::cout<<"last  val: "<<all_distances[all_distances.size()-1]<<std::endl;

        std::sort(all_distances.begin(),all_distances.end());
        std::reverse(all_distances.begin(),all_distances.end());

//        std::cout<<" ----------- All distances After sorting ---------------------\n";
//        std::cout<<all_distances;
//        std::cout<<"\n ------------------------------------------------\n";
//        std::cout<<"first val: "<<all_distances[0]<<std::endl;
//        std::cout<<"last  val: "<<all_distances[all_distances.size()-1]<<std::endl;




        size_t length = 60;
        std::vector<Real>   most_deformed(&all_distances[0],&all_distances[length]);
//        std::cout<<"\nmost deformed :";
//        for (auto i=0 ; i< most_deformed.size(); i++)
//            std::cout<<most_deformed[i]<<" | ";
//        std::cout<<"\n";

        auto average = std::accumulate( most_deformed.begin(), most_deformed.end(), 0.0)/most_deformed.size();
        std::cout<<"  |  Average : "<< average;
        //auto most_deformed =memcopy (first, last);
        //auto most_deformed = all_distances[]

        std::ofstream myfile ("stress_displacement.csv",std::ios::app );
        if (myfile.is_open())
        {
            //myfile << max_distance<<",";
             myfile << average<<",";
        }


        bool limit = false;
        if ((max_distance > 30 && limit)||(temps>=120)){
            std::cout<<"Max displacement has been reached! Simulation has to be stopped!"<<std::endl;
            simulation::getSimulation()->print(dynamic_cast<simulation::Node *>(context->getRootContext()));
            this->getContext()->getRootContext()->setAnimate(false);
            std::cout<<"Waiting 5 seconds before re-running the simulation 1 .. 2 .. 3 .. 4 .. 5 .. "<<std::endl;
            wait(5);
            simulation::getSimulation()->reset(dynamic_cast<simulation::Node *>(context->getRootContext()));
            simulation::getSimulation()->init(dynamic_cast<simulation::Node *>(context->getRootContext()));
            this->getContext()->getRootContext()->setAnimate(true);
          }




    }
}




} // end namespace controller
} // end namespace component
} // end namespace sofa
