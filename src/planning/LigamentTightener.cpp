#include "LigamentTightener.h"

#include <sofa/simulation/Node.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/logging/Messaging.h>

using namespace sofa::core::behavior;

LigamentTightener::LigamentTightener() {

}

void LigamentTightener::reinit() {

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

    msg_warning("We are in here!");

    std::vector<std::string> ligament_names {
        "POC", "DOAC", "CBP", "CBD", "AB", "DOB", "PRUL1", "PRUL2", "DRUL1", "DRUL2"
    };

    for(const auto& ligament_string : ligament_names) {

        // get ligament
        simulation::Node *ligament_node = root->getChild(ligament_string);

        if (ligament_node == nullptr) {
            msg_warning("LigamentTightener") << "Ligament " << ligament_string << " NOT found!";
            continue;
        }

        const std::string deformation_node_name = ligament_string + "_deformationNode";
        simulation::Node *deformation_node = ligament_node->getChild(deformation_node_name);

        if (deformation_node == nullptr) {
            msg_warning("LigamentTightener") << "Node " << deformation_node_name << " NOT found!";
            continue;
        }

        const std::string deformation_strain_node_name = ligament_string + "_StrainNode";
        simulation::Node *deformation_strain_node = deformation_node->getChild(deformation_strain_node_name);


        if (deformation_strain_node == nullptr) {
            msg_warning("LigamentTightener") << "Node " << deformation_strain_node_name << " NOT found!";
            continue;
        }

        std::cout << "deformation force field node name : " << deformation_strain_node->forceField.getName() << "\n";


    }
}

void LigamentTightener::handleEvent(sofa::core::objectmodel::Event *) {

}

namespace sofa {
namespace core {
namespace behavior {

    int LigamentTightenerClass = sofa::core::RegisterObject("This component is dedicated to handel bony anatomies").template add<LigamentTightener>();
    SOFA_DECL_CLASS(LigamentTightener)

} // end namespace behavior
} // end namespcae core
} // end namespace sofa
