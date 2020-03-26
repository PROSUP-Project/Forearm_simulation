#include "TorqueController.h"
#include "TorqueController.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofa {
namespace component {
namespace controller {

SOFA_DECL_CLASS(TorqueController)

int TorqueControllerClass = core::RegisterObject("Provides a torque control to move the Radius around the Ulna.")
        .template add<TorqueController<component::container::MechanicalObject<defaulttype::Vec3dTypes>,   component::container::MechanicalObject<defaulttype::Vec3dTypes>>>()
        .template add<TorqueController<component::container::MechanicalObject<defaulttype::Rigid3dTypes>, component::container::MechanicalObject<defaulttype::Vec3dTypes>>>()
        .template add<TorqueController<component::container::MechanicalObject<defaulttype::Vec3dTypes>,   component::container::MechanicalObject<defaulttype::Rigid3dTypes>>>()
        .template add<TorqueController<component::container::MechanicalObject<defaulttype::Rigid3dTypes>, component::container::MechanicalObject<defaulttype::Rigid3dTypes>>>()
;

template class SOFA_DEFORMABLE_API TorqueController<component::container::MechanicalObject<defaulttype::Vec3dTypes>,   component::container::MechanicalObject<defaulttype::Vec3dTypes>>;
template class SOFA_DEFORMABLE_API TorqueController<component::container::MechanicalObject<defaulttype::Rigid3dTypes>, component::container::MechanicalObject<defaulttype::Vec3dTypes>>;
template class SOFA_DEFORMABLE_API TorqueController<component::container::MechanicalObject<defaulttype::Vec3dTypes>,   component::container::MechanicalObject<defaulttype::Rigid3dTypes>>;
template class SOFA_DEFORMABLE_API TorqueController<component::container::MechanicalObject<defaulttype::Rigid3dTypes>, component::container::MechanicalObject<defaulttype::Rigid3dTypes>>;



} // end namespace controller
} // end namespace component
} // end namespace sofa

