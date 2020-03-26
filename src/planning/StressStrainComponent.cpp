#include "StressStrainComponent.h"
#include "StressStrainComponent.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofa {
namespace component {
namespace controller {

SOFA_DECL_CLASS(StressStrainComponent)

int StressStrainComponentClass = sofa::core::RegisterObject("A component to validate the stress/strain in vitro experiments")
          .template add<StressStrainComponent<component::container::MechanicalObject<defaulttype::Vec3dTypes>>>();

template class SOFA_DEFORMABLE_API StressStrainComponent<component::container::MechanicalObject<defaulttype::Vec3dTypes>>;

} // end namespace controller
} // end namespace component
} // end namespace sofa

