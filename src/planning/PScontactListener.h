#include <sofa/core/objectmodel/BaseObject.h>
#include <SofaBaseCollision/ContactListener.h>
#include <sofa/core/ObjectFactory.h>

class PScontactListener : public sofa::core::collision::ContactListener {
 public:
  SOFA_CLASS(PScontactListener, sofa::core::collision::ContactListener);

 protected:
  PScontactListener(sofa::core::CollisionModel *collModel1 = NULL, sofa::core::CollisionModel *collModel2 = NULL)
      : ContactListener(collModel1, collModel2) { }

  virtual void beginContact(
      const sofa::helper::vector<const sofa::helper::vector<sofa::core::collision::DetectionOutput> *> &contacts)
  {

    for (const sofa::helper::vector<sofa::core::collision::DetectionOutput> *contact : contacts)
    {
        for (const sofa::core::collision::DetectionOutput &detection_output : *contact)
        {
            std::cout << "Object '" << detection_output.elem.first.model->getName() << "' collided with object '" <<
            detection_output.elem.second.model->getName() << "' at positions (" << detection_output.point[0] << ") and (" <<
            detection_output.point[1] << ")\n";
        }
    }

  }
};

//int PScontactListenerClass = sofa::core::RegisterObject("Simple collision component")
//    .add<PScontactListener>();

//SOFA_DECL_CLASS(PScontactListener)
