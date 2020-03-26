#include <sofa/core/objectmodel/BaseObject.h>


//#include <sofa/component/controller/Controller.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionBeginEvent.h>

#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/core/visual/VisualModel.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/helper/system/config.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseTopology/MeshTopology.h>

#include <sofa/defaulttype/Vec.h>

#include <sofa/simulation/Node.h>


#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/accessor.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include "meshprocessing.h"


namespace sofa
{

namespace core
{

namespace behavior
{



template <class DataTypes>
class  MyComponent : public BaseController
{
public:

    SOFA_CLASS(MyComponent, BaseController);



protected:

    MyComponent ();
    virtual ~MyComponent (){};


public:

    //typedef typename sofa::core::objectmodel::Data<Real> Real;
    //typedef typename DataTypes::Real Real;

    sofa::core::objectmodel::Data<int> m_stiffness;
    sofa::core::objectmodel::Data<bool> m_isEnabled;


    void updateOnBeginStep();
    void updateOnEndStep();
    void updateOnCollisionDetection();

    virtual void init()  { reinit(); }
    virtual void reinit();
    virtual void handleEvent(sofa::core::objectmodel::Event *);
    virtual void bwdInit() {};
    virtual void cleanup(){};
    virtual void reset(){};
    void reinitSimulation();

};


} // namespace controller

} // namespace component

} // namespace sofa

