#ifndef MESHPROCESSING_H
#define MESHPROCESSING_H

#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/core/loader/BaseLoader.h>
#include <sofa/core/loader/PrimitiveGroup.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/helper/fixed_array.h>
#include <sofa/helper/io/MeshSTL.h>

#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/core/loader/BaseLoader.h>
#include <sofa/core/loader/PrimitiveGroup.h>
#include <sofa/core/topology/Topology.h>

#include <sofa/helper/fixed_array.h>
#include <sofa/core/loader/MeshLoader.h>

//sofa::core::loader::MeshLoader


namespace sofa
{

namespace core
{

namespace loader
{
class MeshProcessing : public MeshLoader
{

public:

    MeshProcessing();

    void viewInfo();

    sofa::core::loader::MeshLoader* getMesh(){return m_mesh;};

    void setMesh(sofa::core::loader::MeshLoader* m){m_mesh=m;};


//protected:
sofa::core::loader::MeshLoader *m_mesh;

};



}
}
}


//namespace sofa
//{

//namespace helper
//{

//namespace io
//{

//class MeshProcessing
//{

//public:

//    MeshProcessing();

//    void viewInfo();

//    sofa::helper::io::MeshSTL* getMesh(){return m_mesh;};

//    void setMesh(sofa::helper::io::MeshSTL* m){m_mesh=m;};


//protected:
//sofa::helper::io::MeshSTL *m_mesh;

//};



//}
//}
//}


#endif // MESHPROCESSING_H
