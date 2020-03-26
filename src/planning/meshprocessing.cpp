#include "meshprocessing.h"
//#include <sofa/core/loader/BaseLoader.h>



 #include <sofa/helper/io/Mesh.h>
 #include <sofa/helper/helper.h>
#include <sofa/helper/io/MeshSTL.h>


#include <sofa/defaulttype/DataTypeInfo.h>

typedef sofa::defaulttype::Vector3 	Vector3;


namespace sofa
{

namespace core
{

namespace loader
{

MeshProcessing::MeshProcessing()
{
    //std::cout<<"view here 1"<<std::endl;

 //m_mesh = new sofa::core::loader::MeshProcessing(  );
 sofa::core::loader::MeshLoader* my_Loader;
my_Loader->setFilename("POC_shells.stl");

   //sofa::core::loader::MeshLoader::positions pos;
    //std::cout<<"here 1"<<std::endl;
    //m_mesh->setFilename("POC_shells.stl");
    std::cout<<"here 1"<<std::endl;



}

void MeshProcessing::viewInfo(){
    std::cout<<"view here 1"<<std::endl;
    //sofa::core::loader::MeshLoader* test =getMesh();
    //getMesh()->setFilename("/home/nhamze/sofa/v16.08/src/applications/plugins/ProsupPlugin/scenes/stable_scenes/mesh_models/radius_2.stl");
    //m_mesh->setFilename("/mesh_models/radius_2.stl");
    std::cout<<"view here 2"<<std::endl;

    // sofa::helper::vector< Vector3 > my_vertices;
    //sofa::helper::vector < sofa::defaulttype::Vec  < 3, SReal > >  	*my_vertices;
    //my_vertices = getMesh()->getVertices ();
    //*my_vertices = getMesh()->positions;
    //std::cout<<"MeshProcessing::viewInfo() -> "<<std::endl;
    //std::cout<<my_vertices[0,0,0]<<" , "<<my_vertices[0,0,1]<<" , "<<my_vertices[0,0,2]<<std::endl;




}


}
}
}







//namespace sofa
//{

//namespace helper
//{

//namespace io
//{

//MeshProcessing::MeshProcessing()
//{
// //sofa::core::loader::MeshLoader* my_Loader;
// //my_Loader
//   //sofa::core::loader::MeshLoader::positions pos;
//  //  std::cout<<"here 1"<<std::endl;
//    //m_mesh->load("ulna_2.stl");
//    //std::cout<<"here 1"<<std::endl;



//}

//void MeshProcessing::viewInfo(){
//    std::cout<<"here 1"<<std::endl;
//    getMesh()->init("ulna_2.stl");
//    std::cout<<"here 2"<<std::endl;
////    sofa::helper::vector< Vector3 > my_vertices;
////    my_vertices = getMesh()->getVertices ();
////    std::cout<<"MeshProcessing::viewInfo() -> "<<std::endl;
////    std::cout<<my_vertices[0,0,0]<<" , "<<my_vertices[0,0,1]<<" , "<<my_vertices[0,0,2]<<std::endl;

//}


//}
//}
//}
