#include "MyComponent.h"
#include "meshprocessing.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseContext.h>

#include <sofa/core/core.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>


//#include <sofa/helper/io/Mesh.h>
//#include <sofa/helper/helper.h>

#include <sofa/defaulttype/DataTypeInfo.h>

#include <sofa/core/objectmodel/MouseEvent.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
//#include <sofa/core/loader/MeshLoader.h>
//#include<sofa/core/behavior/MechanicalState.h>

//#include </home/nhamze/sofa/v16.08/src_debug/modules/SofaGeneralLoader/MeshGmshLoader.h>
//#include </home/nhamze/sofa/v16.08/src_debug/modules/SofaGeneralLoader/MeshSTLLoader.h>

#include <SofaGeneralLoader/MeshGmshLoader.h>
#include <SofaGeneralLoader/MeshSTLLoader.h>
#include <SofaBaseVisual/VisualModelImpl.h>

#include <SofaBaseTopology/TopologyData.inl>
#include <sofa/core/topology/BaseMeshTopology.h>


#include <iostream>
#include <cmath>


#include <string>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <sofa/helper/system/Locale.h>

using namespace std;


using namespace sofa::component::loader;
using sofa::core::objectmodel::New;
using sofa::defaulttype::Vector3;


typedef struct {
    float x;
    float y;
    float z;
}Point;
Point points;

float rotationMatrix[4][4];
float translationMatrix[4][4];
float InvtranslationMatrix[4][4];
float rotationMatrixA[4][4];
float rotationMatrixB[4][4];



float inputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0};
float outputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0};




// float alpha = 0.135802;
// float beta =  0.155736;
// float gama = 0.978419;
// float step=0;



typedef sofa::defaulttype::Vector3 	Vector3;

typedef sofa::core::objectmodel::Data<sofa::defaulttype::Vec<3, double> > Vec3;



//#include <sofa/simulation/AnimateBeginEvent.h>
//#include <sofa/simulation/AnimateEndEvent.h>


namespace sofa
{

//namespace component
namespace core
{

//namespace controller
namespace behavior
{

//{

//SOFA_DECL_CLASS(MyComponent)

//namespace simulation
//{

using namespace simulation;
//using namespace helper::io;


//using namespace sofa::defaulttype;
//using namespace sofa::helper::io;

template<class DataTypes>


//m_stl_link(initLink("test_link", "link from MyComponent to TSTLMesh type")),
//m_ogl_link(initLink("test_ogl_link", "link from MyComponent to OGLModel type")),
//m_mesh_link(initLink("test_mesh_link", "link from MyComponent to Mesh type")),
//m_stiffness(initData(&m_stiffness, 1, "stiffness", "Here should be a short description of myparam.")),
//m_isEnabled(initData(&m_isEnabled, true, "isEnabled", "Boolean indicating if the component is enable"))

MyComponent<DataTypes>::MyComponent():

                           m_stiffness(initData(&m_stiffness, 1, "stiffness", "Here should be a short description of myparam.")),
                           m_isEnabled(initData(&m_isEnabled, true, "isEnabled", "Boolean indicating if the component is enable"))

{


}

//----------------------------------------------------------------------

//MyComponent<DataTypes>::~MyComponent(){
//    //std::cout<<"MyComponent::~MyComponent(): m_stiffness = "<<m_stiffness.getValue()<<std::endl;
//}
//----------------------------------------------------------------------
template<class DataTypes>
void MyComponent<DataTypes>::reinit(){

    // idleStep=0;

//    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
//    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());
//    simulation::Node *DOB = root->getChild("DOB");


//    if(!DOB)
//        cerr<<"DOB not found"<<sendl;

//    core::objectmodel::BaseObject * BO= DOB->getObject("MO");
//    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();
//    MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction

    //sofa::defaulttype::Vector3 rot = MO->getRotation();
    //MO->getIndicesInSpace(indices,x-d , x+d, y-d , y+d , z-d, z+d );
    std::cout<<"Fini lÃ  ... "<<std::endl;





//std::cout<<"MyComponent::reinit(): m_stiffness = "<<m_stiffness.getValue()<<std::endl;






//    helper::system::TemporaryLocale locale(LC_ALL, "C");




//    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
//      simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());



////     simulation::Node *Radius = root->getChild("Radius")->getChild("Radius_surface");
////    simulation::Node *Ulna = root->getChild("Ulna")->getChild("Ulna_surface");
//    simulation::Node *DOB = root->getChild("DOB");

////        if(!Radius)
////             cerr<<"Radius not found"<<sendl;

////    if(!Ulna)
////         cerr<<"Ulna not found"<<sendl;


//      if(!DOB)
//           cerr<<"DOB not found"<<sendl;

////    core::objectmodel::BaseObject * BO= Radius->getObject("MO_Radius_surf");
////    core::objectmodel::BaseObject * BO= Ulna->getObject("MO_Ulna_surf");
//      core::objectmodel::BaseObject * BO= DOB->getObject("MO");


//    if(!BO)
//       cerr<<"BO IS NULL !"<<sendl;


//  //sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();
//    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();




//    std::ifstream in("../models/iom/DOB_insertion.csv");

//    if (!in) {
//        std::cerr << "Error: cannot write output file" << std::endl;
//        exit(0);
//    }
//    double x,y,z;
//    std::string line;
//    typedef boost::tokenizer<boost::escaped_list_separator<char> > Tokenizer;
//    sofa::helper::vector<unsigned> indices, constraints;
//    double d= 0.000001;

//    // read line by line
//    while (getline(in, line)) {
//         d= 0.000001;
//        std::cout<<"line: "<<line<<std::endl;
//        Tokenizer tok(line);

//        for(Tokenizer::iterator iter=tok.begin(); iter!=tok.end();++iter){
//            x = std::stod(*iter);
//            iter++;
//            y = std::stod(*iter);
//            iter++;
//            z = std::stod(*iter);
//        }
//        std::cout<<"x: "<<x<<"  y: "<<y<<"  z: "<<z<<std::endl;


//         MO->getIndicesInSpace(indices,x-d , x+d, y-d , y+d , z-d, z+d );
//         std::cout<<"Initial settings: d = "<<d<<"  indices size = "<<indices.size()<<std::endl;

//         while(indices.size()<1){
//             d=d*2;
//             MO->getIndicesInSpace(indices,x-d , x+d, y-d , y+d , z-d, z+d );
//             std::cout<<"d = "<<d<<"  indices size = "<<indices.size()<<std::endl;

//         }

//         for (int i=0; i<indices.size(); i++)
//             std::cout<<"Indice["<<i<<"] is : "<<indices[i]<<std::endl;

//         constraints.push_back(indices[0]);
//         indices.clear();


//    }


//    std::cout<<"indices = ";
//    for (int i=0; i<constraints.size(); i++)
//        std::cout<<constraints[i]<<" ";
//    std::cout<<std::endl;

}

//----------------------------------------------------------------------
template<class DataTypes>
void MyComponent<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event){


    if (dynamic_cast<sofa::simulation::CollisionBeginEvent *>(event))
    {

        updateOnCollisionDetection();
    }


    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {

        updateOnBeginStep();
    }

    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        updateOnEndStep();
    }

    // Mouse Events
    if(sofa::core::objectmodel::MouseEvent* ev = dynamic_cast< sofa::core::objectmodel::MouseEvent *>(event))
     {
         switch(ev->getState())
         {
             // The left key of the mouse is pressed
             case sofa::core::objectmodel::MouseEvent::LeftPressed:
             {
             std::cout<<"Left mouse button pressed!";
             }
         }
     }


    // Key pressed event
    if(sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast< sofa::core::objectmodel::KeypressedEvent *>(event))
    {
        switch(ev->getKey())
        {
            // The key M is pressed
            case 'M':
            case 'm':
            {
            std::cout<<"keypressed";
            }
        }
    }

        //std::cout<<"MyComponent::handleEvent"<<std::endl;
        //sofa::simulation::AnimateBeginEvent *ev;
}

//-------------------------------------------------------------------

template<class DataTypes>
void MyComponent<DataTypes>::updateOnBeginStep(){

    // idleStep++;
    // std::cout<<"idlestep= "<<idleStep<<std::endl;
    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

// // simulation according to radius
//   simulation::Node *Radius = root->getChild("Radius");
//      if(!Radius)
//           cerr<<"Radius not found"<<sendl;

//  core::objectmodel::BaseObject * BO= Radius->getObject("MO_Radius");
//  if(!BO)
//     cerr<<"BO IS NULL !"<<sendl;


//sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();
////  sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();






  //----------------------------------------------------------

  simulation::Node *Radius = root->getChild("Radius");
       if(!Radius)
            cerr<<"Radius not found"<<sendl;

   core::objectmodel::BaseObject * BO= Radius->getObject("MO_Radius");
   if(!BO)
      cerr<<"BO IS NULL !"<<sendl;


  sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



  if(!MO)
   cerr<<"MO IS NULL !"<<sendl;

//  MO->applyTranslation(115.14,-473.06,73.985);
//  MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//  MO->applyTranslation(-115.14,473.06,-73.985);

//RAF_P: -11.514  47.306  -7.3985
//RAF_D: -8.0347  25.542  -3.5502
//[u,v,w]: -0.155508  0.972745  -0.172

  MO->applyTranslation(27.39, -121.61, 368.12);
  MO->applyRotation(0.135802, 0.155736,  0.978419); // patient 609 left : rotating radius around ulna
  MO->applyTranslation(-27.39, 121.61, -368.12);

//      RAF_P: -27.39  121.61  -368.12
//      RAF_D: -0.14  152.86  -171.79
//      [u,v,w]: -0.135802  -0.155736  -0.978419



//  step++;
//  //if (step < 20){
//  MO->applyTranslation(27.39, -121.61, 368.12);
//  MO->applyRotation(alpha, beta, gama); // patient 609 left : rotating radius around ulna
//  MO->applyTranslation(-27.39, 121.61, -368.12);
////}













  //----------------------------------------------------------
//  simulation::Node *Lunate = root->getChild("Lunate");
//     if(!Lunate)
//          cerr<<"Lunate not found"<<sendl;

//  BO= Lunate->getObject("MO_Lunate");
// if(!BO)
//    cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
// cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction

//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------

//simulation::Node *Capitate = root->getChild("Capitate");
//     if(!Capitate)
//          cerr<<"Capitate not found"<<sendl;

// BO= Capitate->getObject("MO_Capitate");
// if(!BO)
//    cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
// cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);


////----------------------------------------------------------

//simulation::Node *Hamate = root->getChild("Hamate");
//   if(!Hamate)
//        cerr<<"Hamate not found"<<sendl;

//BO= Hamate->getObject("MO_Hamate");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------




//simulation::Node *Triquetrum = root->getChild("Triquetrum");
//   if(!Triquetrum)
//        cerr<<"Triquetrum not found"<<sendl;

//BO= Triquetrum->getObject("MO_Triquetrum");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------



//simulation::Node *Trapezium = root->getChild("Trapezium");
//   if(!Trapezium)
//        cerr<<"Trapezium not found"<<sendl;

//BO= Trapezium->getObject("MO_Trapezium");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------



//simulation::Node *Trapezoid = root->getChild("Trapezoid");
//   if(!Trapezoid)
//        cerr<<"Trapezoid not found"<<sendl;

//BO= Trapezoid->getObject("MO_Trapezoid");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------



//simulation::Node *Scaphoid = root->getChild("Scaphoid");
//   if(!Scaphoid)
//        cerr<<"Scaphoid not found"<<sendl;

//BO= Scaphoid->getObject("MO_Scaphoid");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------





//simulation::Node *Pisiform = root->getChild("Pisiform");
//   if(!Pisiform)
//        cerr<<"Pisiform not found"<<sendl;

//BO= Pisiform->getObject("MO_Pisiform");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------




//simulation::Node *MetaCarpal1 = root->getChild("MetaCarpal1");
//   if(!MetaCarpal1)
//        cerr<<"MetaCarpal1 not found"<<sendl;

//BO= MetaCarpal1->getObject("MO_MC1");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------





//simulation::Node *MetaCarpal2 = root->getChild("MetaCarpal2");
//   if(!MetaCarpal2)
//        cerr<<"MetaCarpal2 not found"<<sendl;

//BO= MetaCarpal2->getObject("MO_MC2");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------





//simulation::Node *MetaCarpal3 = root->getChild("MetaCarpal3");
//   if(!MetaCarpal3)
//        cerr<<"MetaCarpal3 not found"<<sendl;

//BO= MetaCarpal3->getObject("MO_MC3");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------





//simulation::Node *MetaCarpal4 = root->getChild("MetaCarpal4");
//   if(!MetaCarpal4)
//        cerr<<"MetaCarpal4 not found"<<sendl;

//BO= MetaCarpal4->getObject("MO_MC4");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------





//simulation::Node *MetaCarpal5 = root->getChild("MetaCarpal5");
//   if(!MetaCarpal5)
//        cerr<<"MetaCarpal5 not found"<<sendl;

//BO= MetaCarpal5->getObject("MO_MC5");
//if(!BO)
//  cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
//cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

////MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction
//MO->applyRotation(-0.155390064,0.972763436,-0.172003562); //inverse direction
//MO->applyTranslation(-115.14,473.06,-73.985);

////----------------------------------------------------------




//until here





 // MO->applyTranslation(-0.25,0,0);



//  simulation::Node *Ulna = root->getChild("Ulna");
//     if(!Ulna)
//          cerr<<"Ulna not found"<<sendl;

// BO= Ulna->getObject("MO_Ulna");
// if(!BO)
//    cerr<<"BO IS NULL !"<<sendl;


//MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();



//if(!MO)
// cerr<<"MO IS NULL !"<<sendl;

//MO->applyTranslation(115.14,-473.06,73.985);

//MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//MO->applyTranslation(-115.14,473.06,-73.985);




//  sofa::helper::vector<unsigned> indices;
//  double d= 0.4;
//  double x,y,z;
//  x = -98.4392 ;  y = 314.073 ;  z = -49.7337;
//  MO->getIndicesInSpace(indices,x-d , x+d, y-d , y+d , z-d, z+d );

//  std::cout<<"Indices size is : "<<indices.size()<<std::endl;
//  for (int i=0; i<indices.size(); i++)
//      std::cout<<"Indice["<<i<<"] is : "<<indices[i]<<std::endl;



/*
 *
 * ON AB
 * "cote_radius" indices="0 2 39 59 79 99 119 139 159 179 199 219"
 * "cote_ulna"   indices="37 57 77 97 117 137 157 177 197 217 237 257"


 *
  radius
x = -94.4144 ;  y = 345.989 ;  z = -47.0227;
x = -93.6911 ;  y = 343.527 ;  z = -46.4027;
x = -93.1842 ;  y = 341.066 ;  z = -45.5817;
x = -92.2402 ;  y = 338.604 ;  z = -45.167;
x = -91.9539 ;  y = 336.143 ;  z = -44.2;
x = -90.7237 ;  y = 333.681 ;  z = -43.8163;
x = -90.7771 ;  y = 331.22 ;  z = -42.6955;
x = -90.624 ;  y = 328.758 ;  z = -41.4597;
x = -89.5158 ;  y = 326.296 ;  z = -41.4597;
x = -89.4935 ;  y = 323.835 ;  z = -40.3333;
x = -88.8243 ;  y = 322.604 ;  z = -40.224;
x = -88.2633 ;  y = 320.143 ;  z = -39.6226;

indices=" 4986 4891 4789 4688 4590 4494 4396 4298 4201 4098 4052 3948"

  ulna
    x = -98.4287 ;  y = 315.405 ;  z = -50.1145;
    x = -98.4392 ;  y = 314.073 ;  z = -49.7337;
    x = -97.1062 ;  y = 311.409 ;  z = -49.1453;
    x = -97.1062 ;  y = 310.33 ;  z = -48.7804;
    x = -97.1062 ;  y = 308.745 ;  z = -48.2981;
    x = -95.7733 ;  y = 307.413 ;  z = -48.1995;
    x = -95.7733 ;  y = 306.081 ;  z = -47.7544;
    x = -95.7733 ;  y = 303.417 ;  z = -47.093;
    x = -94.4403 ;  y = 302.084 ;  z = -46.7935;
    x = -94.4403 ;  y = 300.752 ;  z = -46.431;
    x = -94.0899 ;  y = 299.42 ;  z = -46.112;
    x = -94.4403 ;  y = 298.088 ;  z = -45.8233;
  indices=" 2203 2161 2080 2079 1997 1959 1923 1833 1799 1758 1717 1670 "

  */









/*
    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

   simulation::Node *Radius = root->getChild("Radius");
      if(!Radius)
           cerr<<"Radius not found"<<sendl;

  core::objectmodel::BaseObject * BO= Radius->getObject("MO_Radius");
  if(!BO)
     cerr<<"BO IS NULL !"<<sendl;



  sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();


  if(!MO)
   cerr<<"MO IS NULL !"<<sendl;


        MO->applyTranslation(115.14,-473.06,73.985);

        MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

        MO->applyTranslation(-115.14,473.06,-73.985);


        sofa::helper::vector<unsigned> indices;

        //MO->getIndicesInSpace(indices, -45, -0.358 , 0.297, -45, -0.358 , 0.297 ) ; //ulna
        // MO->getIndicesInSpace(indices, -45-0.2, -0.358+0.2 , 0.297-0.2, -45+0.2, -0.358-0.2 , 0.297+0.2 ) ;


        //MO->getIndicesInSpace(indices, -35, 35 ,-35, 35,-35, 35 ) ;
        //MO->getIndicesInSpace(indices, -25, 25 ,-25, 25,-25, 25 ) ;
        //MO->getIndicesInSpace(indices, -95, -25 ,-25, -45,25, 25 ) ;
        double d= 0.5;
        MO->getIndicesInSpace(indices,-25-d , -25+d, -0.474696-d , -0.474696+d , -0.537704-d, -0.537704+d );
//        MO->getIndicesInSpace(indices, -24 , -1.474696 , -1.537704,-26 , -2.474696 , -2.537704 );


//Radius
//-25 , 6.72397 , -7.35074
//-25 , 6.72397 , -7.35074
//-25 , 3.12464 , -3.94422
//-25 , 3.12464 , -3.94422
//-25 , -0.474696 , -0.537704
//-25 , -0.474696 , -0.537704
//-25 , -0.474696 , -0.537704
//-25 , -3.99342 , 2.95682
//-25 , -3.99342 , 2.95682
//-25 , -7.51214 , 6.45135

//Ulna
//-45 , 9.3218 , -6.06845
//-45 , 5.2256 , -5.27191
//-45 , 5.2256 , -5.27191
//-45 , 4.10734 , -0.87175
//-45 , -0.3576 , 0.297151
//-45 , -0.3576 , 0.297151
//-45 , -0.3576 , 0.297151
//-45 , -4.78874 , 1.45026
//-45 , -5.87321 , 5.83463
//-45 , -5.87321 , 5.83463


        std::cout<<"Indices size is : "<<indices.size()<<std::endl;
        for (int i=0; i<indices.size(); i++)
            std::cout<<"Indice["<<i<<"] is : "<<indices[i]<<std::endl;

*/



/*

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());





   simulation::Node *Radius = root->getChild("Radius");
      if(!Radius)
           cerr<<"Radius not found"<<sendl;

  core::objectmodel::BaseObject * BO= Radius->getObject("MO_Radius");
  if(!BO)
     cerr<<"BO IS NULL !"<<sendl;



//  sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();
  sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();

  if(!MO)
   cerr<<"MO IS NULL !"<<sendl;


        MO->applyTranslation(115.14,-473.06,73.985);

        MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

        MO->applyTranslation(-115.14,473.06,-73.985);



*/
        // ----------------- POC ligaments ------------


//        simulation::Node *Lunatum = root->getChild("poc_shell");
//           if(!Lunatum)
//                cerr<<"POC not found"<<sendl;

//       core::objectmodel::BaseObject * BO_poc= Lunatum->getObject("MO_poc");
//       if(!BO_poc)
//          cerr<<"BO_poc IS NULL !"<<sendl;



//       sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO_poc = BO_poc->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();

//       if(!MO_poc)
//        cerr<<"MO_poc IS NULL !"<<sendl;

//       MO_poc->applyTranslation(115.14,-473.06,73.985);

//       MO_poc->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//       MO_poc->applyTranslation(-115.14,473.06,-73.985);


        // ----------------- Lunatum ------------


//        simulation::Node *Lunatum = root->getChild("Lunate");
//           if(!Lunatum)
//                cerr<<"Lunatum not found"<<sendl;

//       core::objectmodel::BaseObject * BO1= Lunatum->getObject("MO_Lunate");
//       if(!BO1)
//          cerr<<"BO1 IS NULL !"<<sendl;



//       sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO1 = BO1->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();

//       if(!MO1)
//        cerr<<"MO1 IS NULL !"<<sendl;

//       MO1->applyTranslation(115.14,-473.06,73.985);

//       MO1->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//       MO1->applyTranslation(-115.14,473.06,-73.985);



//   // ----------------- Triquetrum ------------

//             simulation::Node *Triquetrum = root->getChild("Triquetrum");
//                if(!Triquetrum)
//                     cerr<<"Triquetrum not found"<<sendl;

//            core::objectmodel::BaseObject * BO2= Triquetrum->getObject("MO_Triquetrum");
//            if(!BO2)
//               cerr<<"BO2 IS NULL !"<<sendl;



//            sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO2= BO2->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();

//            if(!MO2)
//             cerr<<"MO2 IS NULL !"<<sendl;

//            MO2->applyTranslation(115.14,-473.06,73.985);

//            MO2->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//            MO2->applyTranslation(-115.14,473.06,-73.985);







//question7.scn


//    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
//    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());




//    simulation::Node *Radius = root->getChild("Lunatum");
//       if(!Radius)
//            cerr<<"Radius not found"<<sendl;

//   core::objectmodel::BaseObject * BO= Radius->getObject("MO_Lunatum");
//   if(!BO)
//      cerr<<"BO IS NULL !"<<sendl;



//   sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();


//   if(!MO)
//    cerr<<"MO IS NULL !"<<sendl;


////         MO->applyTranslation(11.514,-47.306,7.3985);

////         MO->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

////         MO->applyTranslation(-11.514,47.306,-7.3985);





//   simulation::Node *RadiusCenter = root->getChild("pt");
//      if(!RadiusCenter)
//           cerr<<"Radius not found"<<sendl;

//  core::objectmodel::BaseObject * BORC= RadiusCenter->getObject("OM_pt");
//  if(!BORC)
//     cerr<<"BORC IS NULL !"<<sendl;



//  sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MORC = BORC->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();

//  if(!MORC)
//   cerr<<"MO IS NULL !"<<sendl;


//        MORC->applyTranslation(11.514,-47.306,7.3985);

//        MORC->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//        MORC->applyTranslation(-11.514,47.306,-7.3985);








       // MO->computeBBox();


//  simulation::Node *rot_centre = root->getChild("rot_centre");
//     if(!rot_centre)
//          cerr<<"rot_centre not found"<<sendl;

//  core::objectmodel::BaseObject * BO_rc= rot_centre->getObject("MO_rot_centre");
//  if(!BO_rc)
//     cerr<<"BO IS NULL !"<<sendl;



//  sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3dTypes> *MO_rc = BO_rc->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3dTypes> >();

//  if(!MO_rc)
//   cerr<<"MO IS NULL !"<<sendl;



//  MO_rc->applyTranslation(11.514,-47.306,7.3985);

//  MO_rc->applyRotation(0.155390064,-0.972763436,0.172003562); //inverse direction

//  MO_rc->applyTranslation(-11.514,47.306,-7.3985);










        //this->getXmin(),this->getXmax(),this->getYmin(),this->getYmax(),this->getZmin(),this->getZmax();
        //float min_x,min_y,min_z,max_x,max_y,max,z;
//        min_x = BO->getXmin();
//        min_y = BO->getYmin();
//        min_z = BO->getZmin();
//        max_x = BO->getXmax();
//        max_y = BO->getYmax();
//        max_z = BO->getZmax();

}


//-------------------------------------------------------------


template<class DataTypes>
void MyComponent<DataTypes>::updateOnEndStep(){

    // compute_angles();

}




//-------------------------------------------------------------



template<class DataTypes>
void MyComponent<DataTypes>::updateOnCollisionDetection(){
   //cerr<<"COLLISION DETECTED!"<<sendl;
   //std::cout<<this-><<std::end;





}

//-------------------------------------------------------------

template<class DataTypes>
void MyComponent<DataTypes>::reinitSimulation()
{
    std::cout<<"MyComponent::reinitSimulation"<<std::endl;

    /// find all mechanical states and set position = rest_position = reset_position, zero velocities and applied forces
//    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
//    simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

//    /// reinit
//    simulation::getSimulation()->reset(root);
//    simulation::getSimulation()->init(root);
}

//-------------------------------------------------------------
// template<class DataTypes>
// void MyComponent<DataTypes>::compute_angles(){
//
//     std::cout<<"compute angles ----"<<std::endl;
//     simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
//     simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());
//
//
//
//     simulation::Node *Radius = root->getChild("Radius")->getChild("Surface_Radius");
//     simulation::Node *Ulna = root->getChild("Ulna")->getChild("Surface_Ulna");
//
//     if(!Radius)
//         cerr<<"Radius not found"<<sendl;
//
//     if(!Ulna)
//         cerr<<"Ulna not found"<<sendl;
//
//
//     core::objectmodel::BaseObject * BO1= Radius->getObject("mo_Surface_Radius");
//     core::objectmodel::BaseObject * BO2= Ulna->getObject("mo_Surface_Ulna");
//
//
//     if(!BO1)
//         cerr<<"BO1 IS NULL !"<<sendl;
//
//
//     if(!BO2)
//         cerr<<"BO2 IS NULL !"<<sendl;
//
//
//     sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO1 = BO1->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();
//     sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> *MO2 = BO2->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::RigidTypes> >();
//
//     //    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> *MO = BO->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> >();
//
//
//
// }

//----------------------------------------------------------------------

int MyComponentClass = sofa::core::RegisterObject("This component is dedicated to handel bony anatomies").add<MyComponent<int> >();
SOFA_DECL_CLASS(MyComponent)


//int MyComponentClass = sofa::core::RegisterObject("This is a trial initial controller");

} // namespace controller

} // namespace component

} // namespace sofa

//} //namespace simulation
//}
