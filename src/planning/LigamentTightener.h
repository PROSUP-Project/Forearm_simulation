#pragma once

#include <sofa/core/behavior/BaseController.h>

namespace sofa {
namespace core {
namespace behavior {

class LigamentTightener : public BaseController {

public:

    SOFA_CLASS(LigamentTightener, BaseController);

protected:

    LigamentTightener();
    virtual ~LigamentTightener() {};


public:

    virtual void init()  { reinit(); }
    virtual void reinit();
    virtual void handleEvent(sofa::core::objectmodel::Event *);
    virtual void bwdInit() {};
    virtual void cleanup(){};
    virtual void reset(){};

};

} // end namespace behavior
} // end namespace care
} // end namespace sofa