#ifndef IVW_OPENGLQT_INFO_H
#define IVW_OPENGLQT_INFO_H

#include <modules/openglqt/openglqtmoduledefine.h>
#include <inviwo/core/util/resourceinfo.h>
#include <modules/opengl/openglinfo.h>

namespace inviwo {

class IVW_MODULE_OPENGLQT_API OpenglQtInfo : public ResourceInfo {
public:    
    OpenglQtInfo();
    virtual ~OpenglQtInfo();
    void printInfo();
    virtual void retrieveStaticInfo() {};
    virtual void retrieveDynamicInfo() {};
    std::vector<int> getGLVersion();
};

} // namespace

#endif // IVW_OPENGLQT_INFO_H
