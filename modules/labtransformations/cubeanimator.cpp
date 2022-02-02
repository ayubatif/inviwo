/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Thursday, October 12, 2017 - 11:11:30
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labtransformations/cubeanimator.h>
#include <cmath>

namespace inviwo
{

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CubeAnimator::processorInfo_
{
    "org.inviwo.CubeAnimator",      // Class identifier
    "Cube Animator",                // Display name
    "KTH Labs",                 // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo CubeAnimator::getProcessorInfo() const
{
    return processorInfo_;
}


CubeAnimator::CubeAnimator()
    :Processor()
    // Ports
    , meshIn_("meshIn")
    , meshOut_("meshOut")
    // Properties 
    // For a FloatProperty 
    // variablename(identifier, display name, init value, minvalue, maxvalue)
    , radius_("radius", "Radius", 6, 1, 8)
    , radius_variance_("radius_variance_", "RadiusVariance", 6, 1, 8)
    , radius_animation_("radius_animation_", "RadiusAnimation", 0, 0, static_cast<float>(2 * M_PI))
    , rotation1_("rotation1_", "Rotation1", 0, 0, static_cast<float>(2 * M_PI))
    , rotation2_("rotation2_", "Rotation2", 0, 0, static_cast<float>(2 * M_PI))
{
    // Add ports
    addPort(meshIn_);
    addPort(meshOut_);
    
    // Add properties
    addProperty(radius_);
    addProperty(radius_variance_);
    addProperty(radius_animation_);
    addProperty(rotation1_);
    addProperty(rotation2_);

}


void CubeAnimator::process()
{
    // Clone the input mesh
    if (!meshIn_.getData()) return;
    auto mesh = meshIn_.getData()->clone();

    // Get the matrix that defines where the mesh is currently
    auto matrix = mesh->getWorldMatrix();

    // Transform the mesh (TODO)
    matrix = glm::rotate(matrix, rotation1_.get() * static_cast<float>(2 * M_PI), vec3(0, 0, 1))
           * glm::translate(vec3(radius_.get() + radius_variance_.get()*std::sin(2*M_PI*radius_animation_.get()), 0, 0)) * matrix
           * glm::rotate(matrix, rotation2_.get() * static_cast<float>(2 * M_PI), vec3(0, 0, 1));

    // Update
    mesh->setWorldMatrix(matrix);
	
    // Set output
    meshOut_.setData(mesh);
}

} // namespace

