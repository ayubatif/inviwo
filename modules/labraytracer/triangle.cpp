/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, October 17, 2017 - 10:24:30
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labraytracer/triangle.h>
#include <labraytracer/util.h>
#include <memory>

namespace inviwo {

Triangle::Triangle() {}

Triangle::Triangle(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& uvw0,
                   const vec3& uvw1, const vec3& uvw2) {
    mVertices[0] = v0;
    mVertices[1] = v1;
    mVertices[2] = v2;
    mUVW[0] = uvw0;
    mUVW[1] = uvw1;
    mUVW[2] = uvw2;
}

vec3 normalize(vec3 vec) {
    double magnitude = sqrt(dot(vec, vec));
    return vec3(vec.x / magnitude, vec.y / magnitude, vec.z / magnitude);
}

bool Triangle::closestIntersection(const Ray& ray, double maxLambda,
                                   RayIntersection& intersection) const {
    // Programming TASK 1: Implement this method
    // Your code should compute the intersection between a ray and a triangle.
    //
    // If you detect an intersection, the return type should look similar to this:
    // if(rayIntersectsTriangle)
    //{
    //  intersection = RayIntersection(ray,shared_from_this(),lambda,ray.normal,uvw);
    //  return true;
    //}
    //
    // Hints :
    // Ray origin p_r : ray.getOrigin()
    // Ray direction t_r : ray.getDirection()
    // Compute the intersection point using ray.pointOnRay(lambda)

    vec3 v0, v1, v2;
    v0 = mVertices[0];
    v1 = mVertices[1];
    v2 = mVertices[2];

    vec3 t1 = v1 - v0;
    vec3 t2 = v2 - v0;

    vec3 normal = normalize(cross(t1, t2));
    vec3 tr = ray.getDirection();

    double lambda = -1;

    float denom = dot(tr, normal);
    if (denom != 0) {
        vec3 pr = ray.getOrigin();
        vec3 vpr = v0 - pr;
        lambda = dot(vpr, normal) / denom;
    }

    bool rayIntersectsPlane = (lambda >= 0) && (lambda < maxLambda);
    if (rayIntersectsPlane) {
        vec3 uwu = vec3(0, 0, 0);
        vec3 p = ray.pointOnRay(lambda);

        vec3 edge0 = v1 - v0;
        vec3 edge1 = v2 - v1;
        vec3 edge2 = v0 - v2;
        vec3 C0 = p - v0;
        vec3 C1 = p - v1;
        vec3 C2 = p - v2;
        if (dot(normal, cross(edge0, C0)) < 0 || dot(normal, cross(edge1, C1)) < 0 ||
            dot(normal, cross(edge2, C2)) < 0) {
            return false;
        }

        intersection = RayIntersection(ray, shared_from_this(), lambda, normal, uwu);
        return true;
    }

    return false;
}

bool Triangle::anyIntersection(const Ray& ray, double maxLambda) const {
    RayIntersection temp;
    return closestIntersection(ray, maxLambda, temp);
}

void Triangle::drawGeometry(std::shared_ptr<BasicMesh> mesh,
                            std::vector<BasicMesh::Vertex>& vertices) const {
    auto indexBuffer = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    Util::drawLineSegment(mVertices[0], mVertices[1], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
    Util::drawLineSegment(mVertices[1], mVertices[2], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
    Util::drawLineSegment(mVertices[2], mVertices[0], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
}

}  // namespace inviwo
