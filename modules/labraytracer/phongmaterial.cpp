/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, October 24, 2017 - 17:17:44
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labraytracer/phongmaterial.h>
#include <labraytracer/util.h>
#include <math.h>

namespace inviwo {

PhongMaterial::PhongMaterial(const vec3& color, const double reflectance, const double shininess,
    const vec3& ambientMaterialColor, const vec3& diffuseMaterialColor, const vec3& specularMatierialColor) 
    : Material(color, reflectance) {

    constexpr double LightIntensity = 100.0;

    shininess_ = shininess;
    ambientMaterialColor_   = Util::scalarMult(LightIntensity, ambientMaterialColor);
    diffuseMaterialColor_   = Util::scalarMult(LightIntensity, diffuseMaterialColor);
    specularMatierialColor_ = Util::scalarMult(LightIntensity, specularMatierialColor);
}

vec3 vectorMultiply(vec3 v1, vec3 v2) { return vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z); }

vec4 PhongMaterial::shade(const RayIntersection& intersection, const Light& light) const {

    // Programming Task 2: Extend this method.
    // This method currently implements a Lambert's material with ideal
    // diffuse reflection.
    // Your task is to implement a Phong shading model.
    //
    // Hints:
    //
    // 1. This function should return the sum of diffuse and specular parts (no ambient part)
    // 2. The used light color for each type (diffuse/specular) from the light source
    //    is the light color divided by the quadratic distance of the light source from
    //    the point of intersection. (quadratic falloff)
    // 3. The view vector V is the direction of the ray which intersects the object.
    // 4. The rest of the terms as per the slides are as follows
    // 5. You have to take into account shininess_ (p), material colors, light colors
    //    light, view, reflection and normal vector.
    //
    //
    // return vec4(Util::scalarMult(cosNL, this->color()), 1.0);
    // 
    // diffuse  = Kd * Ld * cosNL
    //
    // specular = Ks * Ls * (R . V)^a
    // get normal and light direction

    vec3 D = light.getPosition() - intersection.getPosition();
    double dist2 = dot(D, D);
    vec3 N = intersection.getNormal();
    vec3 L = Util::normalize(D);

    double cosNL = std::max(double(dot(N, L)), double(0));

    vec3 R = Util::normalize(-(N * 2 * cosNL - L));
    vec3 V = intersection.getRay().getDirection();

    vec3 Kd = diffuseMaterialColor_;
    vec3 Ld = light.getDiffuseColor();

    vec3 Ks = specularMatierialColor_;
    vec3 Ls = light.getSpecularColor();

    double RV = std::max(double(dot(R, V)), double(0.0));

    vec3 diffuse = vectorMultiply(Kd, Ld) * cosNL;
    vec3 specular = vectorMultiply(Ks, Ls) * pow(RV, shininess_);
    return vec4(1.0 / dist2 * (diffuse + specular), 1);

}

} // namespace
