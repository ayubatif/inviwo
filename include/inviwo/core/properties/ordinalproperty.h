/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 * Version 0.6b
 *
 * Copyright (c) 2012-2014 Inviwo Foundation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Main file authors: Erik Sund�n, Timo Ropinski, Rickard Englund
 *
 *********************************************************************************/

#ifndef IVW_ORDINALPROPERTY_H
#define IVW_ORDINALPROPERTY_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/properties/templateproperty.h>
#include <string>
#include <sstream>

namespace inviwo {

template<typename T>
struct Defaultvalues {};

#define DEFAULTVALUES(type, name, val, min, max, inc) \
    template<> \
    struct Defaultvalues<type> { \
    public: \
        static type getVal() { return val; } \
        static type getMin() { return min; } \
        static type getMax() { return max; } \
        static type getInc() { return inc; } \
        static std::string getName() { return name; } \
    };

DEFAULTVALUES(float, "Float", 0.0f, 0.0f, 1.0f, 0.01f)
DEFAULTVALUES(double, "Double", 0.0, 0.0, 1.0, 0.01)
DEFAULTVALUES(int, "Int", 0, -100, 100, 1)

DEFAULTVALUES(vec2, "FloatVec2", vec2(0.f), vec2(0.f), vec2(1.f), vec2(0.01f))
DEFAULTVALUES(vec3, "FloatVec3", vec3(0.f), vec3(0.f), vec3(1.f), vec3(0.01f))
DEFAULTVALUES(vec4, "FloatVec4", vec4(0.f), vec4(0.f), vec4(1.f), vec4(0.01f))

DEFAULTVALUES(ivec2, "IntVec2", ivec2(0), ivec2(0), ivec2(10), ivec2(1))
DEFAULTVALUES(ivec3, "IntVec3", ivec3(0), ivec3(0), ivec3(10), ivec3(1))
DEFAULTVALUES(ivec4, "IntVec4", ivec4(0), ivec4(0), ivec4(10), ivec4(1))

DEFAULTVALUES(mat2, "FloatMat2", mat2(0.f), mat2(0.f), mat2(1.f), mat2(0.01f))
DEFAULTVALUES(mat3, "FloatMat3", mat3(0.f), mat3(0.f), mat3(1.f), mat3(0.01f))
DEFAULTVALUES(mat4, "FloatMat4", mat4(0.f), mat4(0.f), mat4(1.f), mat4(0.01f))

#undef DEFAULTVALUES

template<typename T>
class OrdinalProperty : public TemplateProperty<T> {
public:
    OrdinalProperty(std::string identifier,
                    std::string displayName,
                    T value = Defaultvalues<T>::getVal(),
                    T minValue = Defaultvalues<T>::getMin(),
                    T maxValue = Defaultvalues<T>::getMax(),
                    T increment = Defaultvalues<T>::getInc(),
                    PropertyOwner::InvalidationLevel invalidationLevel = PropertyOwner::INVALID_OUTPUT,
                    PropertySemantics semantics = PropertySemantics::Default);

    T getMinValue() const;
    T getMaxValue() const;
    T getIncrement() const;

    virtual void set(const T& value) { TemplateProperty<T>::set(value); } // Need to implement this to avoid compiler confusion with set(const Property*)
    virtual void set(const Property* src);
    void setMinValue(const T& value);
    void setMaxValue(const T& value);
    void setIncrement(const T& value);

    virtual std::string getClassName() const;

    virtual Variant getVariant();
    virtual void setVariant(const Variant& v);
    virtual int getVariantType();

    virtual void serialize(IvwSerializer& s) const;
    virtual void deserialize(IvwDeserializer& d);

private:
    T minValue_;
    T maxValue_;
    T increment_;
};

//Scalar properties
typedef OrdinalProperty<float> FloatProperty;
typedef OrdinalProperty<int> IntProperty;
typedef OrdinalProperty<double> DoubleProperty;

//Vector properties
typedef OrdinalProperty<vec2> FloatVec2Property;
typedef OrdinalProperty<vec3> FloatVec3Property;
typedef OrdinalProperty<vec4> FloatVec4Property;

typedef OrdinalProperty<ivec2> IntVec2Property;
typedef OrdinalProperty<ivec3> IntVec3Property;
typedef OrdinalProperty<ivec4> IntVec4Property;

//Matrix properties
typedef OrdinalProperty<mat2> FloatMat2Property;
typedef OrdinalProperty<mat3> FloatMat3Property;
typedef OrdinalProperty<mat4> FloatMat4Property;


template<typename T>
std::string OrdinalProperty<T>::getClassName() const {
    std::stringstream ss;
    ss << Defaultvalues<T>::getName() << "Property";
    return ss.str();
}

template <typename T>
OrdinalProperty<T>::OrdinalProperty(std::string identifier, std::string displayName, T value,
                                    T minValue, T maxValue, T increment,
                                    PropertyOwner::InvalidationLevel invalidationLevel,
                                    PropertySemantics semantics)
    : TemplateProperty<T>(identifier, displayName, value, invalidationLevel, semantics),
      minValue_(minValue),
      maxValue_(maxValue),
      increment_(increment)
{}

template <typename T>
void OrdinalProperty<T>::set(const Property* srcProperty) {
    const OrdinalProperty<T>* templatedSrcProp = dynamic_cast<const OrdinalProperty<T>*>(srcProperty);

    if (templatedSrcProp) {
        minValue_ = templatedSrcProp->getMinValue();
        maxValue_ = templatedSrcProp->getMaxValue();
        increment_ = templatedSrcProp->getIncrement();
        this->value_ = templatedSrcProp->get();
    } else
        this->setVariant(const_cast<Property*>(srcProperty)->getVariant());

    TemplateProperty<T>::propertyModified();
}

template <typename T>
T OrdinalProperty<T>::getMinValue() const {
    return minValue_;
}

template <typename T>
T OrdinalProperty<T>::getMaxValue() const {
    return maxValue_;
}

template <typename T>
T OrdinalProperty<T>::getIncrement() const {
    return increment_;
}

template <typename T>
void OrdinalProperty<T>::setMinValue(const T& value) {
    minValue_ = value;
}

template <typename T>
void OrdinalProperty<T>::setMaxValue(const T& value) {
    maxValue_ = value;
}

template <typename T>
void OrdinalProperty<T>::setIncrement(const T& value) {
    increment_ = value;
}

template <typename T>
Variant OrdinalProperty<T>::getVariant() {
    return Variant(TemplateProperty<T>::value_);
}

template <typename T>
void OrdinalProperty<T>::setVariant(const Variant& v) {
    if (v.canConvert(getVariantType()))
        this->set(v.get<T>());
}

template <typename T>
int OrdinalProperty<T>::getVariantType() {
    return getVariant().getType();
}

template<typename T>
void OrdinalProperty<T>::serialize(IvwSerializer& s) const {
    Property::serialize(s);
    s.serialize("value", TemplateProperty<T>::get());
    s.serialize("minvalue", getMinValue());
    s.serialize("maxvalue", getMaxValue());
    s.serialize("increment", getIncrement());
}


template<typename T>
void OrdinalProperty<T>::deserialize(IvwDeserializer& d) {
    Property::deserialize(d);
    T value;
    d.deserialize("value", value);
    set(value);
    d.deserialize("minvalue", value);
    setMinValue(value);
    d.deserialize("maxvalue", value);
    setMaxValue(value);
    d.deserialize("increment", value);
    setIncrement(value);
}


} // namespace

#endif // IVW_ORDINALPROPERTY_H
