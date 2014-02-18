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
 * Main file authors: Timo Ropinski, Sathish Kottravel
 *
 *********************************************************************************/

#ifndef IVW_TEMPLATEPROPERTY_H
#define IVW_TEMPLATEPROPERTY_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/properties/property.h>
#include <inviwo/core/util/variant.h>

namespace inviwo {

template<typename T>
class TemplateProperty : public Property {

public:

    TemplateProperty(std::string identifier, std::string displayName, T value,
                     PropertyOwner::InvalidationLevel invalidationLevel = PropertyOwner::INVALID_OUTPUT,
                     PropertySemantics semantics = PropertySemantics::Default);

    virtual T& get();
    virtual const T& get() const { return value_; };
    virtual void set(const T& value);
    virtual void set(const Property* srcProperty);

protected:
    T value_;
};

template <typename T>
TemplateProperty<T>::TemplateProperty(std::string identifier, std::string displayName, T value,
                                      PropertyOwner::InvalidationLevel invalidationLevel,
                                      PropertySemantics semantics)
    : Property(identifier, displayName, invalidationLevel, semantics),
      value_(value)
{}

template <typename T>
T& TemplateProperty<T>::get() {
    return value_;
}

template <typename T>
void TemplateProperty<T>::set(const T& value) {
    value_ = value;
    propertyModified();
}

template <typename T>
void TemplateProperty<T>::set(const Property* srcProperty) {
    const TemplateProperty<T>* templatedSrcProp = dynamic_cast<const TemplateProperty<T>*>(srcProperty);

    if (templatedSrcProp)
        this->value_ = templatedSrcProp->get();
    else
        this->setVariant(const_cast<Property*>(srcProperty)->getVariant());

    propertyModified();
}


} // namespace

#endif // IVW_TEMPLATEPROPERTY_H
