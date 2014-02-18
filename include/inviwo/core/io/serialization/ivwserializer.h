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
 * Main file authors: Peter Steneteg, Sathish Kottravel
 *
 *********************************************************************************/

#ifndef IVW_SERIALIZER_H
#define IVW_SERIALIZER_H

#include <inviwo/core/io/serialization/ivwserializebase.h>
#include <inviwo/core/util/exception.h>

namespace inviwo {

class IvwSerializable;

class IVW_CORE_API IvwSerializer : public IvwSerializeBase {
public:
    IvwSerializer(IvwSerializer& s, bool allowReference=true);
    IvwSerializer(std::string fileName, bool allowReference=true);
    virtual ~IvwSerializer();

    virtual void writeFile();

    // std containers
    template <typename T>
    void serialize(const std::string& key,
                   const std::vector<T>& sVector,
                   const std::string& itemKey);

    template <typename K, typename V, typename C, typename A>
    void serialize(const std::string& key,
                   const std::map<K,V,C,A>& sMap,
                   const std::string& itemKey);


    // strings
    void serialize(const std::string& key,
                   const std::string& data,
                   const bool asAttribute=false);

    // primitive types
    void serialize(const std::string& key, const bool& data);
    void serialize(const std::string& key, const float& data);
    void serialize(const std::string& key, const double& data);
    void serialize(const std::string& key, const int& data);
    void serialize(const std::string& key, const unsigned int& data);
    void serialize(const std::string& key, const long& data);
    void serialize(const std::string& key, const long long& data);
    void serialize(const std::string& key, const unsigned long long& data);

    // glm vector types
    template<class T>
    void serialize(const std::string& key, const glm::detail::tvec4<T, glm::defaultp>& data);
    template<class T>
    void serialize(const std::string& key, const glm::detail::tvec3<T, glm::defaultp>& data);
    template<class T>
    void serialize(const std::string& key, const glm::detail::tvec2<T, glm::defaultp>& data);
    // glm matrix types
    template<class T>
    void serialize(const std::string& key, const glm::detail::tmat4x4<T, glm::defaultp>& data);
    template<class T>
    void serialize(const std::string& key, const glm::detail::tmat3x3<T, glm::defaultp>& data);
    template<class T>
    void serialize(const std::string& key, const glm::detail::tmat2x2<T, glm::defaultp>& data);

    // serializable classes
    void serialize(const std::string& key, const IvwSerializable& sObj);

    // pointers to something of the above.
    template<class T>
    void serialize(const std::string& key, const T* const& data);

protected:
    friend class NodeSwitch;

private:
    template<typename T>
    void serializePrimitives(const std::string& key, const T& data);

    template<class T>
    void serializeVector(const std::string& key, const T& vector, const bool& isColor=false);

    void initialize();

};


template <typename T>
void IvwSerializer::serialize(const std::string& key,
                              const std::vector<T>& sVector,
                              const std::string& itemKey) {
    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    NodeSwitch tempNodeSwitch(*this, newNode);

    for (typename std::vector<T>::const_iterator it = sVector.begin();
         it != sVector.end(); ++it)
        serialize(itemKey, (*it));

    delete newNode;
}


template <typename K, typename V, typename C, typename A>
void IvwSerializer::serialize(const std::string& key,
                              const std::map<K,V,C,A>& sMap,
                              const std::string& itemKey) {
    if (!isPrimitiveType(typeid(K)))
        throw SerializationException("Error: map key has to be a primitive type");

    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    NodeSwitch tempNodeSwitch(*this, newNode);

    for (typename std::map<K, V, C, A>::const_iterator it = sMap.begin();
         it != sMap.end(); ++it) {
        serialize(itemKey, it->second);
        rootElement_->LastChild()->ToElement()->SetAttribute(IvwSerializeConstants::KEY_ATTRIBUTE,
                it->first);
    }

    delete newNode;
}


template<class T>
inline void IvwSerializer::serialize(const std::string& key, const T* const& data) {
    if (!allowRef_)
        serialize(key, *data);
    else {
        if (refDataContainer_.find(data)) {
            TxElement* newNode = refDataContainer_.nodeCopy(data);
            rootElement_->LinkEndChild(newNode);
            refDataContainer_.insert(data, newNode);
        } else {
            serialize(key, *data);
            refDataContainer_.insert(data, rootElement_->LastChild()->ToElement(), false);
        }
    }
}

template<class T>
inline void IvwSerializer::serializePrimitives(const std::string& key, const T& data) {
    TxElement* node = new TxElement(key);
    rootElement_->LinkEndChild(node);
    node->SetAttribute(IvwSerializeConstants::CONTENT_ATTRIBUTE, data);
    delete node;
}

template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tvec4<T, glm::defaultp>& data) {
    serializeVector(key, data);
}
template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tvec3<T, glm::defaultp>& data) {
    serializeVector(key, data);
}
template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tvec2<T, glm::defaultp>& data) {
    serializeVector(key, data);
}

template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tmat4x4<T, glm::defaultp>& data) {
    glm::detail::tvec4<T, glm::defaultp> rowVec;
    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    NodeSwitch tempNodeSwitch(*this, newNode);

    for (glm::length_t i=0; i<4; i++) {
        std::stringstream key;
        key << "row" << i;
        rowVec = glm::detail::tvec4<T, glm::defaultp>(data[i][0], data[i][1], data[i][2], data[i][3]);
        serializeVector(key.str(), rowVec);
    }

    delete newNode;
}
template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tmat3x3<T, glm::defaultp>& data) {
    glm::detail::tvec3<T, glm::defaultp> rowVec;
    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    NodeSwitch tempNodeSwitch(*this, newNode);

    for (glm::length_t i=0; i<3; i++) {
        std::stringstream key;
        key << "row" << i;
        rowVec = glm::detail::tvec3<T, glm::defaultp>(data[i][0], data[i][1], data[i][2]);
        serializeVector(key.str(), rowVec);
    }

    delete newNode;
}
template<class T>
void IvwSerializer::serialize(const std::string& key, const glm::detail::tmat2x2<T, glm::defaultp>& data) {
    glm::detail::tvec2<T, glm::defaultp> rowVec;
    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    NodeSwitch tempNodeSwitch(*this, newNode);

    for (glm::length_t i=0; i<2; i++) {
        std::stringstream key;
        key << "row" << i;
        rowVec = glm::detail::tvec2<T, glm::defaultp>(data[i][0], data[i][1]);
        serializeVector(key.str(), rowVec);
    }

    delete newNode;
}

template<class T>
inline void IvwSerializer::serializeVector(const std::string& key,
        const T& vector,
        const bool& isColor) {
    TxElement* newNode = new TxElement(key);
    rootElement_->LinkEndChild(newNode);
    std::stringstream ss;
    ss.precision(IvwSerializeConstants::STRINGSTREAM_PRECISION);
    ss<<(isColor ? static_cast<int>(vector[0]) : vector[0]);
    newNode->SetAttribute(isColor ? IvwSerializeConstants::COLOR_R_ATTRIBUTE :
                          IvwSerializeConstants::VECTOR_X_ATTRIBUTE,
                          ss.str());

    if (vector.length() >= 2) {
        ss.str(std::string());
        ss<<(isColor ? static_cast<int>(vector[1]) : vector[1]);
        newNode->SetAttribute(isColor ? IvwSerializeConstants::COLOR_G_ATTRIBUTE :
                              IvwSerializeConstants::VECTOR_Y_ATTRIBUTE,
                              ss.str());
    }

    if (vector.length() >= 3) {
        ss.str(std::string());
        ss<<(isColor ? static_cast<int>(vector[2]) : vector[2]);
        newNode->SetAttribute(isColor ? IvwSerializeConstants::COLOR_B_ATTRIBUTE :
                              IvwSerializeConstants::VECTOR_Z_ATTRIBUTE,
                              ss.str());
    }

    if (vector.length() >= 4) {
        ss.str(std::string());
        ss<<(isColor ? static_cast<int>(vector[3]) : vector[3]);
        newNode->SetAttribute(isColor ? IvwSerializeConstants::COLOR_A_ATTRIBUTE :
                              IvwSerializeConstants::VECTOR_W_ATTRIBUTE,
                              ss.str());
    }

    delete newNode;
}

} //namespace
#endif
