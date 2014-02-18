/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 * Version 0.6b
 *
 * Copyright (c) 2013-2014 Inviwo Foundation
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
 * Main file author: Daniel J�nsson
 *
 *********************************************************************************/

#ifndef IVW_BUFFER_REPRESENTATION_H
#define IVW_BUFFER_REPRESENTATION_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/datastructures/datarepresentation.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/datastructures/buffer/buffer.h>

namespace inviwo {

class IVW_CORE_API BufferRepresentation : public DataRepresentation {

public:
    BufferRepresentation(size_t size, const DataFormatBase* format = DataFormatBase::get(), BufferType type = POSITION_ATTRIB,
                         BufferUsage usage = STATIC);
    BufferRepresentation(const BufferRepresentation& rhs);
    BufferRepresentation& operator=(const BufferRepresentation& that);
    virtual BufferRepresentation* clone() const = 0;
    virtual ~BufferRepresentation() {};
    virtual void performOperation(DataOperation*) const {};
    virtual void setSize(size_t size) {  size_ = size; }
    virtual void resize(size_t size) { size_ = size; }


    virtual std::string getClassName() const { return "BufferRepresentation"; }
    /**
     * Return the number of elements in the buffer.
     *
     * @return Number of elements in the buffer
     */
    size_t getSize() const { return size_; }

    /**
     * Return size of buffer element in bytes.
     *
     * @return Size of element in bytes.
     */
    virtual size_t getSizeOfElement() const { return getDataFormat()->getBytesStored(); };

    BufferType getBufferType() const { return type_; }
    BufferUsage getBufferUsage() const { return usage_; }

protected:
    size_t size_;
    BufferType type_;
    BufferUsage usage_;
};

} // namespace

#endif // IVW_BUFFER_REPRESENTATION_H
