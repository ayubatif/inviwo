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
 * Main file authors: Erik Sund�n, Timo Ropinski
 *
 *********************************************************************************/

#ifndef IVW_IMAGERAM_H
#define IVW_IMAGERAM_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/datastructures/image/imagerepresentation.h>
#include <inviwo/core/datastructures/image/layerram.h>

namespace inviwo {

class LayerRAM;

class IVW_CORE_API ImageRAM : public ImageRepresentation {

public:
    ImageRAM();
    ImageRAM(const ImageRAM& rhs);
    ImageRAM& operator=(const ImageRAM& that);
    virtual ImageRAM* clone() const;
    virtual ~ImageRAM();

    virtual void initialize();
    virtual void deinitialize();
    virtual std::string getClassName() const;

    virtual bool copyAndResizeRepresentation(DataRepresentation*) const;

    LayerRAM* getColorLayerRAM(size_t idx = 0);
    LayerRAM* getDepthLayerRAM();
    LayerRAM* getPickingLayerRAM();

    const LayerRAM* getColorLayerRAM(size_t idx = 0) const;
    const LayerRAM* getDepthLayerRAM() const;
    const LayerRAM* getPickingLayerRAM() const;

protected:
    virtual void update(bool editable);

private:
    std::vector<LayerRAM*> colorLayersRAM_;
    LayerRAM* depthLayerRAM_;
    LayerRAM* pickingLayerRAM_;
};

} // namespace

#endif // IVW_IMAGERAM_H
