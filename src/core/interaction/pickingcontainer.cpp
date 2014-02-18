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
 * Main file author: Erik Sund�n
 *
 *********************************************************************************/

#include <inviwo/core/interaction/pickingcontainer.h>
#include <inviwo/core/interaction/pickingmanager.h>
#include <inviwo/core/datastructures/image/image.h>
#include <inviwo/core/datastructures/image/layerram.h>

namespace inviwo {

PickingContainer::PickingContainer()
    : src_(NULL)
    , currentPickObj_(NULL)
    , prevCoord_(uvec2(0,0))
    , selected_(false) {};

PickingContainer::~PickingContainer() {};

bool PickingContainer::isPickableSelected() { return selected_; }

bool PickingContainer::performPick(const uvec2& coord) {
    prevCoord_ = coord;

    if (src_) {
        const Layer* pickingLayer = src_->getPickingLayer();

        if (pickingLayer) {
            const LayerRAM* pickingLayerRAM = pickingLayer->getRepresentation<LayerRAM>();
            vec4 value = pickingLayerRAM->getValueAsVec4Float(coord);
            vec3 pickedColor = (value.a > 0.f ? value.rgb() : vec3(0.f));
            DataVec3UINT8::type pickedColorUINT8;
            DataVec3UINT8::get()->vec3ToValue(pickedColor*255.f, &pickedColorUINT8);
            currentPickObj_ = PickingManager::instance()->getPickingObjectFromColor(pickedColorUINT8);

            if (currentPickObj_) {
                setPickableSelected(true);
                currentPickObj_->setPickingPosition(normalizedCoordinates(coord));

                if (currentPickObj_->readDepth()) {
                    const Layer* depthLayer = src_->getDepthLayer();

                    if (pickingLayer) {
                        const LayerRAM* depthLayerRAM = depthLayer->getRepresentation<LayerRAM>();
                        float depth = depthLayerRAM->getValueAsSingleFloat(coord);
                        currentPickObj_->setPickingDepth(depth);
                    }
                }

                currentPickObj_->setPickingMove(vec2(0.f,0.f));
                currentPickObj_->picked();
                return true;
            }
        }
    }

    setPickableSelected(false);
    return false;
}

void PickingContainer::movePicked(const uvec2& coord) {
    currentPickObj_->setPickingMove(pixelMoveVector(prevCoord_, coord));
    prevCoord_ = coord;
    currentPickObj_->picked();
}

void PickingContainer::setPickableSelected(bool selected) {
    selected_ = selected;
}

void PickingContainer::setPickingSource(const Image* src) {
    src_ = src;
}

vec2 PickingContainer::pixelMoveVector(const uvec2& previous, const uvec2& current) {
    return vec2((static_cast<float>(current.x)-static_cast<float>(previous.x))/static_cast<float>(src_->getDimension().x),
                (static_cast<float>(current.y)-static_cast<float>(previous.y))/static_cast<float>(src_->getDimension().y));
}

vec2 PickingContainer::normalizedCoordinates(const uvec2& coord) {
    return vec2(static_cast<float>(coord.x)/static_cast<float>(src_->getDimension().x),
                static_cast<float>(coord.y)/static_cast<float>(src_->getDimension().y));
}

} // namespace