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
 * Main file authors: Peter Steneteg
 *
 *********************************************************************************/

#include "textureutils.h"
#include "canvasgl.h"
#include <modules/opengl/volume/volumegl.h>
#include <modules/opengl/image/imagegl.h>
#include <modules/opengl/image/layergl.h>
#include <modules/opengl/glwrap/bufferobjectarray.h>

namespace inviwo {

namespace utilgl {
void activateTarget(ImageOutport& outport) {
    Image* outImage = outport.getData();
    ImageGL* outImageGL = outImage->getEditableRepresentation<ImageGL>();
    outImageGL->activateBuffer();
}

void deactivateCurrentTarget() { FrameBufferObject::deactivateFBO(); }

void clearCurrentTarget() { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

void activateAndClearTarget(ImageOutport& outport) {
    activateTarget(outport);
    clearCurrentTarget();
}

void activateAndClearTargets(PortGroup& portGroup) {
    portGroup.activate();
    clearCurrentTarget();
}

void updateAndActivateTarget(ImageOutport& outport, ImageInport& inport) {
    Image* outImage = outport.getData();
    ImageGL* outImageGL = outImage->getEditableRepresentation<ImageGL>();
    outImageGL->updateFrom(inport.getData()->getRepresentation<ImageGL>());
    outImageGL->activateBuffer();
}

void bindTextures(const Image* image, bool color, bool depth, bool picking, GLenum colorTexUnit,
                    GLenum depthTexUnit, GLenum pickingTexUnit) {
    const ImageGL* imageGL = image->getRepresentation<ImageGL>();
    if (color) {
        const LayerGL* layer = imageGL->getColorLayerGL();
        if (layer) {
            layer->bindTexture(colorTexUnit);
        }
    }
    if (depth) {
        const LayerGL* layer = imageGL->getDepthLayerGL();
        if (layer) {
            layer->bindTexture(depthTexUnit);
        }
    }
    if (picking) {
        const LayerGL* layer = imageGL->getPickingLayerGL();
        if (layer) {
            layer->bindTexture(pickingTexUnit);
        }
    }
}

void bindColorTexture(const ImageInport& inport, GLenum texUnit) {
    bindTextures(inport.getData(), true, false, false, texUnit, 0, 0);
}

void bindColorTexture(const ImageOutport& outport, GLenum texUnit) {
    bindTextures(outport.getConstData(), true, false, false, texUnit, 0, 0);
}

void bindDepthTexture(const ImageInport& inport, GLenum texUnit) {
    bindTextures(inport.getData(), false, true, false, 0, texUnit, 0);
}

void bindDepthTexture(const ImageOutport& outport, GLenum texUnit) {
    bindTextures(outport.getConstData(), false, true, false, 0, texUnit, 0);
}

void bindPickingTexture(const ImageInport& inport, GLenum texUnit) {
    bindTextures(inport.getData(), false, false, true, 0, 0, texUnit);
}

void bindPickingTexture(const ImageOutport& outport, GLenum texUnit) {
    bindTextures(outport.getConstData(), false, false, true, 0, 0, texUnit);
}

void bindTextures(const Image* image, GLenum colorTexUnit, GLenum depthTexUnit) {
    bindTextures(image, true, true, false, colorTexUnit, depthTexUnit, 0);
}

void bindTextures(const ImageInport& inport, GLenum colorTexUnit, GLenum depthTexUnit) {
    bindTextures(inport.getData(), true, true, false, colorTexUnit, depthTexUnit, 0);
}

void bindTextures(const ImageOutport& outport, GLenum colorTexUnit, GLenum depthTexUnit) {
    bindTextures(outport.getConstData(), true, true, false, colorTexUnit, depthTexUnit, 0);
}

void bindTextures(const Image* image, GLenum colorTexUnit, GLenum depthTexUnit,
                    GLenum pickingTexUnit) {
    bindTextures(image, true, true, true, colorTexUnit, depthTexUnit, pickingTexUnit);
}

void bindTextures(const ImageInport& inport, GLenum colorTexUnit, GLenum depthTexUnit,
                    GLenum pickingTexUnit) {
    bindTextures(inport.getData(), true, true, true, colorTexUnit, depthTexUnit, pickingTexUnit);
}

void bindTextures(const ImageOutport& outport, GLenum colorTexUnit, GLenum depthTexUnit,
                    GLenum pickingTexUnit) {
    bindTextures(outport.getConstData(), true, true, true, colorTexUnit, depthTexUnit,
                   pickingTexUnit);
}

void bindColorTexture(const ImageInport& inport, const TextureUnit& texUnit) {
    bindTextures(inport.getData(), true, false, false, texUnit.getEnum(), 0, 0);
}

void bindColorTexture(const ImageOutport& outport, const TextureUnit& texUnit) {
    bindTextures(outport.getConstData(), true, false, false, texUnit.getEnum(), 0, 0);
}

void bindDepthTexture(const ImageInport& inport, const TextureUnit& texUnit) {
    bindTextures(inport.getData(), false, true, false, 0, texUnit.getEnum(), 0);
}

void bindDepthTexture(const ImageOutport& outport, const TextureUnit& texUnit) {
    bindTextures(outport.getConstData(), false, true, false, 0, texUnit.getEnum(), 0);
}

void bindPickingTexture(const ImageInport& inport, const TextureUnit& texUnit) {
    bindTextures(inport.getData(), false, false, true, 0, 0, texUnit.getEnum());
}

void bindPickingTexture(const ImageOutport& outport, const TextureUnit& texUnit) {
    bindTextures(outport.getConstData(), false, false, true, 0, 0, texUnit.getEnum());
}

void bindTextures(const Image* image, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit) {
    bindTextures(image, true, true, false, colorTexUnit.getEnum(), depthTexUnit.getEnum(), 0);
}

void bindTextures(const ImageInport& inport, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit) {
    bindTextures(inport.getData(), true, true, false, colorTexUnit.getEnum(),
                   depthTexUnit.getEnum(), 0);
}

void bindTextures(const ImageOutport& outport, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit) {
    bindTextures(outport.getConstData(), true, true, false, colorTexUnit.getEnum(),
                   depthTexUnit.getEnum(), 0);
}

void bindTextures(const Image* image, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit, const TextureUnit& pickingTexUnit) {
    bindTextures(image, true, true, true, colorTexUnit.getEnum(), depthTexUnit.getEnum(),
                   pickingTexUnit.getEnum());
}

void bindTextures(const ImageInport& inport, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit, const TextureUnit& pickingTexUnit) {
    bindTextures(inport.getData(), true, true, true, colorTexUnit.getEnum(),
                   depthTexUnit.getEnum(), pickingTexUnit.getEnum());
}

void bindTextures(const ImageOutport& outport, const TextureUnit& colorTexUnit,
                    const TextureUnit& depthTexUnit, const TextureUnit& pickingTexUnit) {
    bindTextures(outport.getConstData(), true, true, true, colorTexUnit.getEnum(),
                   depthTexUnit.getEnum(), pickingTexUnit.getEnum());
}

void unbindTextures(const Image* image, bool color, bool depth, bool picking) {
    const ImageGL* imageGL = image->getRepresentation<ImageGL>();
    if (color) {
        const LayerGL* layer = imageGL->getColorLayerGL();
        if (layer) {
            layer->unbindTexture();
        }
    }
    if (depth) {
        const LayerGL* layer = imageGL->getDepthLayerGL();
        if (layer) {
            layer->unbindTexture();
        }
    }
    if (picking) {
        const LayerGL* layer = imageGL->getPickingLayerGL();
        if (layer) {
            layer->unbindTexture();
        }
    }
}

void unbindColorTexture(const ImageInport& inport) {
    unbindTextures(inport.getData(), true, false, false);
}

void unbindColorTexture(const ImageOutport& outport) {
    unbindTextures(outport.getConstData(), true, false, false);
}

void unbindDepthTexture(const ImageInport& inport) {
    unbindTextures(inport.getData(), false, true, false);
}

void unbindDepthTexture(const ImageOutport& outport) {
    unbindTextures(outport.getConstData(), false, true, false);
}

void unbindPickingTexture(const ImageInport& inport) {
    unbindTextures(inport.getData(), false, false, true);
}

void unbindPickingTexture(const ImageOutport& outport) {
    unbindTextures(outport.getConstData(), false, false, true);
}

void unbindTextures(const Image* image) { unbindTextures(image, true, true, true); }

void unbindTextures(const ImageInport& inport) {
    unbindTextures(inport.getData(), true, true, true);
}

void unbindTextures(const ImageOutport& outport) {
    unbindTextures(outport.getConstData(), true, true, true);
}

void setShaderUniforms(Shader* shader, const Image* image, const std::string samplerID) {
    vec2 dimensions = vec2(image->getDimension());
    shader->setUniform(samplerID + ".dimensions_", dimensions);
    shader->setUniform(samplerID + ".dimensionsRCP_", vec2(1.0f) / dimensions);
}

void setShaderUniforms(Shader* shader, const ImageInport& inport,
                            const std::string samplerID) {
    setShaderUniforms(shader, inport.getData(),  samplerID);
}

void setShaderUniforms(Shader* shader, const ImageOutport& outport,
                            const std::string samplerID) {
    setShaderUniforms(shader, outport.getConstData(), samplerID);
}

BufferObjectArray* enableImagePlaneRect() {
    BufferObjectArray* rectArray = new BufferObjectArray();
    CanvasGL::attachImagePlanRect(rectArray);
    glDepthFunc(GL_ALWAYS);
    rectArray->bind();
    return rectArray;
}

void disableImagePlaneRect(BufferObjectArray* rectArray) {
    rectArray->unbind();
    glDepthFunc(GL_LESS);
}

void singleDrawImagePlaneRect() {
    BufferObjectArray* rectArray = enableImagePlaneRect();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    disableImagePlaneRect(rectArray);
    delete rectArray;
}

void multiDrawImagePlaneRect(int instances) {
    BufferObjectArray* rectArray = enableImagePlaneRect();
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, instances);
    disableImagePlaneRect(rectArray);
    delete rectArray;
}

void bindTexture(const TransferFunctionProperty& tfp, const TextureUnit& texUnit) {
    const Layer* tfLayer = tfp.get().getData();
    if (tfLayer) {
        const LayerGL* transferFunctionGL = tfLayer->getRepresentation<LayerGL>();
        transferFunctionGL->bindTexture(texUnit.getEnum());
    }
}

void bindTexture(const Volume* volume, const TextureUnit& texUnit) {
    const VolumeGL* volumeGL = volume->getRepresentation<VolumeGL>();
    volumeGL->bindTexture(texUnit.getEnum());
}

void bindTexture(const VolumeInport& inport, const TextureUnit& texUnit) {
    bindTexture(inport.getData(), texUnit);
}
}

}  // namespace
