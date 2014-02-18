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

#include <modules/base/basemodule.h>
#include <modules/base/processors/cubeproxygeometry.h>
#include <modules/base/processors/diffuselightsourceprocessor.h>
#include <modules/base/processors/directionallightsourceprocessor.h>
#include <modules/base/processors/geometrysource.h>
#include <modules/base/processors/imagesource.h>
#include <modules/base/processors/imagesourceseries.h>
#include <modules/base/processors/meshclipping.h>
#include <modules/base/processors/meshcreator.h>
#include <modules/base/processors/pointlightsourceprocessor.h>
#include <modules/base/processors/volumesource.h>
#include <modules/base/processors/volumeexport.h>
#include <modules/base/processors/volumebasistransformer.h>
#include <modules/base/processors/volumeslice.h>
#include <modules/base/processors/volumesubset.h>

namespace inviwo {

BaseModule::BaseModule() : InviwoModule() {
    setIdentifier("Base");
    registerProcessor(CubeProxyGeometry);
    registerProcessor(DiffuseLightSourceProcessor);
    registerProcessor(DirectionalLightSourceProcessor);
    registerProcessor(GeometrySource);
    registerProcessor(ImageSource);
    registerProcessor(ImageSourceSeries);
    registerProcessor(MeshClipping);
    registerProcessor(MeshCreator);
    registerProcessor(PointLightSourceProcessor);
    registerProcessor(VolumeSource);
    registerProcessor(VolumeExport);
    registerProcessor(VolumeBasisTransformer);
    registerProcessor(VolumeSlice);
    registerProcessor(VolumeSubset);
}

} // namespace
