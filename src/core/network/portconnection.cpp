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
 * Main file authors: Sathish Kottravel, Erik Sund�n
 *
 *********************************************************************************/

#include <inviwo/core/network/portconnection.h>

namespace inviwo {

PortConnection::PortConnection()
    : inport_(0),
      outport_(0) {}

PortConnection::PortConnection(Outport* outport, Inport* inport)
    : inport_(inport),
      outport_(outport) {}

PortConnection::~PortConnection() {}

void PortConnection::serialize(IvwSerializer& s) const {
    s.serialize("OutPort", *getOutport());
    s.serialize("InPort", *getInport());
}

void PortConnection::deserialize(IvwDeserializer& d) {
    Outport outport("");
    d.deserialize("OutPort", outport);
    Processor* outPortProcessor = outport.getProcessor();

    if (outPortProcessor)
        outport_ = outPortProcessor->getOutport(outport.getIdentifier());
    else
        LogWarn("Could not deserialize " << outport.getIdentifier());

    Inport inport("");
    d.deserialize("InPort", inport);
    Processor* inPortProcessor = inport.getProcessor();

    if (inPortProcessor)
        inport_ = inPortProcessor->getInport(inport.getIdentifier());
    else
        LogWarn("Could not deserialize " << inport.getIdentifier());
}

} // namespace
