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
 * Main file author: Peter Steneteg
 *
 *********************************************************************************/

#include <inviwo/core/io/ivfvolumewriter.h>
#include <inviwo/core/util/urlparser.h>
#include <inviwo/core/datastructures/volume/volumeram.h>

namespace inviwo {

IvfVolumeWriter::IvfVolumeWriter() : DataWriterType<Volume>() {
    addExtension(FileExtension("ivf","Inviwo ivf file format"));
}

IvfVolumeWriter::IvfVolumeWriter(const IvfVolumeWriter& rhs) : DataWriterType<Volume>(rhs) {
}

IvfVolumeWriter& IvfVolumeWriter::operator=(const IvfVolumeWriter& that) {
    if (this != &that)
        DataWriterType<Volume>::operator=(that);

    return *this;
}

IvfVolumeWriter* IvfVolumeWriter::clone() const {
    return new IvfVolumeWriter(*this);
}

void IvfVolumeWriter::writeData(const Volume* volume, const std::string filePath) const {
    std::string rawPath = URLParser::replaceFileExtension(filePath, "raw");

    if (URLParser::fileExists(filePath) && !overwrite_)
        throw DataWriterException("Error: Output file: " + filePath + " already exists");

    if (URLParser::fileExists(rawPath) && !overwrite_)
        throw DataWriterException("Error: Output file: " + rawPath + " already exists");

    std::string fileName = URLParser::getFileNameWithoutExtension(filePath);
    const VolumeRAM* vr = volume->getRepresentation<VolumeRAM>();
    IvwSerializer s(filePath);
    s.serialize("ObjectFileName", fileName + ".raw");
    s.serialize("Format", vr->getDataFormatString());
    s.serialize("BasisAndOffset", volume->getBasisAndOffset());
    s.serialize("WorldTransform", volume->getWorldTransform());
    s.serialize("Dimension", volume->getDimension());
    volume->getMetaDataMap()->serialize(s);
    s.writeFile();
    std::fstream fout(rawPath.c_str(), std::ios::out | std::ios::binary);

    if (fout.good()) {
        fout.write((char*)vr->getData(),
                   vr->getDimension().x*vr->getDimension().x*vr->getDimension().x
                   * vr->getDataFormat()->getBytesStored());
    } else
        throw DataWriterException("Error: Could not write to raw file: " + rawPath);

    fout.close();
}

} // namespace
