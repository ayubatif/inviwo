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
 * Main file author: Sathish Kottravel
 *
 *********************************************************************************/

#include <inviwo/core/properties/propertywidget.h>
#include <inviwo/core/properties/property.h>

namespace inviwo {

PropertyWidget::PropertyWidget()
    : property_(0), propertyEditor_(0) {
}

PropertyWidget::PropertyWidget(Property* property)
    : property_(property), propertyEditor_(0) {
}

PropertyWidget::~PropertyWidget() {
}

Property* PropertyWidget::getProperty() {
    return property_;
}

void PropertyWidget::setProperty(Property* property) {
    property_ = property;
}

void PropertyWidget::setEditorWidget(PropertyEditorWidget* propertyEditorWidget) {
    propertyEditor_ = propertyEditorWidget;
}

PropertyEditorWidget* PropertyWidget::getEditorWidget() const {
    return propertyEditor_;
}

bool PropertyWidget::hasEditorWidget() const {
    return (propertyEditor_ != 0);
}

void PropertyWidget::initializeEditorWidgetsMetaData() {}

//////////////////////////////////////////////////////////////////////////

//Additional widgets owned by property

PropertyEditorWidget::PropertyEditorWidget()
    : metaData_(0) {
}

PropertyEditorWidget::~PropertyEditorWidget() {
}

void PropertyEditorWidget::initialize(Property* property) {
    metaData_ = dynamic_cast<PropertyEditorWidgetMetaData*>(property->getMetaData("PropertyEditorWidgetMetaData"));
}

void PropertyEditorWidget::deinitialize() {
    //Not yet tested
    //metaData_ = NULL;
}

void PropertyEditorWidget::setEditorVisibility(bool visible) {
    metaData_->setVisibile(visible);
}

void PropertyEditorWidget::showEditor() {
    metaData_->setVisibile(true);
}

void PropertyEditorWidget::hideEditor() {
    metaData_->setVisibile(false);
}

void PropertyEditorWidget::setEditorDimension(ivec2 dimension) {
    metaData_->setDimension(dimension);
}

void PropertyEditorWidget::moveEditor(ivec2 pos) {
    metaData_->setWidgetPosition(pos);
}

void PropertyEditorWidget::setDockStatus(PropertyEditorWidgetDockStatus dockStatus) {
    metaData_->setDockStatus(dockStatus);
}
bool PropertyEditorWidget::getEditorVisibilityMetaData() {
    return metaData_->isVisible();
}

ivec2 PropertyEditorWidget::getEditorPositionMetaData() {
    return metaData_->getWidgetPosition();
}

ivec2 PropertyEditorWidget::getEditorDimensionMetaData() {
    return metaData_->getDimension();
}

PropertyEditorWidgetDockStatus PropertyEditorWidget::getEditorDockStatus() {
    return metaData_->getDocStatus();
}

} // namespace
