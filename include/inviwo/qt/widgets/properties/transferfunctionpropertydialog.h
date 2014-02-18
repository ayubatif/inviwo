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
 * Main file authors: Viktor Axelsson, Timo Ropinski
 *
 *********************************************************************************/

/** \ Widget for containing the TransferFunctionEditor QGraphicsScene
 *       Widget that contains the TransferFunctionEditor and the painted representation
 */

#ifndef IVW_TRANSFERFUNCTIONPROPERTYDIALOG_H
#define IVW_TRANSFERFUNCTIONPROPERTYDIALOG_H

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/qt/widgets/properties/transferfunctioneditor.h>
#include <inviwo/qt/widgets/properties/transferfunctioneditorview.h>
#include <inviwo/qt/widgets/colorwheel.h>
#include <inviwo/qt/widgets/rangesliderqt.h>
#include <inviwo/qt/widgets/properties/intminmaxpropertywidgetqt.h>
#include <inviwo/qt/widgets/properties/optionpropertywidgetqt.h>
#include <inviwo/core/properties/propertywidget.h>

#include <QCheckBox>
#include <QColorDialog>
#include <QGradientStops>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPainter>
#include <QPixmap>
#include <QSpinBox>
#include <QWheelEvent>
#include <inviwo/core/util/observer.h>

namespace inviwo {

class TransferFunctionPropertyWidgetQt;

class IVW_QTWIDGETS_API TransferFunctionPropertyDialog : public PropertyEditorWidgetQt {

    Q_OBJECT

public:
    TransferFunctionPropertyDialog(TransferFunctionProperty* property, QWidget* parent);
    ~TransferFunctionPropertyDialog();

    /** \Updates and draws the visual transfer function
    *
    *      Fetches the float array from the TransferFunction and draws a line for each value
    *      Redraws it fully every time, to be optimized if it is allowed to stay
    */
    void updateFromProperty();

    QLinearGradient* getTFGradient() { return gradient_; }

    virtual void notify();

public slots:
    void setPointColor();
    void setPointColorDialog();
    void updateColorWheel();
    void showColorDialog();
    void switchInterpolationType(int interpolationType);
    void changeMask(int maskMin, int maskMax);

    void importTransferFunction();
    void exportTransferFunction();
    void showHistogram(bool);
    void dockLocationChanged(Qt::DockWidgetArea dockArea);
private:
    int arrayWidth_;
    int arrayHeight_;

    TransferFunctionProperty* tfProperty_;     ///< Pointer to property, for get and invalidation in the widget
    TransferFunctionEditor* tfEditor_;         ///< TransferFunctionEditor inherited from QGraphicsScene
    TransferFunctionEditorView* tfEditorView_; ///< View that contains the editor
    QPushButton* btnClearTF_;
    QPushButton* btnImportTF_;
    QPushButton* btnExportTF_;
    QComboBox* cmbInterpolation_;
    QCheckBox* chkShowHistogram_;

    QLabel* tfPreview_; ///< View that contains the scene for the painted transfer function
    QPixmap* tfPixmap_;

    QLinearGradient* gradient_;

    QColorDialog* colorDialog_;
    ColorWheel* colorWheel_;

    RangeSliderQt* zoomVSlider_;
    RangeSliderQt* zoomHSlider_;
    RangeSliderQt* maskSlider_;

    bool colorChange_;
    void generateWidget();
    void setPointColor(QColor color);
    void updateTFPreview();
protected:
    virtual void resizeEvent(QResizeEvent*);
    virtual void closeEvent(QCloseEvent*);
    virtual void showEvent(QShowEvent*);
    virtual void moveEvent(QMoveEvent*);
};

} // namespace

#endif // IVW_TRANSFERFUNCTIONPROPERTYDIALOG_H