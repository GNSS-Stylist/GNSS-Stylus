/*
    lidarchartview.cpp (part of GNSS-Stylus)
    Copyright (C) 2020-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "lidarchartview.h"
#include <QtCharts/QValueAxis>
#include <math.h>

LidarChartView::LidarChartView(QWidget *parent)
    : QChartView(parent)
{    
}

void LidarChartView::keyPressEvent(QKeyEvent *event)
{
    QtCharts::QValueAxis* yAxis = dynamic_cast<QtCharts::QValueAxis*>(chart()->axes(Qt::Vertical)[0]);
//    QtCharts::QValueAxis* xAxis = dynamic_cast<QtCharts::QValueAxis*>(chart()->axes(Qt::Horizontal)[0]);

    QSizeF size = chart()->size();

    switch (event->key())
    {
    case Qt::Key_Plus:
        yAxis->setMax(yAxis->max()*0.9);
//        chart()->zoomIn();
        break;
    case Qt::Key_Minus:
        yAxis->setMax(yAxis->max()/0.9);
//        chart()->zoomOut();
        break;
    case Qt::Key_Left:
        chart()->setTransformOriginPoint(size.width()/2, size.height()/2);
        chart()->setRotation(chart()->rotation() + 90);
//        chart()->scroll(-1.0, 0);
        break;
    case Qt::Key_Right:
        chart()->setTransformOriginPoint(size.width()/2, size.height()/2);
        chart()->setRotation(chart()->rotation() - 90);
//        chart()->scroll(1.0, 0);
        break;
    case Qt::Key_Up:
        chart()->scroll(0, 1.0);
        break;
    case Qt::Key_Down:
        chart()->scroll(0, -1.0);
        break;
/*        case Qt::Key_Space:
        switchChartType();
        break;
        */
    default:
        QChartView::keyPressEvent(event);
        break;
    }
}

void LidarChartView::wheelEvent(QWheelEvent* event)
{
    const double factor = 1.1;
    const double deltaMultiplier = -0.01;

    double delta = event->angleDelta().y() * deltaMultiplier;
    double zoomFactor = pow(factor, delta);

    QtCharts::QValueAxis* yAxis = dynamic_cast<QtCharts::QValueAxis*>(chart()->axes(Qt::Vertical)[0]);

    yAxis->setMax(yAxis->max() * zoomFactor);
}
