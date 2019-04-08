#ifndef MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>
#include <qt5/QtXml/QtXml>

#include "cgal_types.h"

class ConfigurationGraphicsItem : public CGAL::Qt::GraphicsItem {

public:
    inline ConfigurationGraphicsItem(const std::vector<Point>& configs) : configs(configs)
    {
        this->setDiscPen(QPen(Qt::red, 1.));
        this->setDiscBrush(QBrush(Qt::red));
    }

    inline void modelChanged()
    {
        update();
    }

    inline QRectF boundingRect() const
    {
        CGAL::Bbox_2 bbox = CGAL::bbox_2(configs.begin(), configs.end());
        return {bbox.xmin(), bbox.ymin(), bbox.xmax() - bbox.xmin(), bbox.ymax() - bbox.ymin()};
    }

    void setDiscPen(const QPen& pen)
    {
        this->dPen = pen;
    }

    const QPen& discPen() const
    {
        return dPen;
    }

    void setDiscBrush(const QBrush& brush)
    {
        this->dBrush = brush;
    }

    const QBrush& discBrush() const
    {
        return dBrush;
    }

    inline void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

        painter->setPen(this->discPen());
        painter->setBrush(this->discBrush());
        for (const Point& config : configs) {
            Circle unit_disc(config, 1.);
            painterOstream << unit_disc;
        }
    }

protected:
    const std::vector<Point>& configs;
    QPen dPen;
    QBrush dBrush;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONGRAPHICSITEM_H
