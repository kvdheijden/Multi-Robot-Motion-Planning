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
    }

    inline void modelChanged()
    {
        update();
    }

    inline QRectF boundingRect() const
    {
        qreal xmin = INFINITY;
        qreal xmax = -INFINITY;
        qreal ymin = INFINITY;
        qreal ymax = -INFINITY;

        for (const Point& config : configs) {
            qreal x = config.x().doubleValue();
            qreal y = config.y().doubleValue();
            if (x < xmin) {
                xmin = x;
            }
            if (x > xmax) {
                xmax = x;
            }
            if (y < ymin) {
                ymin = y;
            }
            if (y > ymax) {
                ymax = y;
            }
        }

        return {xmin, ymin, xmax - xmin, ymax - ymin};
    }

    void setDiscPen(const QPen& pen)
    {
        this->dPen = pen;
    }

    const QPen& discPen() const
    {
        return dPen;
    }

    inline void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

        painter->setPen(this->discPen());
        for (const Point& config : configs) {
            Circle unit_disc(config, 1.);
            painterOstream << unit_disc;
        }
    }

protected:
    const std::vector<Point>& configs;
    QPen dPen;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONGRAPHICSITEM_H
