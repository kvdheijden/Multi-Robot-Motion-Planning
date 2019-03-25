#ifndef MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include "cgal_types.h"

class FreeSpaceGraphicsItem : public CGAL::Qt::GraphicsItem {
private:
    static constexpr int POLYLINE_APPROXIMATION = 100;

public:
    inline FreeSpaceGraphicsItem(const std::vector<Inset_polygon_with_holes>& F) : F(F)
    {
        this->setCurvesPen(QPen(Qt::blue, .1));
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

        for (const Inset_polygon_with_holes& f : F) {
            CGAL::Bbox_2 bbox = f.outer_boundary().bbox();

            qreal x = bbox.xmin();
            qreal y = bbox.ymin();
            if (x < xmin) {
                xmin = x;
            }
            if (y < ymin) {
                ymin = y;
            }

            x = bbox.xmax();
            y = bbox.ymax();
            if (x > xmax) {
                xmax = x;
            }
            if (y > ymax) {
                ymax = y;
            }
        }

        return {xmin, ymin, xmax - xmin, ymax - ymin};
    }

    void setCurvesPen(const QPen& pen)
    {
        this->cPen = pen;
    }

    const QPen& curvesPen() const
    {
        return cPen;
    }

    inline void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

        painter->setPen(this->curvesPen());
        for (const Inset_polygon_with_holes& f : F) {
            paint_conic_polygon(painterOstream, f.outer_boundary());
            for (auto iter = f.holes_begin(); iter != f.holes_end(); ++iter) {
                paint_conic_polygon(painterOstream, *iter);
            }
        }
    }

protected:
    const std::vector<Inset_polygon_with_holes>& F;
    QPen cPen;

    void paint_conic_polygon(CGAL::Qt::PainterOstream<Kernel>& painterOstream, const Inset_polygon& p)
    {
        for (auto iter = p.curves_begin(); iter != p.curves_end(); ++iter) {
            std::list<std::pair<double, double>> polyline;
            iter->polyline_approximation(POLYLINE_APPROXIMATION, std::back_inserter(polyline));
            for (int i = 1; i < polyline.size(); i++) {
                std::pair<double, double> first = polyline.front();
                polyline.pop_front();
                std::pair<double, double> second = polyline.front();

                Point source(first.first, first.second);
                Point target(second.first, second.second);

                Segment segment(source, target);

                painterOstream << segment;
            }
        }
    }
};


#endif //MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
