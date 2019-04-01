#ifndef MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <CGAL/Segment_2.h>
#include <CGAL/Circular_arc_point_2.h>
#include <CGAL/Circular_arc_2.h>

#include "cgal_types.h"

class FreeSpaceGraphicsItem : public CGAL::Qt::GraphicsItem {
private:
    static constexpr int POLYLINE_APPROXIMATION = 100;

public:
    inline FreeSpaceGraphicsItem(const std::vector<std::pair<Polygon, General_polygon_set>>& F) : F(F)
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

        for (const auto& f : F) {
            CGAL::Bbox_2 bbox = f.first.bbox();

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
        CGAL::Qt::PainterOstream<Kernel> painterOstream(painter, boundingRect());

        painter->setPen(this->curvesPen());
        for (const auto& f : F) {
            std::vector<Polygon_with_holes> polygons_with_holes;
            f.second.polygons_with_holes(std::back_inserter(polygons_with_holes));

            for (const Polygon_with_holes& polygon_with_holes : polygons_with_holes) {
                paintPolygon(painterOstream, polygon_with_holes.outer_boundary());
                for (auto iter = polygon_with_holes.holes_begin(); iter != polygon_with_holes.holes_end(); ++iter) {
                    paintPolygon(painterOstream, *iter);
                }
            }
        }
    }

protected:
    const std::vector<std::pair<Polygon, General_polygon_set>>& F;
    QPen cPen;

    inline void paintPolygon(CGAL::Qt::PainterOstream<Kernel>& painterOstream, const Polygon& f)
    {
        for (auto iter = f.curves_begin(); iter != f.curves_end(); ++iter) {
            const Polygon::X_monotone_curve_2& curve = *iter;

            if (curve.is_linear()) {
                painterOstream << curve.supporting_line();
            } else if (curve.is_circular()) {
                painterOstream << curve.supporting_circle();
            }
        }
    }
};


#endif //MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
