#ifndef MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <CGAL/Segment_2.h>
#include <CGAL/Circular_arc_point_2.h>
#include <CGAL/Circular_arc_2.h>

#include "cgal_types.h"

#include <CGAL/CORE/Expr.h>

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
        CGAL::Qt::PainterOstream<CircularKernel> painterOstreamCK(painter, boundingRect());
        CGAL::Qt::PainterOstream<Kernel> painterOstreamK(painter, boundingRect());

        painter->setPen(this->curvesPen());
        for (const auto& f : F) {
            for (auto iter = f.first.curves_begin(); iter != f.first.curves_end(); ++iter) {
                const Polygon::X_monotone_curve_2& curve = *iter;
                const Polygon::Point_2& source = curve.source();
                CORE::Expr sourceX = source.x().a0() + source.x().a1() * sqrt(source.x().root());
                CORE::Expr sourceY = source.y().a0() + source.y().a1() * sqrt(source.y().root());

                const Polygon::Point_2& target = curve.target();
                CORE::Expr targetX = target.x().a0() + target.x().a1() * sqrt(target.x().root());
                CORE::Expr targetY = target.y().a0() + target.y().a1() * sqrt(target.y().root());

                if (curve.is_linear()) {
                    Point start(sourceX, sourceY);
                    Point end(targetX, targetY);
                    painterOstreamK << Segment(end, start);
                } else if (curve.is_circular()) {
                    const Circle& supporting_circle = curve.supporting_circle();

                    CircularKernel::FT squared_radius(supporting_circle.squared_radius());
                    CircularKernel::Point_2 center(supporting_circle.center().x(), supporting_circle.center().y());

                    CircularKernel::Circle_2 circle(center, squared_radius);
                    CircularKernel::Point_2 start(sourceX, sourceY);
                    CircularKernel::Point_2 end(targetX, targetY);

                    CircularKernel::Circular_arc_2 carc(circle, end, start);
                    painterOstreamCK << carc;
                }
            }
        }
    }

protected:
    const std::vector<std::pair<Polygon, General_polygon_set>>& F;
    QPen cPen;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
