#include "FreeSpaceGraphicsItem.h"

FreeSpaceGraphicsItem::FreeSpaceGraphicsItem(const FreeSpace &F) : F(F) {
    this->setColor(Qt::blue);
}

FreeSpaceGraphicsItem::~FreeSpaceGraphicsItem() = default;

void FreeSpaceGraphicsItem::modelChanged() {
    update();
}

QRectF FreeSpaceGraphicsItem::boundingRect() const {
    qreal xmin = INFINITY;
    qreal xmax = -INFINITY;
    qreal ymin = INFINITY;
    qreal ymax = -INFINITY;

    for (const Polygon &f : F) {
        CGAL::Bbox_2 bbox = f.bbox();

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

void FreeSpaceGraphicsItem::setColor(const QColor &c) {
    this->color = c;
}

void FreeSpaceGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    CGAL::Qt::PainterOstream<CircularKernel> painterOstreamCK(painter, boundingRect());
    CGAL::Qt::PainterOstream<Kernel> painterOstreamK(painter, boundingRect());

    painter->setPen(QPen(color, .1));

    for (const Polygon &f : F) {
        for (auto iter = f.curves_begin(); iter != f.curves_end(); ++iter) {
            const Polygon::X_monotone_curve_2 &curve = *iter;
            const Polygon::Point_2 &source = curve.source();
            CORE::Expr sourceX = source.x().a0() + source.x().a1() * CGAL::sqrt(source.x().root());
            CORE::Expr sourceY = source.y().a0() + source.y().a1() * CGAL::sqrt(source.y().root());

            const Polygon::Point_2 &target = curve.target();
            CORE::Expr targetX = target.x().a0() + target.x().a1() * CGAL::sqrt(target.x().root());
            CORE::Expr targetY = target.y().a0() + target.y().a1() * CGAL::sqrt(target.y().root());

            if (curve.is_linear()) {
                Point start(sourceX, sourceY);
                Point end(targetX, targetY);
                painterOstreamK << Segment(end, start);
            } else if (curve.is_circular()) {
                const Circle &supporting_circle = curve.supporting_circle();

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