#ifndef MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <CGAL/Segment_2.h>
#include <CGAL/Circular_arc_point_2.h>
#include <CGAL/Circular_arc_2.h>

#include "FreeSpace.h"

#include <CGAL/CORE/Expr.h>

class FreeSpaceGraphicsItem : public CGAL::Qt::GraphicsItem {
private:
    const FreeSpace &F;
    QColor color;

public:
    explicit FreeSpaceGraphicsItem(const FreeSpace &F);
    ~FreeSpaceGraphicsItem() override;

    void modelChanged() override;

    QRectF boundingRect() const override;

    void setColor(const QColor &c);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_FREESPACEGRAPHICSITEM_H
