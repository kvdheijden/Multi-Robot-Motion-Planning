#ifndef MULTI_ROBOT_MOTION_PLANNING_PATHGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_PATHGRAPHICSITEM_H

#include <vector>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include "Path.h"

class PathGraphicsItem : public CGAL::Qt::GraphicsItem {
private:
    const std::vector<Path> &paths;
    QColor color;

public:
    explicit PathGraphicsItem(const std::vector<Path> &paths);

    ~PathGraphicsItem() override;

    void modelChanged() override;

    QRectF boundingRect() const override;

    void setColor(const QColor &c);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_PATHGRAPHICSITEM_H
