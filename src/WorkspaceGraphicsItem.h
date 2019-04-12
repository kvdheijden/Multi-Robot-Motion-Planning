#ifndef MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include "cgal_types.h"

class WorkspaceGraphicsItem : public CGAL::Qt::GraphicsItem {

private:
    const Workspace &W;
    QColor color;

public:
    explicit WorkspaceGraphicsItem(const Workspace &W);

    ~WorkspaceGraphicsItem() override;

    void modelChanged() override;

    QRectF boundingRect() const override;

    void setColor(const QColor &c);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H
