#ifndef MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include "cgal_types.h"

class WorkspaceGraphicsItem : public CGAL::Qt::GraphicsItem {

public:
    inline WorkspaceGraphicsItem(const Input_polygon& pgn) : pgn(pgn)
    {
        this->setVerticesPen(QPen(Qt::black, 1.));
        this->setEdgesPen(QPen(Qt::black, 1.));
    }

    inline void modelChanged()
    {
        update();
    }

    inline QRectF boundingRect() const
    {
        CGAL::Bbox_2 bbox = pgn.bbox();
        return {bbox.xmin(), bbox.ymin(), bbox.xmax() - bbox.xmin(), bbox.ymax() - bbox.ymin()};
    }

    void setVerticesPen(const QPen& pen)
    {
        this->vPen = pen;
    }

    void setEdgesPen(const QPen& pen)
    {
        this->ePen = pen;
    }

    const QPen& verticesPen() const
    {
        return vPen;
    }

    const QPen& edgesPen() const
    {
        return ePen;
    }

    inline void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

        painter->setPen(this->verticesPen());
        for (auto iter = pgn.vertices_begin(); iter != pgn.vertices_end(); ++iter) {
            painterOstream << *iter;
        }

        painter->setPen(this->edgesPen());
        for (auto iter = pgn.edges_begin(); iter != pgn.edges_end(); ++iter) {
            painterOstream << *iter;
        }
    }

protected:
    const Input_polygon& pgn;

    QPen vPen, ePen;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_WORKSPACEGRAPHICSITEM_H
