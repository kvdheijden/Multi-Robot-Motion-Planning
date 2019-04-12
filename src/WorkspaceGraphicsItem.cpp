#include "WorkspaceGraphicsItem.h"

WorkspaceGraphicsItem::WorkspaceGraphicsItem(const Workspace &W) : W(W) {
    this->setColor(Qt::black);
}

WorkspaceGraphicsItem::~WorkspaceGraphicsItem() = default;

void WorkspaceGraphicsItem::modelChanged() {
    update();
}

QRectF WorkspaceGraphicsItem::boundingRect() const {
    CGAL::Bbox_2 bbox = W.bbox();
    return {bbox.xmin(), bbox.ymin(), bbox.xmax() - bbox.xmin(), bbox.ymax() - bbox.ymin()};
}

void WorkspaceGraphicsItem::setColor(const QColor &c) {
    this->color = c;
}

void WorkspaceGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

    painter->setPen(QPen(color, .1));
    for (auto iter = W.vertices_begin(); iter != W.vertices_end(); ++iter) {
        painterOstream << *iter;
    }
    for (auto iter = W.edges_begin(); iter != W.edges_end(); ++iter) {
        painterOstream << *iter;
    }
}