#ifndef MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSETGRAPHICSITEM_H
#define MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSETGRAPHICSITEM_H

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>
#include <qt5/QtXml/QtXml>

#include "ConfigurationSet.h"

class ConfigurationSetGraphicsItem : public CGAL::Qt::GraphicsItem {

private:
    const ConfigurationSet &configs;

    QColor sourceColor;
    QColor targetColor;
    QColor collisionDiscColor;

public:
    explicit ConfigurationSetGraphicsItem(const ConfigurationSet &configs);

    ~ConfigurationSetGraphicsItem() override;

    void modelChanged() override;

    QRectF boundingRect() const override;

    void setSourceColor(const QColor &color);

    void setTargetColor(const QColor &color);

    void setCollisionDiscColor(const QColor &color);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSETGRAPHICSITEM_H
