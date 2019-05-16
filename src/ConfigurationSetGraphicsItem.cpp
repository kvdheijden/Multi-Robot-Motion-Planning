#include "ConfigurationSetGraphicsItem.h"

ConfigurationSetGraphicsItem::ConfigurationSetGraphicsItem(const ConfigurationSet &configs) : configs(configs) {
    this->setSourceColor(Qt::green);
    this->setTargetColor(Qt::magenta);
    this->setCollisionDiscColor(Qt::red);
}

ConfigurationSetGraphicsItem::~ConfigurationSetGraphicsItem() = default;

void ConfigurationSetGraphicsItem::modelChanged() {
    update();
}

QRectF ConfigurationSetGraphicsItem::boundingRect() const {
    CGAL::Bbox_2 bbox = CGAL::bbox_2(configs.begin(), configs.end());
    return {bbox.xmin(), bbox.ymin(), bbox.xmax() - bbox.xmin(), bbox.ymax() - bbox.ymin()};
}

void ConfigurationSetGraphicsItem::setSourceColor(const QColor &color) {
    this->sourceColor = color;
}

void ConfigurationSetGraphicsItem::setTargetColor(const QColor &color) {
    this->targetColor = color;
}

void ConfigurationSetGraphicsItem::setCollisionDiscColor(const QColor &color) {
    this->collisionDiscColor = color;
}

void ConfigurationSetGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    CGAL::Qt::PainterOstream<Kernel> painterOstream(painter);

    Kernel::FT unit_sq_r(1.);
    Kernel::FT coll_sq_r(2. * 2.);

    QFont font = painter->font();
    font.setPixelSize(2);

    for (const Configuration &config : configs) {
        const Point &point = config.getPoint();
        // Set unit disc color
        if (config.isStart()) {
            painter->setPen(QPen(sourceColor, .1));
            painter->setBrush(QBrush(sourceColor));
        } else {
            painter->setPen(QPen(targetColor, .1));
            painter->setBrush(QBrush(targetColor));
        }

        // Paint unit disc
        painterOstream << Circle(point, unit_sq_r);

        // Set collision disc color
        painter->setPen(QPen(collisionDiscColor, .1));
        painter->setBrush(QBrush());

        // Paint collision disc
        painterOstream << Circle(point, coll_sq_r);

        // Paint name
        char label[8] = {0};
        painter->setPen(QPen(Qt::black, .1));
        sprintf(label, "%c%u", config.isStart() ? 's' : 't', config.getIndex());
        painter->setFont(font);
        painter->drawText(QPointF(point.x().doubleValue(), point.y().doubleValue()), QString(label));
    }
}