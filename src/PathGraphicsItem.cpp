#include "PathGraphicsItem.h"

PathGraphicsItem::PathGraphicsItem(const std::vector<Path> &paths) : paths(paths) {
    this->setColor(Qt::black);
}

PathGraphicsItem::~PathGraphicsItem() = default;

void PathGraphicsItem::modelChanged() {
    update();
}

QRectF PathGraphicsItem::boundingRect() const {
    std::function<float(float, float)> min = [](float a, float b){
        return (a < b) ? a : b;
    };
    std::function<float(float, float)> max = [](float a, float b){
        return (a > b) ? a : b;
    };

    float xmin = INFINITY, ymin = INFINITY, xmax = -INFINITY, ymax = -INFINITY;
    for (const Path &path : paths) {
        for (const Path::Element &element : path.elements) {
            xmin = min(xmin, element.next.first);
            ymin = min(ymin, element.next.second);
            xmax = max(xmax, element.next.first);
            ymax = max(ymax, element.next.second);
        }
    }
    return {xmin, ymin, xmax - xmin, ymax - ymin};
}

void PathGraphicsItem::setColor(const QColor &c) {
    this->color = c;
}

void PathGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setPen(QPen(color, .1));

    QVector<QLineF> lines;
    std::vector<std::tuple<QRectF, int, int>> arcs;

    for (const Path &path : paths) {
        const Path::Point *previous = nullptr;
        for (const Path::Element &element : path.elements) {
            const Path::Point *next = &element.next;

            if (previous != nullptr) {
                if (element.is_circular) {
                    const Path::Point &center = element.helper;
                    double x = next->first - center.first;
                    double y = next->second - center.second;
                    double r = sqrt(x*x + y*y);
                    QRectF rect(x-r, y-r, 2*r, 2*r);

                    int startAngle = 16 * (int)(atan2(y, x) * 180 / 3.14159265);
                    int spanAngle = (16 * (int)(atan2(previous->first - center.first, previous->second - center.second) * 180 / 3.14159265)) - startAngle;

                    arcs.emplace_back(rect, startAngle, spanAngle);
                } else {
                    lines.push_back({previous->first, previous->second, next->first, next->second});
                }
            }

            previous = next;
        }
    }

    painter->drawLines(lines);
    for (const std::tuple<QRectF, int, int> &arc : arcs) {
        const QRectF &rectangle = std::get<0>(arc);
        const int startAngle = std::get<1>(arc);
        const int spanAngle = std::get<2>(arc);
        painter->drawArc(rectangle, startAngle, spanAngle);
    }
}
