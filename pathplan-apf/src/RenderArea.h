
#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QWidget>
#include "globalstruct.h"
#include <QPointF>
#include "PathPlan.h"

class RenderArea : public QWidget
{
    Q_OBJECT

public:
    RenderArea(QWidget *parent = 0);

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

    QPointF wp2sp(Pos p);
    QPointF wp2sp(QPointF p);
    QPointF wp2sp(double x, double y);
    QPointF sp2wp(QPointF p);
    QPointF sp2wp(double x, double y);
    void setMap(const QHash<int, tagChartPointInfo> &chartpoint, const QHash<int, tagChartLineInfo> &chartline);

    void setPath(QVector<QPointF> path) { _path = path; }
    void setOptPath(QVector<QPointF> path) { _optPath = path; }

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    void getMapInfo(const QVector<QPointF> &allPoints, double &centerLon, double &centerLat, double &centerx, double &centery, double &scale);
    QVector<QPointF> _allPoints;
    QVector<QPointF> _points;
    std::vector<Pos> _obs;
    QVector<QPointF> _path;
    QVector<QPointF> _optPath;
    QVector<QPointF> _finalPath;
    QHash<int, tagChartPointInfo> _chartPoint;
    QHash<int, tagChartLineInfo> _chartLine;
    QPointF _start;
    QPointF _end;
    int _clicksatus = 0;
};

#endif // RENDERAREA_H
