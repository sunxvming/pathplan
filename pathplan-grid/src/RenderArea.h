
#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QBrush>
#include <QWidget>
#include "globalstruct.h"
#include "PathPlan.h"

class RenderArea : public QWidget
{
    Q_OBJECT

public:
    RenderArea(QWidget *parent = 0);

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

    void setMap(QHash<int, tagChartPointInfo> chartpoint, QHash<int, tagChartLineInfo> chartline);
    void setPath(QVector<QPointF> path) { _path = path; }
    void setOptPathCons(QVector<QPointF> path) { _optPathCons = path; }
    void setOptPathRad(QVector<QPointF> path) { _optPathRad = path; }
    QPointF wp2sp(Pos p);
    QPointF wp2sp(QPointF p);
    QPointF wp2sp(double x, double y);
    QPointF sp2wp(QPointF p);
    QPointF sp2wp(double x, double y);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    void getMapInfo(const QVector<QPointF> &allPoints, double &centerLon, double &centerLat, double &centerx, double &centery, double &scale);
    QVector<QPointF> _allPoints;
    QVector<QPointF> _path;
    QVector<QPointF> _optPathCons;
    QVector<QPointF> _optPathRad;
    QHash<int, tagChartPointInfo> _chartpoint;
    QHash<int, tagChartLineInfo> _chartline;
    QPointF _start;
    QPointF _end;
    int _clicksatus = 0;
    std::vector<std::vector<Grid>> _gridMap;
    std::vector<Pos> _obs;
    bool _showGrid;
};

#endif // RENDERAREA_H
