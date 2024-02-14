
#include "RenderArea.h"

#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QLineEdit>
#include <QCheckBox>

#include "Widget.h"
#include "PathPlan.h"

RenderArea::RenderArea(QWidget *parent)
    : QWidget(parent)
{
}

QSize RenderArea::minimumSizeHint() const
{
    return QSize(200, 200);
}

QSize RenderArea::sizeHint() const
{
    return QSize(900, 900);
}

QPointF RenderArea::wp2sp(Pos p)
{
    return wp2sp(p.x, p.y);
}

QPointF RenderArea::wp2sp(QPointF p)
{
    return wp2sp(p.x(), p.y());
}

QPointF RenderArea::wp2sp(double x, double y)
{
    double centerLon, centerLat, centerx, centery, scale;
    getMapInfo(_allPoints, centerLon, centerLat, centerx, centery, scale);
    double screen_x = (x - centerLon) * scale + centerx;
    double screen_y = (-y + centerLat) * scale + centery;
    return QPointF(screen_x, screen_y);
}

QPointF RenderArea::sp2wp(QPointF p)
{
    return sp2wp(p.x(), p.y());
}

QPointF RenderArea::sp2wp(double x, double y)
{
    double centerLon, centerLat, centerx, centery, scale;
    getMapInfo(_allPoints, centerLon, centerLat, centerx, centery, scale);
    double lon = (x - centerx) / scale + centerLon;
    double lat = centerLat - (y - centery) / scale;

    return QPointF(lon, lat);
}

void RenderArea::getMapInfo(const QVector<QPointF> &allPoints, double &centerLon, double &centerLat, double &centerx, double &centery, double &scale)
{

    double minlon = 180;
    double maxlon = -180;
    double minlat = 90;
    double maxlat = -90;

    for (const QPointF &point : allPoints)
    {
        minlon = qMin(minlon, point.x());
        maxlon = qMax(maxlon, point.x());
        minlat = qMin(minlat, point.y());
        maxlat = qMax(maxlat, point.y());
    }

    double lonRange = maxlon - minlon;
    double latRange = maxlat - minlat;

    QPointF center = QPointF(width() / 2, height() / 2);
    centerx = center.x();
    centery = center.y();
    centerLon = (maxlon + minlon) / 2.0;
    centerLat = (maxlat + minlat) / 2.0;

    scale = qMin(width() / lonRange, height() / latRange) / 1.5;
}

void RenderArea::mousePressEvent(QMouseEvent *event)
{

    Widget *p = (Widget *)parentWidget();
    double att = p->l_att->text().toDouble();
    double rep = p->l_rep->text().toDouble();
    double step = p->l_step->text().toDouble();
    double turnAngle = p->l_turnAngle->text().toDouble();
    double emCPA = p->l_emCPA->text().toDouble();

    QPointF pos = event->pos();
    double centerLon, centerLat, centerx, centery, scale;
    getMapInfo(_allPoints, centerLon, centerLat, centerx, centery, scale);
    double lon = (pos.x() - centerx) / scale + centerLon;
    double lat = centerLat - (pos.y() - centery) / scale;
    if (_clicksatus == 0)
    {
        _start = QPointF(lon, lat);
        _clicksatus = 1;
    }
    else if (_clicksatus == 1)
    {
        _end = QPointF(lon, lat);

        PathPlan pp;
        pp.setChart(_chartPoint, _chartLine);
        pp.initOwnShip(_start.x(), _start.y());
        pp.initGoalShip(_end.x(), _end.y());

        double cos, dis;
        PathPlan::GetDistanceAngle(_start.y(), _start.x(), _end.y(), _end.x(), dis, cos);

        qDebug() << "_start:" << _start.x() << "," << _start.y() << endl;
        qDebug() << "_end:" << _end.x() << "," << _end.y() << endl;
        qDebug() << "goal dis is:" << dis << endl;
        qDebug() << "att:" << att << "rep:" << rep << "step:" << step << "ang:" << turnAngle << "emCPA:" << emCPA << endl;
        pp.setPara(emCPA, step, turnAngle, att, rep);
        std::vector<Pos> pathList = pp.findPath();

        _obs = pp.getObs();

        QVector<QPointF> path;
        for (auto &v : pathList)
        {
            path << QPointF(v.lon, v.lat);
        }
        setPath(path);

        pathList = pp.cutCircle(pathList);
        for (auto &v : pathList)
        {
            _optPath << QPointF(v.lon, v.lat);
        }

        pathList = pp.optimizePathRad();
        for (auto &v : pathList)
        {
            _finalPath << QPointF(v.lon, v.lat);
        }

        _clicksatus = 2;
    }
    else
    {
        _start = QPointF(0, 0);
        _end = QPointF(0, 0);
        _path.clear();
        _optPath.clear();
        _finalPath.clear();
        _clicksatus = 0;
    }

    update();
}

void RenderArea::paintEvent(QPaintEvent * /* event */)
{
    if (_allPoints.size() <= 0)
    {
        return;
    }

    Widget *p = (Widget *)parentWidget();
    bool checked_rawpath = p->_checkbox_rawpath->isChecked();
    bool checked_cut_circle_path = p->_checkbox_cut_circle_path->isChecked();
    bool checked_opt_path = p->_checkbox_opt_path->isChecked();
    bool checked_supplement_point = p->_checkbox_supplement_point->isChecked();

    QPainter painter(this);
    QPen pen;

    //=======draw chart point element =============
    pen.setColor(Qt::blue);
    pen.setWidth(10);
    painter.setPen(pen);
    for (const QPointF &p : _points)
    {
        auto sp = wp2sp(p);
        painter.drawPoint(sp);
    }

    //==========draw chart line element ============
    QList<int> keylist1 = _chartLine.keys();
    QVector<QPointF> points;
    for (int i = 0; i < keylist1.count(); ++i)
    {
        QVector<QPointF> linep;
        auto lineInfo = _chartLine[keylist1[i]].tLineInfo;
        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {
            auto lon1 = lineInfo[j].fStartLon;
            auto lat1 = lineInfo[j].fStartLat;
            auto lon2 = lineInfo[j].fEndLon;
            auto lat2 = lineInfo[j].fEndLat;
            auto p1 = wp2sp(lon1, lat1);
            auto p2 = wp2sp(lon2, lat2);
            points << p1 << p2;

            pen.setColor(Qt::green);
            pen.setWidth(3);
            painter.setPen(pen);
            painter.drawLine(p1, p2);
        }
    }
    // ========= draw chart line point ===========
    pen.setColor(Qt::red);
    pen.setWidth(6);
    painter.setPen(pen);
    for (const QPointF &p : points)
    {
        painter.drawPoint(p);
    }

    //==========draw supplement point============
    if (checked_supplement_point)
    {

        for (const auto &p : _obs)
        {
            pen.setColor(Qt::yellow);
            pen.setWidth(4);
            painter.setPen(pen);
            painter.drawPoint(wp2sp(p.lon, p.lat));
        }
    }

    //==========draw start end=======
    QColor c(219, 120, 0);
    pen.setColor(c);
    pen.setWidth(9);
    painter.setPen(pen);
    if (_clicksatus == 1)
    {
        painter.drawPoint(wp2sp(_start));
    }
    else if (_clicksatus == 2)
    {
        painter.drawPoint(wp2sp(_start));
        painter.drawPoint(wp2sp(_end));
    }

    //===========draw path===========
    // first draw path and then draw point, so that the point can be covered by path
    if (_path.size() > 0 && checked_rawpath)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _path)
        {
            auto sp = wp2sp(p);
            points << sp;
        }
        pen.setColor(Qt::green);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.drawPolyline(points);

        pen.setColor(Qt::red);
        pen.setWidth(3);
        painter.setPen(pen);
        for (const QPointF &p : points)
        {
            painter.drawPoint(p);
        }
    }

    //===========draw opt path===========
    if (_optPath.size() > 0 && checked_cut_circle_path)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _optPath)
        {
            auto sp = wp2sp(p);
            points << sp;
        }
        pen.setColor(Qt::red);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.drawPolyline(points);

        pen.setColor(Qt::red);
        pen.setWidth(6);
        painter.setPen(pen);
        for (const QPointF &p : points)
        {
            painter.drawPoint(p);
        }
    }

    if (_finalPath.size() > 0 && checked_opt_path)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _finalPath)
        {
            auto sp = wp2sp(p);
            points << sp;
        }
        pen.setColor(Qt::blue);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.drawPolyline(points);

        pen.setColor(Qt::red);
        pen.setWidth(6);
        painter.setPen(pen);
        for (const QPointF &p : points)
        {
            painter.drawPoint(p);
        }
    }
}

void RenderArea::setMap(const QHash<int, tagChartPointInfo> &chartpoint, const QHash<int, tagChartLineInfo> &chartline)
{
    _chartPoint = chartpoint;
    _chartLine = chartline;

    _allPoints.clear();
    _points.clear();
    _obs.clear();

    QList<int> keylist = chartpoint.keys();
    for (int i = 0; i < keylist.count(); ++i)
    {
        auto lon = chartpoint[keylist[i]].tPosInfo.fLon;
        auto lat = chartpoint[keylist[i]].tPosInfo.fLat;
        _points.push_back(QPointF(lon, lat));
        _allPoints.push_back(QPointF(lon, lat));
    }

    QList<int> keylist1 = chartline.keys();
    for (int i = 0; i < keylist1.count(); ++i)
    {
        QVector<QPointF> linep;
        auto lineInfo = chartline[keylist1[i]].tLineInfo;
        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {

            auto line = lineInfo[j];
            int ncount = 1;
            double dlon = (line.fEndLon - line.fStartLon) / ncount;
            double dlat = (line.fEndLat - line.fStartLat) / ncount;
            for (int count = 0; count <= ncount; ++count)
            {
                auto lon = line.fStartLon + dlon * count;
                auto lat = line.fStartLat + dlat * count;
                linep.push_back(QPointF(lon, lat));
                _allPoints.push_back(QPointF(lon, lat));
            }
        }
    }
}
