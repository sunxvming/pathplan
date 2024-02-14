
#include "RenderArea.h"

#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QLineEdit>
#include <QRadioButton>
#include <QComboBox>
#include <QCheckBox>
#include <QPen>

#include "Widget.h"
#include "Utils.h"

RenderArea::RenderArea(QWidget *parent)
    : QWidget(parent)
{
}

QSize RenderArea::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize RenderArea::sizeHint() const
{
    return QSize(800, 800);
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

    scale = qMin(width() / lonRange, height() / latRange) / 1;
}

void RenderArea::mousePressEvent(QMouseEvent *event)
{

    Widget *p = (Widget *)parentWidget();
    int grid_num = p->_line_edit_grid_num->text().toInt();
    int extend_num = p->_line_edit_extend_num->text().toInt();

    QPointF pos = event->pos();
    QPointF world_pos = sp2wp(pos);

    if (_clicksatus == 0)
    {
        _start = world_pos;
        _clicksatus = 1;
    }
    else if (_clicksatus == 1)
    {
        _end = world_pos;

        PathPlan pp;
        pp.setChart(_chartpoint, _chartline, grid_num, extend_num);
        CheckPathResult res = pp.checkPath({_start.x(), _start.y()}, {_end.x(), _end.y()});
        if (res == CheckPathResult::StartEndPointInvalid)
        {
            Utils::showMsg("info", "start point and end point is invalid");
        }
        else if (res == CheckPathResult::StartPointInvalid)
        {
            Utils::showMsg("info", "start point is invalid");
        }
        else if (res == CheckPathResult::EndPointInvalid)
        {
            Utils::showMsg("info", "end point is invalid");
        }
        else if (res == CheckPathResult::Unreachable)
        {
            Utils::showMsg("info", "target point unreachable");
        }

        std::vector<Grid> pathList;
        if (res == CheckPathResult::Valid)
        {
            pathList = pp.findPath({_start.x(), _start.y()}, {_end.x(), _end.y()});
        }
        // pp.setCrossGrid({_start.x(), _start.y()}, {_end.x(), _end.y()});
        _gridMap = pp.getGridMap();
        _obs = pp.getObs();

        // origin path
        QVector<QPointF> path;
        for (auto &v : pathList)
        {
            path << QPointF(v.center.x, v.center.y);
        }
        setPath(path);

        // opt path, conservative strategy
        QVector<QPointF> optPath;
        std::vector<Grid> optPathList;
        optPathList = pp.optimizePathCons();
        for (auto &v : optPathList)
        {
            optPath << QPointF(v.center.x, v.center.y);
        }
        setOptPathCons(optPath);

        // opt path, radical strategy
        optPath.clear();
        optPathList.clear();
        optPathList = pp.optimizePathRad();
        for (auto &v : optPathList)
        {
            optPath << QPointF(v.center.x, v.center.y);
        }
        setOptPathRad(optPath);

        _clicksatus = 2;
    }
    else
    {
        _start = QPointF(0, 0);
        _end = QPointF(0, 0);
        _path.clear();
        _optPathCons.clear();
        _optPathRad.clear();
        for (auto &row : _gridMap)
        {
            for (auto &grid : row)
            {
                grid.type = 0;
            }
        }
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
    bool checked_grid = p->_checkbox_grid->isChecked();
    bool checked_rawpath = p->_checkbox_rawpath->isChecked();
    bool checked_opt_path_rad = p->_checkbox_opt_path_rad->isChecked();
    bool checked_opt_path_cons = p->_checkbox_opt_path_cons->isChecked();
    bool checked_supplement_point = p->_checkbox_supplement_point->isChecked();

    QPainter painter(this);
    QPen pen;

    //===========draw grid and grid center point===========
    pen.setColor(Qt::red);
    pen.setWidth(1);
    painter.setPen(pen);
    if (_gridMap.size() > 0 && checked_grid)
    {
        for (auto row : _gridMap)
        {
            for (auto grid : row)
            {
                auto p1 = wp2sp(grid.tl);
                auto p2 = wp2sp(grid.tr);
                auto p3 = wp2sp(grid.bl);
                auto p4 = wp2sp(grid.br);

                QColor c(255, 128, 178);
                pen.setColor(c);
                pen.setWidth(1);
                painter.setPen(pen);
                painter.drawLine(p1, p2);
                painter.drawLine(p2, p4);
                painter.drawLine(p4, p3);
                painter.drawLine(p3, p1);

                // draw grid center point
                // pen.setColor(Qt::red);
                // pen.setWidth(2);
                // painter.setPen(pen);
                // painter.drawPoint(wp2sp(grid.center));

                if (!grid.isWalkable)
                {
                    pen.setWidth(1);
                    painter.setPen(pen);
                    // 创建一个矩形
                    QRectF rect(p1, p4);
                    // 创建一个画刷并设置其颜色
                    QColor lightGray(245, 163, 245);

                    QBrush brush(lightGray);
                    // 使用画刷填充矩形
                    painter.fillRect(rect, brush);
                    // 绘制矩形的边框
                    painter.drawRect(rect);
                }
                if (grid.type == 1)
                {
                    // pen.setWidth(1);
                    // painter.setPen(pen);
                    // // 创建一个矩形
                    // QRectF rect(p1, p4);
                    // // 创建一个画刷并设置其颜色
                    // QColor c(219, 120, 0);

                    // QBrush brush(c);
                    // 使用画刷填充矩形
                    // painter.fillRect(rect, brush);
                    // // 绘制矩形的边框
                    // painter.drawRect(rect);
                }
                if (grid.type == 2)
                { // 算法探索过的格子
                    pen.setWidth(1);
                    painter.setPen(pen);
                    // 创建一个矩形
                    QRectF rect(p1, p4);
                    // 创建一个画刷并设置其颜色
                    QColor c(102, 216, 255);

                    QBrush brush(c);
                    painter.fillRect(rect, brush);
                    // 绘制矩形的边框
                    painter.drawRect(rect);
                }
            }
        }
    }

    //==========draw supplement point============
    if (checked_supplement_point)
    {
        for (const auto &p : _obs)
        {
            pen.setColor(Qt::yellow);
            pen.setWidth(5);
            painter.setPen(pen);
            painter.drawPoint(wp2sp(p));
        }
    }
    //==========draw map point============
    for (const auto &p : _allPoints)
    {
        pen.setColor(Qt::blue);
        pen.setWidth(5);
        painter.setPen(pen);
        painter.drawPoint(wp2sp(p));
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
    if (_path.size() > 0 && checked_rawpath)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _path)
        {
            auto sp = wp2sp(p);
            points << sp;
        }
        pen.setColor(Qt::green);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.drawPolyline(points);
    }

    // ===========draw opt path cons===========
    if (_optPathCons.size() > 0 && checked_opt_path_cons)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _optPathCons)
        {
            auto sp = wp2sp(p);
            points << sp;
        }

        pen.setColor(Qt::blue);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.drawPolyline(points);
    }

    // ===========draw opt path rad ===========
    if (_optPathRad.size() > 0 && checked_opt_path_rad)
    {
        QVector<QPointF> points;
        for (const QPointF &p : _optPathRad)
        {
            auto sp = wp2sp(p);
            points << sp;
        }

        pen.setColor(Qt::yellow);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.drawPolyline(points);
    }
}

void RenderArea::setMap(QHash<int, tagChartPointInfo> chartpoint, QHash<int, tagChartLineInfo> chartline)
{
    _allPoints.clear();
    _path.clear();
    _optPathCons.clear();
    _optPathRad.clear();
    _start = QPointF(0, 0);
    _end = QPointF(0, 0);
    _gridMap.clear();
    _obs.clear();
    _clicksatus = 0;

    _chartpoint = chartpoint;
    _chartline = chartline;

    QList<int> keylist = chartpoint.keys();
    for (int i = 0; i < keylist.count(); ++i)
    {
        auto lon = chartpoint[keylist[i]].tPosInfo.fLon;
        auto lat = chartpoint[keylist[i]].tPosInfo.fLat;
        _allPoints.push_back(QPointF(lon, lat));
    }

    QList<int> keylist1 = chartline.keys();
    for (int i = 0; i < keylist1.count(); ++i)
    {
        auto lineInfo = chartline[keylist1[i]].tLineInfo;
        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {
            auto lon1 = lineInfo[j].fStartLon;
            auto lat1 = lineInfo[j].fStartLat;
            auto lon2 = lineInfo[j].fEndLon;
            auto lat2 = lineInfo[j].fEndLat;

            _allPoints.push_back(QPointF(lon1, lat1));
            _allPoints.push_back(QPointF(lon2, lat2));
        }
    }
}
