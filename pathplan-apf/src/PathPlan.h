// Create by
#ifndef PATHPLAN_H
#define PATHPLAN_H
#define NM2M 1852

#include "globalstruct.h"
#include <vector>
#include <cmath>

#include <QHash>
#include <QVector>

#define NM 1852.0
#define M_PI 3.14159265358979323846

const double EARTH_RADIUS = 3440.06479;
const int RUN_STEP = 30; // 陷入局部最优之后逃逸出的总步长
const int CHECK_CIRCLE_NUM = 30;

struct Pos
{
    double lon;
    double lat;
    double x;
    double y;
    double hdg;
    int lineId;
};

struct Segment
{
    Pos start;
    Pos end;
    int startIndex;
};

class PathPlan
{

public:
    using Polygon = std::vector<Segment>;
    using Polygons = std::vector<Polygon>;

    PathPlan() = default;
    ~PathPlan() = default;
    void initOwnShip(double lon, double lat);
    void initGoalShip(double lon, double lat);

    void addTargetObs(Pos targetobsInfo) { _targetObs.push_back(targetobsInfo); }
    void setPara(double EmCPA, double fStep, double TurnAngTOL, double att, double rep);
    std::vector<Pos> getObs() { return _targetObs; }

    bool calculate(bool isCircle);
    double getAdviceCourse() { return _adviceCourse; }
    double getRealCourse() { return _realCourse; }

    std::vector<Pos> findPath();
    void updateChartElem();
    void setChart(QHash<int, tagChartPointInfo> chartpointHash, QHash<int, tagChartLineInfo> chartlineHash);

    static double A2R(double d) { return d * M_PI / 180.0; }
    static double R2A(double d) { return d / M_PI * 180.0; }
    static void GetDistanceAngle(double lat1 /*deg*/, double lon1 /*deg*/, double lat2 /*deg*/, double lon2 /*deg*/, double &dbDist /*nm*/, double &dbCrs /*deg*/);
    static void GetRelativeXY(double lon1, double lat1, double lon2, double lat2, double &x, double &y);
    // > 0 left,  < 0 right   ab*ac  c在ab的哪一侧
    double crossProduct(const Pos &a, const Pos &b, const Pos &c)
    {
        return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    }
    bool intersect(const Pos &a, const Pos &b, const Pos &c, const Pos &d);
    bool intersect(const Pos &a, const Pos &b, const Polygon &segs);
    bool intersect(const Pos &a, const Pos &b, const Polygons &polygons);
    bool intersect(const Polygon &polygon, const Polygons &polygons);
    bool canCross(const Pos &a, const Pos &b, const Polygons &polygons);
    Polygon lineToPolygon(const Pos &a, const Pos &b, double len);
    static Pos calcNextPos(double lon, double lat, double heading, double distance);
    static double getDis(double x1, double y1, double x2, double y2)
    {
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    std::vector<Segment> splitPath(const std::vector<Pos> &path);
    std::vector<Pos> reducePathPoint(const std::vector<Pos> &path);
    std::vector<Pos> cutCircle(const std::vector<Pos> &path);
    std::vector<Pos> optimizePathCons();
    std::vector<Pos> optimizePathRad();

private:
    QHash<int, tagChartPointInfo> _chartelem_point;
    QHash<int, tagChartLineInfo> _chartelem_line;
    double _adviceCourse = 0;
    double _realCourse;

    Pos _ownshipinfo;
    Pos _goalinfo;
    std::vector<Pos> _targetObs;
    QHash<int, QVector<Pos>> _map;
    std::vector<std::vector<Segment>> _polygons;
    std::vector<Pos> _path;

    double _emCPA; // 避碰范围
    double _step;  // 步长
    double _stepOri;
    double _turnAngTOL; // 最大转向角
    double _att = 5;
    double _rep_obs = 15000;
    double _startLon;
    double _startLat;

    // 用于逃出局部最优
    bool _isCircle = false;
    int _runStep = 0;
    int _prePos = -1;      // 索引从0开始
    int _circlePos = -1;   // 索引从0开始
    int _runDirection = 0; // 小于0 left,  大于0 right
    int _same_circle_num = 0;
    double _intersect_check_dis = 0.05;
    int _attempRunNum = 0;
    int _runTotalStep = RUN_STEP;
};

#endif // PATHPLAN_H
