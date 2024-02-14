#include "PathPlan.h"
#include <unordered_map>
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <cassert>
#include <QVector2D>
#include <QDebug>

void PathPlan::setPara(double EmCPA, double fStep, double TurnAngTOL, double att, double rep)
{
    _emCPA = EmCPA;
    _step = fStep;
    _stepOri = fStep;
    _turnAngTOL = TurnAngTOL;
    _att = att;
    _rep_obs = rep;
}

double trans_agl(double angle)
{
    if (angle < -180)
        angle += 360;
    else if (angle > 180)
        angle -= 360;
    return angle;
}

bool PathPlan::calculate(bool isCircle)
{
    // 涉及方向均为角度，涉及位置，速度，单位均为海里
    double goal_dis = getDis(_ownshipinfo.x, _ownshipinfo.y, _goalinfo.x, _goalinfo.y);

    if (goal_dis < _stepOri * 3)
    {
        return true;
    }
    auto attDir = QVector2D(_goalinfo.x - _ownshipinfo.x, _goalinfo.y - _ownshipinfo.y);
    QVector2D F_att = _att * attDir / goal_dis;
    if (isCircle)
    {
        // 判断逃逸力的方向逻辑
        if (_runDirection == 0)
        {
            // 确定多边形的lineId
            std::unordered_map<int, int> countMap;
            for (std::size_t i = 0; i < _targetObs.size(); ++i)
            {
                auto obs = _targetObs[i];
                double tmp_dis = getDis(_ownshipinfo.x, _ownshipinfo.y, obs.x, obs.y);
                if (tmp_dis < _emCPA * 2)
                {
                    countMap[obs.lineId]++;
                }
            }
            if (countMap.size() > 0)
            {
                // qDebug() << "======countMap=======";
                // for(const auto& entry:countMap){
                //     qDebug()<< entry.first << ":" << entry.second;
                // }

                auto maxEntry = std::max_element(countMap.begin(), countMap.end(),
                                                 [](const std::pair<int, int> &a, const std::pair<int, int> &b)
                                                 {
                                                     return a.second < b.second;
                                                 });
                int closeLineId = maxEntry->first;
                // qDebug()<< "closeLineId" << ":" << closeLineId;

                // 统计两侧的数量
                auto closePloygen = _map[closeLineId];
                int leftNum = 0;
                int rightNum = 0;
                for (const auto &v : closePloygen)
                {
                    double ret = crossProduct(_ownshipinfo, _goalinfo, v);
                    if (ret > 0)
                        leftNum++;
                    if (ret < 0)
                        rightNum++;
                }

                // qDebug()<< "leftNum is:" << leftNum;
                // qDebug()<< "rightNum is:" << rightNum ;
                if (leftNum > rightNum)
                {
                    _runDirection = 1;
                }
                else
                {
                    _runDirection = -1;
                }
            }
            else
            {
                _runDirection = 1;
            }
        }
        QVector2D verticalDir;
        if (_runDirection <= 0)
        {
            verticalDir = QVector2D(-(_goalinfo.y - _ownshipinfo.y), _goalinfo.x - _ownshipinfo.x);
            // qDebug()<< "go left============";
        }
        else
        {
            verticalDir = QVector2D(_goalinfo.y - _ownshipinfo.y, -(_goalinfo.x - _ownshipinfo.x));
            // qDebug()<< "go right============";
        }

        F_att = _att * verticalDir / goal_dis; // 逃逸力方向为垂直于目标的方向
    }
    QVector2D F_rep = QVector2D(0, 0);

    int far_point_num = 0;
    int near_point_num = 0;
    double far_dis = _stepOri * 15; // 在周围无障碍物时，增加每次的步长以进行优化
    for (std::size_t i = 0; i < _targetObs.size(); ++i)
    {
        auto obs = _targetObs[i];
        double tmp_dis = getDis(_ownshipinfo.x, _ownshipinfo.y, obs.x, obs.y);
        if (tmp_dis < far_dis)
        {
            far_point_num++;
        }
        if (tmp_dis < _emCPA / 2)
        {
            near_point_num++;
        }

        if (tmp_dis > _emCPA)
        {
            continue;
        }

        QVector2D tmp_vec_obs = QVector2D((obs.x - _ownshipinfo.x) / tmp_dis, (obs.y - _ownshipinfo.y) / tmp_dis);

        // 障碍物对我船基本斥力，方向由他船指向我船
        QVector2D F_rep_tmp = -(_rep_obs * pow(1 / tmp_dis - 1 / _emCPA, 2) * pow(goal_dis, 2) / pow(tmp_dis, 2)) * tmp_vec_obs;

        F_rep += F_rep_tmp;
    }
    // qDebug() << "far_point_num:" << far_point_num << " goal_dis:" << goal_dis << " far_dis:" << far_dis ;
    if (far_point_num == 0 && goal_dis > far_dis)
    {
        _step = _stepOri * 10;
    }
    else if (near_point_num > 0)
    {
        _step = _stepOri / 3;
    }
    else
    {
        _step = _stepOri;
    }
    if (isCircle)
    {
        _step = _stepOri;
    }

    QVector2D F_tol = F_att + F_rep;

    _adviceCourse = atan2(F_tol.x(), F_tol.y()) * 180.0 / 3.1415;
    _realCourse = _adviceCourse;
    double delta_angle = trans_agl(_adviceCourse - _ownshipinfo.hdg);

    if (delta_angle >= _turnAngTOL)
    {
        _adviceCourse = _ownshipinfo.hdg + _turnAngTOL;
    }
    else if (delta_angle <= -_turnAngTOL)
    {
        _adviceCourse = _ownshipinfo.hdg - _turnAngTOL;
    }

    if (_adviceCourse < 0)
    {
        _adviceCourse += 360.0;
    }
    if (_adviceCourse > 360.0)
    {
        _adviceCourse -= 360.0;
    }

    return false;
}

std::vector<Pos> PathPlan::findPath()
{
    _path.clear();
    _path.push_back(Pos{_ownshipinfo.lon, _ownshipinfo.lat, 0, 0, 0, 0});
    updateChartElem();

    bool finish = false;
    for (int i = 0; i <= 10000; i++)
    {
        // qDebug()<<"times:" << i;
        if (_isCircle && _runStep < _runTotalStep)
        { // 走完_runTotalStep后重置，不再有逃逸力
            _runStep++;
        }
        else
        {
            _isCircle = false;
            _runStep = 0;
        }
        // qDebug()<<"calculate begin:";
        finish = calculate(_isCircle);
        // qDebug()<<"calculate end:";
        if (finish)
        {
            break;
        }

        // 更新角度
        if (0 == i)
        { // 首次直接转到对应角度
            _ownshipinfo.hdg = getRealCourse();
        }
        else
        {
            _ownshipinfo.hdg = getAdviceCourse();
        }

        // 计算next经纬度
        Pos pos = PathPlan::calcNextPos(_ownshipinfo.lon, _ownshipinfo.lat, _ownshipinfo.hdg, _step);
        pos.hdg = _ownshipinfo.hdg;
        // 更新本船信息
        _ownshipinfo.lon = pos.lon;
        _ownshipinfo.lat = pos.lat;
        double dCos1, dDistance;
        PathPlan::GetDistanceAngle(_startLat, _startLon, pos.lat, pos.lon, dDistance, dCos1);

        auto x = dDistance * sin(PathPlan::A2R(dCos1));
        auto y = dDistance * cos(PathPlan::A2R(dCos1));
        _ownshipinfo.x = x;
        _ownshipinfo.y = y;
        pos.x = x;
        pos.y = y;
        // 判定局部最优
        if (_path.size() > CHECK_CIRCLE_NUM)
        {
            auto prePos = _path.at(_path.size() - CHECK_CIRCLE_NUM - 1);
            double dis, cos;
            PathPlan::GetDistanceAngle(prePos.lat, prePos.lon, pos.lat, pos.lon, dis, cos);
            if (!_isCircle && dis < _stepOri * 10)
            {
                qDebug() << "................go to circle..................";
                _isCircle = true;
                // 判断陷入点和上一次是否相同
                bool isSameCircle = false;
                if (_circlePos > -1 && (std::size_t)_circlePos < _path.size())
                { // 之前陷入过，并且和之前的距离小于10，判定相同
                    auto preCircle = _path[_circlePos];
                    double dis, cos;
                    PathPlan::GetDistanceAngle(preCircle.lat, preCircle.lon, pos.lat, pos.lon, dis, cos);

                    if (dis < _stepOri * 150)
                    {
                        isSameCircle = true;
                    }
                }
                qDebug() << "check isSameCircle:" << isSameCircle << " _attempRunNum:" << _attempRunNum << " _runDirection:" << _runDirection;
                if (isSameCircle)
                { // 陷入点相同逻辑，  若多次无法逃出，改变方向
                    if (_attempRunNum > 5)
                    {
                        _runDirection = -_runDirection;
                        _attempRunNum = 0;
                    }
                }
                else
                { // 不同逻辑，第一次也视为不同的
                    _runDirection = 0;
                    _prePos = -1;
                    _runTotalStep = RUN_STEP;
                    _attempRunNum = 0;
                    if (_circlePos == -1)
                    {                                                                                                              // 第一个陷入点
                        _circlePos = ((int)_path.size() - CHECK_CIRCLE_NUM) >= 0 ? _path.size() - CHECK_CIRCLE_NUM : _path.size(); // 设置陷入后的位置
                    }
                    else
                    { // 第二个陷入点
                        if (int(_path.size() - CHECK_CIRCLE_NUM) < _circlePos)
                        { // 非首次陷入，往后倒的太多了
                            _circlePos = _path.size();
                        }
                        else
                        {
                            _circlePos = _path.size() - CHECK_CIRCLE_NUM; // 相距较远的，非首次陷入
                        }
                    }
                }
                // qDebug() << "_prePos is:" << _prePos << " path size is:" << path.size();
                // 处理二次逃逸的逻辑
                if (_prePos > 0 && _prePos < (int)_path.size())
                { // 二次逃逸，逃逸一次后没有逃出，继续从最远处接着逃
                    // 设置逃逸速度
                    //  _runTotalStep = _attempRunNum * RUN_STEP;
                    _runTotalStep = RUN_STEP << _attempRunNum;
                    _attempRunNum++;
                    _path.erase(_path.begin() + _prePos + 1, _path.end()); // 前闭后开
                    _ownshipinfo.lon = _path.back().lon;
                    _ownshipinfo.lat = _path.back().lat;
                    _ownshipinfo.hdg = _path.back().hdg;
                    _prePos = _path.size() + _runTotalStep - 1; // 索引从0开始
                    qDebug() << "_runTotalStep is:" << _runTotalStep;
                }
                else
                {                                                             // 首次逃逸逻辑
                    _path.erase(_path.end() - CHECK_CIRCLE_NUM, _path.end()); // 往前回溯
                    _ownshipinfo.lon = _path.back().lon;
                    _ownshipinfo.lat = _path.back().lat;
                    _ownshipinfo.hdg = _path.back().hdg;
                    _prePos = _path.size() + _runTotalStep - 1; // 上一次逃逸逃到的位置，索引从0开始
                }
                continue;
            }
        }
        _path.push_back(pos);

        //        if(i % 10 == 0){
        //            qDebug()<<"times:" << i << "heading:" << _ownshipinfo.hdg << " lon:" << pos.lon << " " << "lat:" << pos.lat << "\n";
        //        }
    }
    if (finish)
    {
        _path.push_back(Pos{_goalinfo.lon, _goalinfo.lat, _goalinfo.x, _goalinfo.y, 0, 0});
    }
    return _path;
}

void PathPlan::initOwnShip(double lon, double lat)
{

    _startLon = lon;
    _startLat = lat;
    _ownshipinfo.lon = lon;
    _ownshipinfo.lat = lat;
    _ownshipinfo.x = 0;
    _ownshipinfo.y = 0;
}
void PathPlan::initGoalShip(double lon, double lat)
{
    _goalinfo.lon = lon;
    _goalinfo.lat = lat;
    double dCos1, dDistance;
    PathPlan::GetDistanceAngle(_ownshipinfo.lat, _ownshipinfo.lon, lat, lon, dDistance, dCos1);
    _goalinfo.x = dDistance * sin(PathPlan::A2R(dCos1));
    _goalinfo.y = dDistance * cos(PathPlan::A2R(dCos1));
}

void PathPlan::setChart(QHash<int, tagChartPointInfo> chartpointHash, QHash<int, tagChartLineInfo> chartlineHash)
{
    _chartelem_point = chartpointHash;
    _chartelem_line = chartlineHash;
}

void PathPlan::updateChartElem()
{
    _map.clear();
    QList<int> keylist = _chartelem_point.keys();
    for (int i = 0; i < keylist.count(); ++i)
    {
        Pos pos;
        auto fLon = _chartelem_point[keylist[i]].tPosInfo.fLon;
        auto fLat = _chartelem_point[keylist[i]].tPosInfo.fLat;

        double dCos1, dDistance;
        PathPlan::GetDistanceAngle(_ownshipinfo.lat, _ownshipinfo.lon, fLat, fLon, dDistance, dCos1);
        pos.x = dDistance * sin(PathPlan::A2R(dCos1));
        pos.y = dDistance * cos(PathPlan::A2R(dCos1));
        pos.lon = fLon;
        pos.lat = fLat;

        addTargetObs(pos);
    }
    QList<int> keylist1 = _chartelem_line.keys();
    for (int i = 0; i < keylist1.count(); ++i)
    {
        int lineId = keylist1[i];
        auto lineInfo = _chartelem_line[lineId].tLineInfo;
        QVector<Pos> mapLine;
        std::vector<Segment> polygonLine;
        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {
            auto line = lineInfo[j];
            double dis = 0;
            double fcos = 0;
            PathPlan::GetDistanceAngle(line.fStartLat, line.fStartLon, line.fEndLat, line.fEndLon, dis, fcos);
            // int ncount = 1;
            int ncount = (dis / _emCPA) + 1;
            double dlon = -(line.fStartLon - line.fEndLon) / ncount;
            double dlat = -(line.fStartLat - line.fEndLat) / ncount;
            Segment seg;
            for (int count = 0; count <= ncount; ++count)
            {
                Pos pos;
                auto fLon = line.fStartLon + dlon * count;
                auto fLat = line.fStartLat + dlat * count;

                double dCos1, dDistance;
                PathPlan::GetDistanceAngle(_ownshipinfo.lat, _ownshipinfo.lon, fLat, fLon, dDistance, dCos1);
                pos.x = dDistance * sin(PathPlan::A2R(dCos1));
                pos.y = dDistance * cos(PathPlan::A2R(dCos1));
                pos.lon = fLon;
                pos.lat = fLat;
                pos.lineId = lineId;
                addTargetObs(pos);
                mapLine.push_back(pos);

                if (count == 0)
                {
                    seg.start = pos;
                }
                if (count == ncount)
                {
                    seg.end = pos;
                }
            }
            polygonLine.push_back(seg);
        }
        _map[lineId] = mapLine;
        _polygons.push_back(polygonLine);
    }
}

void PathPlan::GetDistanceAngle(double lat1 /*deg*/, double lon1 /*deg*/, double lat2 /*deg*/, double lon2 /*deg*/, double &d /*nm*/, double &angle /*deg*/)
{
    // calculate distance
    double radlat1 = A2R(lat1), radlat2 = A2R(lat2);
    double radlon1 = A2R(lon1), radlon2 = A2R(lon2);
    double dlat = radlat2 - radlat1;
    double dlon = radlon2 - radlon1;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(radlat1) * cos(radlat2) *
                   sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    d = EARTH_RADIUS * c;
    angle = R2A(atan2(sin(radlon2 - radlon1) *
                          cos(radlat2),
                      cos(radlat1) * sin(radlat2) -
                          sin(radlat1) * cos(radlat2) *
                              cos(radlon2 - radlon1)));
    if (angle < 0)
    {
        angle += 360.0;
    }
    if (angle > 360.0)
    {
        angle -= 360.0;
    }
}

void PathPlan::GetRelativeXY(double lon1, double lat1, double lon2, double lat2, double &x, double &y)
{
    double distance = 0.0, angle = 0.0;
    GetDistanceAngle(lon1, lat1, lon2, lat2, distance, angle);
    x = distance * sin(angle);
    y = distance * cos(angle);
}

// 大圆法求两点间距离
Pos PathPlan::calcNextPos(double lon, double lat, double heading, double distance)
{
    Pos result;
    double lonRad = A2R(lon);
    double latRad = A2R(lat);
    double headingRad = A2R(heading);

    double d = distance / EARTH_RADIUS;
    double nextLatRad = asin(sin(latRad) * cos(d) + cos(latRad) * sin(d) * cos(headingRad));
    double nextLonRad = lonRad + atan2(sin(headingRad) * sin(d) * cos(latRad), cos(d) - sin(latRad) * sin(nextLatRad));

    result.lon = R2A(nextLonRad);
    result.lat = R2A(nextLatRad);

    return result;
}

std::vector<Segment> PathPlan::splitPath(const std::vector<Pos> &path)
{
    std::vector<Segment> segments;

    for (decltype(path.size()) i = 0; i < path.size() - 1; i++)
    {
        Segment s;
        s.start = path[i];
        s.end = path[i + 1];
        s.startIndex = i;
        segments.push_back(s);
    }
    return segments;
}

bool PathPlan::intersect(const Pos &a, const Pos &b, const Pos &c, const Pos &d)
{
    if (std::max(a.x, b.x) < std::min(c.x, d.x))
    {
        return false;
    }
    if (std::max(a.y, b.y) < std::min(c.y, d.y))
    {
        return false;
    }
    if (std::max(c.x, d.x) < std::min(a.x, b.x))
    {
        return false;
    }
    if (std::max(c.y, d.y) < std::min(a.y, b.y))
    {
        return false;
    }

    if (crossProduct(a, b, c) * crossProduct(a, b, d) > 0)
    {
        return false;
    }
    if (crossProduct(c, d, a) * crossProduct(c, d, b) > 0)
    {
        return false;
    }
    return true;
}

bool PathPlan::intersect(const Pos &a, const Pos &b, const Polygon &segs)
{
    for (const auto &v : segs)
    {
        if (intersect(a, b, v.start, v.end))
        {
            return true;
        }
    }
    return false;
}

bool PathPlan::intersect(const Pos &a, const Pos &b, const Polygons &polygons)
{
    for (const auto &v : polygons)
    {
        if (intersect(a, b, v))
        {
            return true;
        }
    }
    return false;
}

bool PathPlan::intersect(const Polygon &polygon, const Polygons &polygons)
{
    for (const auto &v : polygon)
    {
        if (intersect(v.start, v.end, polygons))
        {
            return true;
        }
    }
    return false;
}

PathPlan::Polygon PathPlan::lineToPolygon(const Pos &a, const Pos &b, double len)
{

    Pos direct;
    direct.x = b.x - a.x;
    direct.y = b.y - a.y;
    double length = std::sqrt(direct.x * direct.x + direct.y * direct.y);
    Pos unit;
    unit.x = direct.x / length;
    unit.y = direct.y / length;

    Pos perp;
    perp.x = -unit.y * len;
    perp.y = unit.x * len;

    Pos tl, tr, bl, br;
    tl.x = a.x - perp.x;
    tl.y = a.y - perp.y;

    tr.x = a.x + perp.x;
    tr.y = a.y + perp.y;

    bl.x = b.x - perp.x;
    bl.y = b.y - perp.y;

    br.x = b.x + perp.x;
    br.y = b.y + perp.y;

    std::vector<Segment> segs;
    segs.push_back(Segment{tl, tr, 0});
    segs.push_back(Segment{bl, br, 0});
    segs.push_back(Segment{tl, bl, 0});
    segs.push_back(Segment{tr, br, 0});
    return segs;
}

bool PathPlan::canCross(const Pos &a, const Pos &b, const Polygons &polygons)
{
    auto rect = lineToPolygon(a, b, _intersect_check_dis);
    return !intersect(rect, polygons);
}

std::vector<Pos> PathPlan::cutCircle(const std::vector<Pos> &path)
{
    std::vector<std::pair<std::size_t, std::size_t>> cutPoints;
    std::vector<Segment> segments = splitPath(path);
    for (std::size_t i = 0; i < segments.size() - 2; i++)
    {
        int intersectId = -1;
        auto segA = segments[i];
        for (std::size_t j = i + 2; j < segments.size(); j++)
        {
            auto segB = segments[j];
            if (intersect(segA.start, segA.end, segB.start, segB.end))
            {
                intersectId = segB.startIndex;
            }
        }
        if (intersectId > 0)
        {
            cutPoints.push_back(std::pair<std::size_t, std::size_t>{i + 1, intersectId});
            i = intersectId;
        }
    }
    std::vector<Pos> newPath;
    for (std::size_t i = 0; i < path.size(); i++)
    {
        bool isCut = false;
        for (const auto &v : cutPoints)
        {
            if (v.first <= i && i <= v.second)
            {
                isCut = true;
            }
        }
        if (!isCut)
        {
            newPath.push_back(path[i]);
        }
    }
    return newPath;
}

std::vector<Pos> PathPlan::reducePathPoint(const std::vector<Pos> &path)
{
    std::unordered_set<int> passIndex;
    for (std::size_t i = 1; i < path.size() - 1; i++)
    {
        auto pre = path[i - 1];
        auto cur = path[i];
        if (std::abs(cur.hdg - pre.hdg) <= 1)
        {
            passIndex.insert(i);
        }
    }
    std::vector<Pos> newPath;
    for (std::size_t i = 0; i < path.size(); i++)
    {
        if (passIndex.find(i) != passIndex.end())
        {
            continue;
        }
        newPath.push_back(path[i]);
    }
    return newPath;
}

// 保守策略, 优化的路径段在遇到第一个不能连通的点就停止
std::vector<Pos> PathPlan::optimizePathCons()
{
    if (_path.size() <= 2)
    {
        return _path;
    }

    std::vector<Pos> newPath;
    bool finish = false;
    newPath.push_back(_path[0]);
    for (std::size_t i = 0; i < _path.size(); i++)
    {
        if (finish)
        {
            break;
        }
        int thoughIndex = -1;
        for (std::size_t j = i + 1; j < _path.size(); j++)
        {
            auto cur = _path[i];
            auto next = _path[j];
            if (canCross(cur, next, _polygons))
            {
                thoughIndex = j;
                if ((std::size_t)thoughIndex == _path.size() - 1)
                { // 直连终点
                    finish = true;
                    newPath.push_back(_path[thoughIndex]);
                    break;
                }
            }
            else
            {
                if (thoughIndex < 0)
                { // 向前推进一点
                    thoughIndex = j;
                }
                newPath.push_back(_path[thoughIndex]);
                i = thoughIndex; // 更新到下一个点
                i--;
                break;
            }
        }
    }
    return newPath;
}

// 激进策略, 优化的路径段在从起点开始, 一直到终点, 一直走到不能走为止
std::vector<Pos> PathPlan::optimizePathRad()
{
    if (_path.size() <= 2)
    {
        return _path;
    }

    std::vector<Pos> newPath;
    bool finish = false;
    newPath.push_back(_path[0]);
    for (std::size_t i = 0; i < _path.size() - 1; i++)
    {
        if (finish)
        {
            break;
        }
        int thoughIndex = -1;
        for (std::size_t j = i + 1; j < _path.size(); j++)
        {
            auto cur = _path[i];
            auto next = _path[j];
            if (canCross(cur, next, _polygons))
            {
                thoughIndex = j;
                if ((std::size_t)thoughIndex == _path.size() - 1)
                { // 直连终点
                    finish = true;
                    newPath.push_back(_path[thoughIndex]);
                    break;
                }
            }
        }

        if (thoughIndex < 0)
        { // 向前推进一点
            newPath.push_back(_path[i + 1]);
        }
        else
        {
            newPath.push_back(_path[thoughIndex]);
            i = thoughIndex; // 更新到下一个点
            i--;
        }
    }
    return newPath;
}
