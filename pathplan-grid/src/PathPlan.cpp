//========================================================================
// Name: pathplan.cpp
// Description: Grid-based map representation and path planning using the A* algorithm
// Author: sunxvming@gmail.com
// Modified: 2023.10.19
//========================================================================

#include "PathPlan.h"
#include <unordered_map>
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <cassert>
#include <queue>
#include <QDebug>

// 设置格子被探索过
void PathPlan::setExplored(GridIndex index)
{
    grid(index).type = 2;
}

// 设置两点连线穿过的格子，用于调试，验证getCrossGrid算法是否正确
void PathPlan::setCrossGrid(Pos start, Pos end)
{
    auto start_index = getGridIndex(start.x - _minLon, start.y - _minLat);
    auto end_index = getGridIndex(end.x - _minLon, end.y - _minLat);

    auto cross_grid = getCrossGrid(start_index, end_index, 1);
    for (auto &v : cross_grid)
    {
        grid(v).type = 1;
    }
}

int PathPlan::calcHScore(GridIndex start, GridIndex end)
{
    int score = std::abs(start.x - end.x) + std::abs(start.y - end.y);
    // double score = getDis(start.x, start.y, end.x, end.y);
    return score * 10; // bacause one grid cost is 10, multiply by 10 to avoid using float
}

Grid &PathPlan::grid(GridIndex index)
{
    // set index.x and index.y in 0 ~ _grid_num
    int x = std::max(index.x, 0);
    x = std::min(x, _grid_num - 1);

    int y = std::max(index.y, 0);
    y = std::min(y, _grid_num - 1);

    return _gridMap[x][y];
}

CheckPathResult PathPlan::checkPath(Pos start, Pos end)
{
    auto start_index = getGridIndex(start.x - _minLon, start.y - _minLat);
    auto end_index = getGridIndex(end.x - _minLon, end.y - _minLat);
    if (grid(start_index).isWalkable == false && grid(end_index).isWalkable == false)
    {
        return CheckPathResult::StartEndPointInvalid;
    }

    if (grid(start_index).isWalkable == false)
    {
        return CheckPathResult::StartPointInvalid;
    }

    if (grid(end_index).isWalkable == false)
    {
        return CheckPathResult::EndPointInvalid;
    }
    auto path = findPath(start, end);
    if (path.size() == 0)
    {
        return CheckPathResult::Unreachable;
    }
    return CheckPathResult::Valid;
}

std::vector<Grid> PathPlan::findPath(Pos start, Pos end)
{
    _path.clear();
    auto start_index = getGridIndex(start.x - _minLon, start.y - _minLat);
    auto end_index = getGridIndex(end.x - _minLon, end.y - _minLat);
    qDebug() << "start:" << start_index.x << "," << start_index.y << " end:" << end_index.x << "," << end_index.y;
    grid(start_index).center = start;
    grid(end_index).center = end;

    std::unordered_set<GridIndex, GridIndexHash> closed_set;

    auto cmp = [this](GridIndex &a, GridIndex &b) -> bool
    {
        int s1 = grid(a).hScore + grid(a).gScore;
        int s2 = grid(b).hScore + grid(b).gScore;
        return s1 > s2;
    };
    std::priority_queue<GridIndex, std::vector<GridIndex>, decltype(cmp)> open_list(cmp);
    std::unordered_set<GridIndex, GridIndexHash> open_list_set;

    // 设置起点的代价为0
    grid(start_index).gScore = 0;
    grid(start_index).hScore = calcHScore(start_index, end_index);
    setExplored(start_index);
    // 添加起点到开放列表
    open_list.push(start_index);
    open_list_set.insert(start_index);

    while (!open_list.empty())
    {
        // 从开放列表中选择代价最小的格子
        GridIndex current_index = open_list.top();

        // 退出条件：如果当前格子是目标格子，回溯路径并返回
        if (current_index == end_index)
        {
            while (current_index != start_index)
            {
                _path.push_back(grid(current_index));
                current_index = grid(current_index).parent;
            }
            _path.push_back(grid(start_index));
            std::reverse(_path.begin(), _path.end());
            return _path;
        }

        // move current node from open to closed
        open_list.pop();
        open_list_set.erase(current_index);
        closed_set.insert({current_index.x, current_index.y});

        // 获取当前格子的邻居
        int x = current_index.x;
        int y = current_index.y;

        Neighbor t = {{x - 1, y}, false};
        Neighbor b = {{x + 1, y}, false};
        Neighbor l = {{x, y - 1}, false};
        Neighbor r = {{x, y + 1}, false};
        Neighbor tl = {{x - 1, y - 1}, true};
        Neighbor tr = {{x - 1, y + 1}, true};
        Neighbor bl = {{x + 1, y - 1}, true};
        Neighbor br = {{x + 1, y + 1}, true};

        std::unordered_map<std::string, Neighbor> neighbors = {{"t", t}, {"b", b}, {"l", l}, {"r", r}, {"tl", tl}, {"tr", tr}, {"bl", bl}, {"br", br}};

        for (const auto &pair : neighbors)
        {
            auto key = pair.first;
            auto neighbor = pair.second.index;
            auto isDiagonal = pair.second.isDiagonal;

            if (neighbor.x < 0 || neighbor.x >= _grid_num || neighbor.y < 0 || neighbor.y >= _grid_num)
            {
                continue; // 忽略超出地图范围的格子
            }

            if (closed_set.find(neighbor) != closed_set.end() || !grid(neighbor).isWalkable)
            {
                continue; // 忽略已经在关闭列表中的格子或不可行走的格子
            }
            auto t_index = neighbors["t"].index;
            auto b_index = neighbors["b"].index;
            auto l_index = neighbors["l"].index;
            auto r_index = neighbors["r"].index;

            if (key == "tl" && grid(t_index).isWalkable == false && grid(l_index).isWalkable == false)
            {
                continue;
            }
            if (key == "tr" && grid(t_index).isWalkable == false && grid(r_index).isWalkable == false)
            {
                continue;
            }
            if (key == "bl" && grid(b_index).isWalkable == false && grid(l_index).isWalkable == false)
            {
                continue;
            }
            if (key == "br" && grid(b_index).isWalkable == false && grid(r_index).isWalkable == false)
            {
                continue;
            }

            // 计算到邻居格子的实际代价，横向或纵向移动的代价为10，斜向移动的代价为14
            double tentative_g_score;
            if (isDiagonal)
            {
                tentative_g_score = grid(current_index).gScore + 14;
            }
            else
            {
                tentative_g_score = grid(current_index).gScore + 10;
            }

            // 如果邻居不在开放列表中，或者新的代价更低
            bool in_open_list_set = open_list_set.find(neighbor) != open_list_set.end();
            if (!in_open_list_set || tentative_g_score < grid(neighbor).gScore)
            {
                grid(neighbor).parent = current_index;
                grid(neighbor).gScore = tentative_g_score;
                grid(neighbor).hScore = calcHScore(neighbor, end_index);

                // 如果邻居不在开放列表中，将其加入
                if (!in_open_list_set)
                {
                    open_list.push(neighbor);
                    open_list_set.insert(neighbor);
                    setExplored(neighbor);
                }
            }
        }
    }

    // 如果开放列表为空，无法找到路径，返回空路径
    return _path;
}

void PathPlan::setChart(QHash<int, tagChartPointInfo> chartpointHash, QHash<int, tagChartLineInfo> chartlineHash, int grid_num, int extend_num)
{
    _grid_num = grid_num;
    _chartelemPoint = chartpointHash;
    _chartelemLine = chartlineHash;
    calcGrid();
    updateChartElem();
    genGrid(extend_num);
}

void PathPlan::calcGrid()
{
    double minLon = 180;
    double minLat = 90;
    double maxLon = -180;
    double maxLat = -90;
    QList<int> keylist1 = _chartelemLine.keys();
    for (int i = 0; i < keylist1.count(); ++i)
    {
        int lineId = keylist1[i];
        auto lineInfo = _chartelemLine[lineId].tLineInfo; // lineInfo 为多边形

        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {
            auto line = lineInfo[j];
            minLon = std::min(minLon, line.fStartLon);
            minLon = std::min(minLon, line.fEndLon);
            minLat = std::min(minLat, line.fStartLat);
            minLat = std::min(minLat, line.fEndLat);
            maxLon = std::max(maxLon, line.fStartLon);
            maxLon = std::max(maxLon, line.fEndLon);
            maxLat = std::max(maxLat, line.fStartLat);
            maxLat = std::max(maxLat, line.fEndLat);
        }
    }

    _minLon = minLon;
    _minLat = minLat;
    _maxLon = maxLon;
    _maxLat = maxLat;
    _grid_len = std::max(_maxLat - _minLat, _maxLon - _minLon) / _grid_num;

    double dis_lon = 0;
    double dis_lat = 0;
    double angle = 0;
    getDistanceAngle(_minLat, _minLon, _maxLat, _minLon, dis_lat, angle);
    getDistanceAngle(_minLat, _minLon, _minLat, _maxLon, dis_lon, angle);
    double grid_nm_len = std::max(dis_lon, dis_lat) / _grid_num;
    qDebug() << "dis_lon:" << dis_lon << " dis_lat:" << dis_lat << " grid_nm_len:" << grid_nm_len;

    qDebug() << "minLon:" << minLon << " minLat:" << minLat << " maxLon:" << maxLon << " maxLat:" << maxLat;
    qDebug() << "_grid_num:" << _grid_num << "_grid_len size is:" << _grid_len;
}

void PathPlan::updateChartElem()
{
    clearTargetObs();
    QList<int> point_keys = _chartelemPoint.keys();
    for (int i = 0; i < point_keys.count(); ++i)
    {
        Pos tobsinfo;
        auto fLon = _chartelemPoint[point_keys[i]].tPosInfo.fLon;
        auto fLat = _chartelemPoint[point_keys[i]].tPosInfo.fLat;
        tobsinfo.x = fLon;
        tobsinfo.y = fLat;

        addTargetObs(tobsinfo);
    }
    QList<int> line_keys = _chartelemLine.keys();
    for (int i = 0; i < line_keys.count(); ++i)
    {
        int lineId = line_keys[i];
        auto lineInfo = _chartelemLine[lineId].tLineInfo;

        for (std::size_t j = 0; j < lineInfo.size(); ++j)
        {
            auto line = lineInfo[j];
            double dis = getDis(line.fStartLon, line.fStartLat, line.fEndLon, line.fEndLat);
            int ncount = (dis / _grid_len) + 1 + 1; // +1 为了保证最后一个点也能被加入,防止边界条件
            double dlon = -(line.fStartLon - line.fEndLon) / ncount;
            double dlat = -(line.fStartLat - line.fEndLat) / ncount;
            for (int count = 0; count <= ncount; ++count)
            {
                Pos tobsinfo;
                auto fLon = line.fStartLon + dlon * count;
                auto fLat = line.fStartLat + dlat * count;
                tobsinfo.x = fLon;
                tobsinfo.y = fLat;
                addTargetObs(tobsinfo);
            }
        }
    }
}

GridIndex PathPlan::getGridIndex(double x, double y)
{
    int x_index = x / _grid_len;
    int y_index = y / _grid_len;
    x_index = std::min(x_index, _grid_num - 1);
    x_index = std::max(x_index, 0);
    y_index = std::min(y_index, _grid_num - 1);
    y_index = std::max(y_index, 0);

    return {y_index, x_index};
}

void PathPlan::genGrid(int extend_num)
{

    for (int i = 0; i < _grid_num; i++)
    {
        std::vector<Grid> row(_grid_num);
        for (int j = 0; j < _grid_num; j++)
        {
            Grid grid;
            grid.index = {i, j};
            grid.type = 0;
            grid.isWalkable = true;
            grid.tl.x = _minLon + j * _grid_len;
            grid.tl.y = _minLat + i * _grid_len;
            grid.tr.x = _minLon + (j + 1) * _grid_len;
            grid.tr.y = _minLat + i * _grid_len;
            grid.bl.x = _minLon + j * _grid_len;
            grid.bl.y = _minLat + (i + 1) * _grid_len;
            grid.br.x = _minLon + (j + 1) * _grid_len;
            grid.br.y = _minLat + (i + 1) * _grid_len;
            grid.center.x = _minLon + (j + 0.5) * _grid_len;
            grid.center.y = _minLat + (i + 0.5) * _grid_len;
            grid.gScore = std::numeric_limits<int>::max();
            grid.parent = {-1, -1};
            row[j] = grid;
        }
        _gridMap.push_back(row);
    }

    for (std::size_t i = 0; i < _targetObs.size(); ++i)
    {
        auto &obs = _targetObs[i];
        auto index = getGridIndex(obs.x - _minLon, obs.y - _minLat);
        grid(index).isWalkable = false;

        // 由地图边界向内外两侧扩展
        for (int i = index.x - extend_num; i <= index.x + extend_num; ++i)
        {
            for (int j = index.y - extend_num; j <= index.y + extend_num; ++j)
            {
                if (i < 0 || i >= _grid_num || j < 0 || j >= _grid_num)
                {
                    continue;
                }
                _gridMap[i][j].isWalkable = false;
            }
        }
    }
}

std::vector<std::vector<Grid>> PathPlan::getGridMap()
{
    return _gridMap;
}

void PathPlan::getDistanceAngle(double lat1 /*deg*/, double lon1 /*deg*/, double lat2 /*deg*/, double lon2 /*deg*/, double &d /*nm*/, double &angle /*deg*/)
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

bool PathPlan::canCross(GridIndex start, GridIndex end)
{
    std::vector<GridIndex> cross_grid = getCrossGrid(start, end);
    for (auto &v : cross_grid)
    {
        if (grid(v).isWalkable == false)
        {
            return false;
        }
    }
    return true;
}

// 保守策略, 优化的路径段在遇到第一个不能连通的点就停止
std::vector<Grid> PathPlan::optimizePathCons()
{
    if (_path.size() <= 2)
    {
        return _path;
    }

    std::vector<Grid> newPath;
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
            if (canCross(cur.index, next.index))
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
std::vector<Grid> PathPlan::optimizePathRad()
{
    if (_path.size() <= 2)
    {
        return _path;
    }

    std::vector<Grid> newPath;
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
            if (canCross(cur.index, next.index))
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

// 布雷森汉姆直线演算法（英语：Bresenham's line algorithm）
std::vector<GridIndex> PathPlan::getCrossGrid(GridIndex start, GridIndex end, int extend_num)
{
    std::vector<GridIndex> crossedGrids;

    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int sx = (start.x < end.x) ? 1 : -1;
    int sy = (start.y < end.y) ? 1 : -1;
    int err = dx - dy;

    int x = start.x;
    int y = start.y;

    while (true)
    {

        // crossedGrids.push_back({x, y});
        for (int i = x - extend_num; i <= x + extend_num; ++i)
        {
            for (int j = y - extend_num; j <= y + extend_num; ++j)
            {
                if (i < 0 || i >= _grid_num || j < 0 || j >= _grid_num)
                {
                    continue;
                }
                crossedGrids.push_back({i, j});
            }
        }

        if (x == end.x && y == end.y)
        {
            break;
        }

        int err2 = 2 * err;

        if (err2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (err2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return crossedGrids;
}
