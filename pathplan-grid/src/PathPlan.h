//========================================================================
// Name: PathPlan.h
// Description: Grid-based map representation and path planning using the A* algorithm
// Author: sunxvming@gmail.com
// Modified: 2023.10.19
//========================================================================

#ifndef PATHPLAN_H
#define PATHPLAN_H


#include "globalstruct.h"
#include <QHash>
#include <cmath>

#define M_PI       3.14159265358979323846   // pi
const double EARTH_RADIUS = 3440.06479;

struct Pos
{
    double x;
    double y;
};

struct GridIndex
{
    int x;
    int y;
    bool operator==(const GridIndex &other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const GridIndex &other) const
    {
        return !(*this == other);
    }
};

struct GridIndexHash
{
    std::size_t operator()(const GridIndex &index) const
    {
        std::size_t hash_x = std::hash<int>{}(index.x);
        std::size_t hash_y = std::hash<int>{}(index.y);
        return hash_x ^ (hash_y << 1); // 移动一位是为了防止 {1,2}和{2,1}的哈希值相同
    }
};

struct Grid
{
    GridIndex index;  // 格子的索引
    int type;         // 调试用， 1：两点连线穿过的格子  2：算法探索过的格子
    bool isWalkable;  // 是否可行走
    Pos tl;           // 左上
    Pos tr;           // 右上
    Pos bl;           // 左下
    Pos br;           // 右下
    Pos center;       // 格子中心点，绝对坐标
    GridIndex parent; // 父节点，用于回溯路径
    int gScore;       // gScore
    int hScore;       // hScore
};

struct Neighbor
{
    GridIndex index;
    bool isDiagonal;
};

enum class CheckPathResult
{
    Valid,
    StartPointInvalid,
    EndPointInvalid,
    StartEndPointInvalid,
    Unreachable
};

class PathPlan
{

public:
    PathPlan() = default;
    ~PathPlan() = default;

    void setChart(QHash<int, tagChartPointInfo> chartpointHash, QHash<int, tagChartLineInfo> chartlineHash, int grid_num, int extend_num = 1);
    void setGridNum(int grid_num) { _grid_num = grid_num; }
    void updateChartElem();
    std::vector<Grid> findPath(Pos start, Pos end);
    CheckPathResult checkPath(Pos start, Pos end);

    void setCrossGrid(Pos start, Pos end);

    std::vector<Pos> getObs() { return _targetObs; }
    std::vector<std::vector<Grid>> getGridMap();
    std::vector<Grid> optimizePathCons();
    std::vector<Grid> optimizePathRad();

private:
    Grid &grid(GridIndex index);
    void setExplored(GridIndex index);
    int calcHScore(GridIndex start, GridIndex end);
    void addTargetObs(Pos pos) { _targetObs.push_back(pos); }
    void clearTargetObs() { _targetObs.clear(); }
    void calcGrid();
    void genGrid(int genGrid);
    bool canCross(GridIndex start, GridIndex end);
    std::vector<GridIndex> getCrossGrid(GridIndex start, GridIndex end, int extend_num = 0);
    GridIndex getGridIndex(double x, double y);

    static double A2R(double d) { return d * M_PI / 180.0; }
    static double R2A(double d) { return d / M_PI * 180.0; }
    static void getDistanceAngle(double lat1 /*deg*/, double lon1 /*deg*/, double lat2 /*deg*/, double lon2 /*deg*/, double &dbDist /*nm*/, double &dbCrs /*deg*/);

    static double getDis(double x1, double y1, double x2, double y2)
    {
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

private:
    QHash<int, tagChartPointInfo> _chartelemPoint;
    QHash<int, tagChartLineInfo> _chartelemLine;

    std::vector<Pos> _targetObs;

    double _minLat = 0;
    double _maxLat = 0;
    double _minLon = 0;
    double _maxLon = 0;

    std::vector<std::vector<Grid>> _gridMap;
    int _grid_num = 10;
    double _grid_len = 0;

    std::vector<Grid> _path;
};

#endif // PATHPLAN_H
