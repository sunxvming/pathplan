// artificial_potential_field.h

#ifndef ARTIFICIAL_POTENTIAL_FIELD_H
#define ARTIFICIAL_POTENTIAL_FIELD_H

#include <vector>
#include <cmath>


struct Point {
    double x;
    double y;
};



class ArtificialPotentialField {
public:
    ArtificialPotentialField(double attract_gain, double repulse_gain, double threshold, double step_length);

    // 设置机器人初始位置
    void setStartPosition(double x, double y);

    // 设置目标点位置
    void setGoalPosition(double x, double y);

    // 添加障碍物位置
    void addObstacle(double x, double y);

    // 执行路径规划并返回路径
    std::vector<Point> planPath();

private:
    // 计算两点之间的距离
    double distance(double x1, double y1, double x2, double y2);

    // 计算吸引力
    void computeAttract(double &Fx, double &Fy);

    // 计算斥力
    void computeRepulsion(double &Fx, double &Fy);

    // 更新机器人位置
    void updatePosition(double Fx, double Fy);

    Point current_position_;  // 机器人当前位置
    Point goal_;  // 目标点位置
    std::vector<Point> obstacles_;  // 障碍物位置
    double attract_gain_;  // 吸引力增益
    double repulse_gain_;  // 斥力增益
    double threshold_;  // 障碍物斥力影响距离
    double step_length_;  // 步长
    int max_iterations_ = 100;  // 迭代次数，避免无限循环
};

#endif // ARTIFICIAL_POTENTIAL_FIELD_H
