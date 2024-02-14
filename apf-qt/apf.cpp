// artificial_potential_field.cpp

#include "apf.h"
#include <iostream>

ArtificialPotentialField::ArtificialPotentialField(double attract_gain, double repulse_gain, double threshold, double step_length)
    : attract_gain_(attract_gain), repulse_gain_(repulse_gain), threshold_(threshold), step_length_(step_length) {}

void ArtificialPotentialField::setStartPosition(double x, double y) {
    current_position_.x = x;
    current_position_.y = y;
}

void ArtificialPotentialField::setGoalPosition(double x, double y) {
    goal_.x = x;
    goal_.y = y;
}

void ArtificialPotentialField::addObstacle(double x, double y) {
    obstacles_.push_back({x, y});
}

double ArtificialPotentialField::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void ArtificialPotentialField::computeAttract(double &Fx, double &Fy) {
    // 计算距离
    double distance_to_goal = distance(current_position_.x, current_position_.y, goal_.x, goal_.y);

    // 计算吸引力大小
    double force_magnitude = attract_gain_ * distance_to_goal;

    // 计算吸引力的方向向量
    double Fxunit = (goal_.x - current_position_.x) / distance_to_goal;
    double Fyunit = (goal_.y - current_position_.y) / distance_to_goal;
    Fx = force_magnitude * Fxunit;
    Fy = force_magnitude * Fyunit;
}

void ArtificialPotentialField::computeRepulsion(double &Fx, double &Fy) {
    Fx = 0;
    Fy = 0;

    for (const auto& obstacle : obstacles_) {
        // 计算机器人与障碍物之间的距离
        double distance_to_obstacle = distance(current_position_.x, current_position_.y, obstacle.x, obstacle.y);

        // 如果距离小于阈值，则计算斥力
        if (distance_to_obstacle < threshold_) {
            // 计算斥力大小
            double force_magnitude = repulse_gain_ * (1 / distance_to_obstacle - 1 / threshold_) / (distance_to_obstacle * distance_to_obstacle);

//            double force_magnitude = repulse_gain_ * exp(-distance_to_obstacle * distance_to_obstacle / (2 * threshold_ * threshold_));


            // 计算斥力的方向向量
            Fx += force_magnitude * (current_position_.x - obstacle.x) / distance_to_obstacle;
            Fy += force_magnitude * (current_position_.y - obstacle.y) / distance_to_obstacle;
        }
    }

}


void ArtificialPotentialField::updatePosition(double Fx, double Fy) {
    // 更新机器人位置
    current_position_.x += step_length_ * Fx;
    current_position_.y += step_length_ * Fy;
}

std::vector<Point> ArtificialPotentialField::planPath() {
    std::vector<Point> path;
    path.push_back(current_position_);

    for (int i = 0; i < max_iterations_; ++i) {
        // 计算吸引力
        double Fx_attract, Fy_attract;
        computeAttract(Fx_attract, Fy_attract);

        // 计算斥力
        double Fx_repulse, Fy_repulse;
        computeRepulsion(Fx_repulse, Fy_repulse);

        // 计算合力
        double Fx_total = Fx_attract + Fx_repulse;
        double Fy_total = Fy_attract + Fy_repulse;

        // 更新机器人位置
        updatePosition(Fx_total, Fy_total);

        // 输出机器人位置
        path.push_back(current_position_);

        // 判断是否到达目标点
        if (distance(current_position_.x, current_position_.y, goal_.x, goal_.y) < 0.1) {
            std::cout << "Goal reached!" << std::endl;
            break;
        }
    }

    return path;
}
