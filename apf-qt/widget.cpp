#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QtCore/qmath.h>
#include "apf.h"
#include <iostream>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    drawPath();

}

Widget::~Widget()
{
    delete ui;
}


void Widget::drawPath()
{
    Point start = {0, 0};
    Point goal = {6, 6};
    Point Obstacle[] = {{5,5}, {3,4}, {3,2.5} ,{4,4.5} };

//    Point Obstacle[] = {{2,2}, {3,2.5}, {4,4.5}, {3,6}, {6,2}, {5,5.5}, {9,9}};

    ArtificialPotentialField path_planner(1.0, 10.0, 1.0, 0.1);
    path_planner.setStartPosition(start.x, start.y);
    path_planner.setGoalPosition(goal.x, goal.y);

    //=============绘制起点=====================
    QVector<double> start_x,start_y;
    start_x << start.x;
    start_y << start.y;

    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(0)->setData(start_x,start_y);
    ui->qCustomPlot->graph(0)->setPen(QPen(Qt::darkGreen));
    ui->qCustomPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(0)->setName("Start");


    //================绘制障碍物点=================
    QVector<double> obs_x;
    QVector<double> obs_y;//定义坐标
    for(auto& pos:Obstacle){
        obs_x << pos.x;
        obs_y << pos.y;
        path_planner.addObstacle(pos.x, pos.y);
    }

    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(1)->setData(obs_x,obs_y);
    ui->qCustomPlot->graph(1)->setPen(QPen(Qt::red));
    ui->qCustomPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(1)->setName("Obstacle");


    //==================绘制目标点=================
    QVector<double> goal_x,goal_y;
    goal_x<<goal.x;
    goal_y<<goal.y;
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(2)->setData(goal_x,goal_y);
    ui->qCustomPlot->graph(2)->setPen(QPen(Qt::blue));
    ui->qCustomPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(2)->setName("Goal");

    //开启图例显示
    ui->qCustomPlot->legend->setVisible(true);//true false
    ui->qCustomPlot->legend->setFont(QFont("Helvetica", 9));

    //重设图例位置为左上
    ui->qCustomPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);


    //================绘制路径===================
    //定义两个可变数组存放绘图的坐标数据
    std::vector<Point> path = path_planner.planPath();

    QVector<double> path_x,path_y;
    for (const auto& point : path) {
        path_x << point.x;
        path_y << point.y;
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;

    }
    // create empty curve objects and add them to customPlot:
    QCPCurve *road = new QCPCurve(ui->qCustomPlot->xAxis, ui->qCustomPlot->yAxis);
    ui->qCustomPlot->addPlottable(road);

    // pass the data to the curves:
    road->setData(path_x, path_y);
    // color the curves:
    road->setPen(QPen(Qt::black));
    road->setName ("Road");


//    //==================绘制路径================
//    ui->qCustomPlot->addGraph();

//    //添加数据
//    ui->qCustomPlot->graph(3)->setData(path_x,path_y);
//    ui->qCustomPlot->graph(3)->setPen(QPen(Qt::black));
//    ui->qCustomPlot->graph(3)->setLineStyle(QCPGraph::lsLine );
//    ui->qCustomPlot->graph(3)->setScatterStyle(QCPScatterStyle::ssNone);
//    ui->qCustomPlot->graph(3)->setName("Road");

    //设置坐标轴标签名称
    ui->qCustomPlot->xAxis->setLabel("x");
    ui->qCustomPlot->yAxis->setLabel("y");

    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->qCustomPlot->xAxis->setRange(0,10);
    ui->qCustomPlot->yAxis->setRange(0,10);

    //关闭自动刻度步长
    ui->qCustomPlot->xAxis->setAutoTickStep(false);
    ui->qCustomPlot->yAxis->setAutoTickStep(false);
    //关闭自动子刻度
    ui->qCustomPlot->xAxis->setAutoSubTicks(false);
    ui->qCustomPlot->yAxis->setAutoSubTicks(false);
    //设置主刻度步长1
    ui->qCustomPlot->xAxis->setTickStep (1);
    ui->qCustomPlot->yAxis->setTickStep (1);
    //设置子刻度数4，即分成5份
    ui->qCustomPlot->xAxis->setSubTickCount (4);
    ui->qCustomPlot->yAxis->setSubTickCount (4);
    //设置刻度长度5
    ui->qCustomPlot->xAxis->setTickLength (5, 0);
    ui->qCustomPlot->yAxis->setTickLength (5, 0);

}
