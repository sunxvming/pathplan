#include "Widget.h"
#include <QDebug>
#include <QFileDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <cassert>
#include "math.h"
#include "ChartObjectLoad.h"
#include "RenderArea.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    _renderArea = new RenderArea(this);

    // ======= line1 =======
    _btn1 = new QPushButton("load chart file");
    QHBoxLayout *row_btn = new QHBoxLayout;
    row_btn->addWidget(_btn1);

    //======= line2 ========
    double att = 10000;
    double rep = 100;
    double step = 0.03;
    double turnAngle = 45;
    double emCPA = 0.1;

    l_att = new QLineEdit(QString::number(att));
    l_rep = new QLineEdit(QString::number(rep));
    l_step = new QLineEdit(QString::number(step));
    l_turnAngle = new QLineEdit(QString::number(turnAngle));
    l_emCPA = new QLineEdit(QString::number(emCPA));

    QLabel *lab1 = new QLabel("Attraction:");
    QLabel *lab2 = new QLabel("Repulsion:");
    QLabel *lab3 = new QLabel("Step:");
    QLabel *lab4 = new QLabel("Max turn angle:");
    QLabel *lab5 = new QLabel("Avoidance range:");

    QHBoxLayout *row_attr = new QHBoxLayout;
    row_attr->addWidget(lab1);
    row_attr->addWidget(l_att);
    row_attr->addWidget(lab2);
    row_attr->addWidget(l_rep);
    row_attr->addWidget(lab3);
    row_attr->addWidget(l_step);
    row_attr->addWidget(lab4);
    row_attr->addWidget(l_turnAngle);
    row_attr->addWidget(lab5);
    row_attr->addWidget(l_emCPA);

    // ======= line3 =======
    QLabel *lab_set = new QLabel("<b>display options:</b>");
    QHBoxLayout *row_display_set = new QHBoxLayout;
    _checkbox_supplement_point = new QCheckBox("Points supplement");
    _checkbox_rawpath = new QCheckBox("Raw path");
    _checkbox_cut_circle_path = new QCheckBox("Cut circle path");
    _checkbox_opt_path = new QCheckBox("Optimize path");
    _checkbox_rawpath->setChecked(true);
    _checkbox_cut_circle_path->setChecked(true);
    _checkbox_opt_path->setChecked(true);
    _checkbox_supplement_point->setChecked(true);

    row_display_set->addWidget(lab_set);
    row_display_set->addWidget(_checkbox_rawpath);
    row_display_set->addWidget(_checkbox_cut_circle_path);
    row_display_set->addWidget(_checkbox_opt_path);
    row_display_set->addWidget(_checkbox_supplement_point);

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->addWidget(_renderArea, 0, 0);
    mainLayout->addLayout(row_btn, 1, 0);
    mainLayout->addLayout(row_attr, 2, 0);
    mainLayout->addLayout(row_display_set, 3, 0);
    setLayout(mainLayout);

    connect(_btn1, SIGNAL(clicked()), this, SLOT(chooseChartDataFile()));
    connect(_checkbox_rawpath, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_cut_circle_path, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_opt_path, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_supplement_point, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
}

void Widget::onCheckBoxStateChanged()
{
    _renderArea->update();
}

void Widget::chooseChartDataFile()
{
    QString filepath = QFileDialog::getOpenFileName(this, "chartFile", "/root/chartdata");
    qDebug() << " filepath :" << filepath << endl;
    ChartObjectLoad::loadChartElem(filepath, _chartelem_point, _chartelem_line);
    _renderArea->setMap(_chartelem_point, _chartelem_line);
    _renderArea->update();
}

Widget::~Widget()
{
}
