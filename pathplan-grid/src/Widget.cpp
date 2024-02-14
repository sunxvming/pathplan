
#include "Widget.h"

#include "math.h"
#include <cassert>

#include <QDebug>
#include <QFileDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QRadioButton>
#include <QComboBox>
#include <QCheckBox>

#include "ChartObjectLoad.h"
#include "RenderArea.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    _renderArea = new RenderArea(this);

    btn1 = new QPushButton("Load chart file");

    QHBoxLayout *hlayout = new QHBoxLayout;
    hlayout->addWidget(btn1);

    // 格子数量输入框
    int grid_num = 100;
    QLabel *lab1 = new QLabel("<b>grid num:</b>");
    _line_edit_grid_num = new QLineEdit(QString::number(grid_num));
    QLabel *lab2 = new QLabel("<b>extend num:</b>");
    _line_edit_extend_num = new QLineEdit(QString::number(1));

    QHBoxLayout *layout2 = new QHBoxLayout;
    QHBoxLayout *layout3 = new QHBoxLayout;
    QHBoxLayout *layout4 = new QHBoxLayout;
    layout2->addWidget(lab1);
    layout2->addWidget(_line_edit_grid_num);
    layout2->addWidget(lab2);
    layout2->addWidget(_line_edit_extend_num);

    // 添加单选按钮
    QLabel *lab_set = new QLabel("<b>display options:</b>");

    _checkbox_grid = new QCheckBox("Grid");
    _checkbox_rawpath = new QCheckBox("Raw path");
    _checkbox_opt_path_cons = new QCheckBox("Optimize path(conservative strategy)");
    _checkbox_opt_path_rad = new QCheckBox("Optimize path(radicalness strategy)");
    _checkbox_supplement_point = new QCheckBox("Points supplement");

    _checkbox_grid->setChecked(true);
    _checkbox_rawpath->setChecked(true);
    _checkbox_opt_path_rad->setChecked(true);
    _checkbox_opt_path_cons->setChecked(false);
    _checkbox_supplement_point->setChecked(true);

    layout3->addWidget(lab_set);
    layout3->addWidget(_checkbox_supplement_point);
    layout3->addWidget(_checkbox_grid);
    layout3->addWidget(_checkbox_rawpath);
    layout4->addWidget(_checkbox_opt_path_rad);
    layout4->addWidget(_checkbox_opt_path_cons);

    QGridLayout *mainLayout = new QGridLayout;

    mainLayout->addWidget(_renderArea, 0, 0);
    mainLayout->addLayout(hlayout, 1, 0);
    mainLayout->addLayout(layout2, 2, 0);
    mainLayout->addLayout(layout3, 3, 0);
    mainLayout->addLayout(layout4, 4, 0);
    setLayout(mainLayout);

    connect(btn1, SIGNAL(clicked()), this, SLOT(chooseChartDataFile()));
    connect(_checkbox_grid, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_supplement_point, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_rawpath, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_opt_path_rad, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
    connect(_checkbox_opt_path_cons, &QCheckBox::stateChanged, this, &Widget::onCheckBoxStateChanged);
}

void Widget::chooseChartDataFile()
{
    QString filepath = QFileDialog::getOpenFileName(this, "chartFile", ".");
    qDebug() << " filepath :" << filepath << endl;
    ChartObjectLoad::loadChartElem(filepath, _chartelemPoint, _chartelemLine);
    _renderArea->setMap(_chartelemPoint, _chartelemLine);
    _renderArea->update();
}

void Widget::onCheckBoxStateChanged()
{
    _renderArea->update();
}