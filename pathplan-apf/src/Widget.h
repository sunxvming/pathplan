#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "PathPlan.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class Widget;
}
QT_END_NAMESPACE

class RenderArea;
class QPushButton;
class QLineEdit;
class QCheckBox;

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = NULL);
    ~Widget();

    QLineEdit *l_att;
    QLineEdit *l_rep;
    QLineEdit *l_step;
    QLineEdit *l_turnAngle;
    QLineEdit *l_emCPA;

    QCheckBox *_checkbox_rawpath;
    QCheckBox *_checkbox_cut_circle_path;
    QCheckBox *_checkbox_opt_path;
    QCheckBox *_checkbox_supplement_point;

private slots:
    void chooseChartDataFile();
    void onCheckBoxStateChanged();

private:
    RenderArea *_renderArea;
    QPushButton *_btn1;
    QHash<int, tagChartPointInfo> _chartelem_point;
    QHash<int, tagChartLineInfo> _chartelem_line;
};
#endif // WIDGET_H
