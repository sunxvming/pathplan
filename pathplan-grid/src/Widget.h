#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class Widget;
}
QT_END_NAMESPACE

class RenderArea;
class QPushButton;
class QLineEdit;
class QRadioButton;
class QComboBox;
class QCheckBox;

#include "globalstruct.h"

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = NULL);
    ~Widget() = default;

    QLineEdit *_line_edit_grid_num;
    QLineEdit *_line_edit_extend_num;

    QCheckBox *_checkbox_grid;
    QCheckBox *_checkbox_rawpath;
    QCheckBox *_checkbox_opt_path_rad;
    QCheckBox *_checkbox_opt_path_cons;
    QCheckBox *_checkbox_supplement_point;

private slots:
    void chooseChartDataFile();
    void onCheckBoxStateChanged();

private:
    RenderArea *_renderArea;
    QPushButton *btn1;
    QPushButton *btn2;
    QHash<int, tagChartPointInfo> _chartelemPoint;
    QHash<int, tagChartLineInfo> _chartelemLine;
};
#endif // WIDGET_H
