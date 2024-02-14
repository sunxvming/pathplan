#ifndef CHARTOBJECTLOAD_H
#define CHARTOBJECTLOAD_H

#include "qstring.h"
#include <QHash>
#include "globalstruct.h"

class ChartObjectLoad
{
public:
    ChartObjectLoad();
    static void loadChartElem(QString filename, QHash<int, tagChartPointInfo> &_chartpointvec, QHash<int, tagChartLineInfo> &_chartlinevec);
};

#endif // CHARTOBJECTLOAD_H
