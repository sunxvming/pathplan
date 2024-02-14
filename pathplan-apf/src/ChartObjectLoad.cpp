#include "ChartObjectLoad.h"
#include <qfile.h>
#include <QIODevice>
#include <QTextStream>
#include <QStringList>
#include <QDebug>
#include <QDir>

ChartObjectLoad::ChartObjectLoad()
{
}

void ChartObjectLoad::loadChartElem(QString filename, QHash<int, tagChartPointInfo> &chartpointHash, QHash<int, tagChartLineInfo> &chartlineHash)
{
    // 打开海图数据文件
    QFile file;
    file.setFileName(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "filepath = " << filename;
        qDebug() << "chartdata open fail!";
        return;
    }
    // 读取每行属性内容
    QTextStream stream(&file);
    while (!stream.atEnd())
    {
        QString line = stream.readLine();
        QStringList strlist = line.split(",");
        if (strlist.size() < 6)
            continue;
        int flag = strlist[0].toUInt();
        if (20 == flag)
        {
            tagChartPointInfo chartpoint;
            chartpoint.nType = strlist[1].toInt();
            chartpoint.nID = strlist[2].toInt();
            chartpoint.fRangTOL = strlist[3].toFloat();
            chartpoint.tPosInfo.fLon = strlist[4].toDouble();
            chartpoint.tPosInfo.fLat = strlist[5].toDouble();
            chartpointHash[chartpoint.nID] = chartpoint;
        }
        else if (21 == flag)
        {
            tagChartLineInfo chartline;
            chartline.nType = strlist[1].toInt();
            chartline.nID = strlist[2].toInt();
            chartline.fRangTOL = strlist[3].toFloat();
            int size = strlist[4].toInt();
            float radius = strlist[5].toFloat();
            double realsize = (strlist.size() - 6) / 4;
            if (size > realsize)
                size = realsize;
            if (0 == size || 6 == strlist.size())
                continue;
            for (int i = 0; i < size; ++i)
            {
                tagLineInfo lineinfo;
                lineinfo.fStartLon = strlist[6 + i * 4].toDouble();
                lineinfo.fStartLat = strlist[7 + i * 4].toDouble();
                lineinfo.fEndLon = strlist[8 + i * 4].toDouble();
                lineinfo.fEndLat = strlist[9 + i * 4].toDouble();
                lineinfo.fLineObsRadius = radius;
                chartline.tLineInfo.push_back(lineinfo);
            }
            chartlineHash[chartline.nID] = chartline;
        }
    }
    file.close();
}
