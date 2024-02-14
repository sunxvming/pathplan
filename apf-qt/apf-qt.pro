
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = QCustomPlot
TEMPLATE = app


SOURCES += main.cpp\
        widget.cpp \
        apf.cpp \
    qcustomplot.cpp

HEADERS  += widget.h apf.h \
    qcustomplot.h

FORMS    += widget.ui

