#include "widget.h"
#include <QApplication>
//#include "ARF_C++/arf.h"
#include <QDebug>
//#include <iostream>
//using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;

    w.show();

    return a.exec();
}
