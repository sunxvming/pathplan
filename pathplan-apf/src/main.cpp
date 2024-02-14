#include "Widget.h"

#include <QApplication>
#include <QTextStream>
#include <QDesktopWidget>
#include <QDebug>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifdef _WIN32
void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    (void)type;
    (void)context;
    (void)msg;
    QByteArray localMsg = msg.toLocal8Bit();
    QTextStream out(stdout);
    out << localMsg.constData() << endl;
}

bool NewConsole()
{
    ::AllocConsole();
    qInstallMessageHandler(myMessageOutput); // 设置自定义消息处理程序，不然qDebug()无效
    return true;
}
#endif

int main(int argc, char *argv[])
{

#ifdef _WIN32
    if (AttachConsole(ATTACH_PARENT_PROCESS) || NewConsole())
    {
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);
    }
#endif

    QApplication a(argc, argv);
    Widget w;
    w.resize(1000, 1000);

    // set default font size
    QFont defaultFont = a.font();
    defaultFont.setPointSize(12);
    a.setFont(defaultFont);

    QDesktopWidget desktop;
    int screen_width = desktop.screenGeometry().width();
    int screen_height = desktop.screenGeometry().height();

    // 计算窗口的 x 和 y 坐标，使其在屏幕中居中
    int window_width = w.width();
    int window_height = w.height();
    int x = (screen_width - window_width) / 2;
    int y = (screen_height - window_height) / 2;

    w.move(x, y);
    w.show();
    return a.exec();
}
