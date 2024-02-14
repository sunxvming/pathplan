#include "Utils.h"

#include <QMessageBox>

void Utils::showMsg(const QString &title, const QString &text)
{
    QMessageBox messageBox;
    messageBox.setWindowTitle(title);
    messageBox.setText(text);
    messageBox.setIcon(QMessageBox::Information);
    messageBox.addButton(QMessageBox::Ok);
    messageBox.exec();
}