#ifndef UTILS_H
#define UTILS_H

#include <QString>

class Utils
{
    Utils() = default;
    ~Utils() = default;

public:
    static void showMsg(const QString &title, const QString &text);
};

#endif // UTILS_H
