#include <QString>
#include <iostream>

int main()
{
    QString s("hello world");
    std::cout << s.toUtf8().constData() << std::endl;
}
