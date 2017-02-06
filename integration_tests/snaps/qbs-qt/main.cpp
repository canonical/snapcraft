#include <QCoreApplication>
#include <QTextStream>

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);
#ifdef PRINT_STRING
    out << QByteArrayLiteral("Hello Snapcraft World");
#else
    out << QByteArrayLiteral("Wrong snapcraft string");
#endif
    return 0;
}
