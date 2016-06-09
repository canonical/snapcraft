#include <QApplication>
#include <QPushButton>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);


    QPushButton hello(QPushButton::tr("Hello world!"));
    hello.resize(500, 500);

    hello.show();
    return app.exec();
}
