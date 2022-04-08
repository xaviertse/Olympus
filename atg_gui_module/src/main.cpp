#include "./include/atg_gui/atg_window.hpp"
#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    atg_gui::ATG_Window w(argc,argv);
    //QTimer::singleShot(5000, &w, SLOT(show()));
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    return app.exec();
}
