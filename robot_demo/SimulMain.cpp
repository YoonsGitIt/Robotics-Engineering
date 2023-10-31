//
// Created by tina on 23. 10. 31.
//
#include <robot_simulation/SimulMain.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartSimulation();

    w.show();
    return a.exec();
}

