#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("Plane Game");

    pvestart.setParent(this);
    pvestart.setText("PVE");
    pvestart.move(120,90);
    pvestart.resize(100,60);

}

MainWindow::~MainWindow()
{
    delete ui;
}
