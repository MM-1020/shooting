#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    showFullScreen();

private:
    Ui::MainWindow *ui;
    QPushButton pvestart;
    QPushButton pvpstart;
    QPushButton close;
};

#endif // MAINWINDOW_H
