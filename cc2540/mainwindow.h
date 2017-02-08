#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QSerialPort *serialPort;
    void switchFxn(int step);
    void calcRps(QByteArray data);
    QTimer *timer = new QTimer(this);

private:
    Ui::MainWindow *ui;
public slots:
    void handleReadyRead();
    void readChar();


};

#endif // MAINWINDOW_H
