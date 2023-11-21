#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "serialporthandler.h"
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void showImg(int idx);

private slots:
    void on_pushButton_connectCom_clicked();
    void on_pushButton_refreshComList_clicked();

    void on_pushButton_test_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort m_serialPort;
    SerialPortHandler m_serialPortReader;
    QTimer m_timer;

public slots:
    void handleLidarData(int dist, int amp);
    void slotSwitchImg();
};
#endif // MAINWINDOW_H
