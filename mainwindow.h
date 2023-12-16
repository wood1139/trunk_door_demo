#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "serialporthandler.h"
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QThread>

QT_CHARTS_USE_NAMESPACE

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define DATA_WIN_SIZE 20

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

    QLineSeries *m_lineSeries;
    QChart      *m_chart;
    QValueAxis  *m_axisX;
    QValueAxis  *m_axisY;

public slots:
    void slotHandleLidarData(QByteArray frameData);
    void slotSwitchImg();
};
#endif // MAINWINDOW_H
