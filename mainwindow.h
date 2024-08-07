#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "serialporthandler.h"
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QStandardItemModel>

QT_CHARTS_USE_NAMESPACE

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
    void on_comboBox_mode_activated(int index);

    void on_pushButton_record_clicked();

    void on_pushButton_openDataDir_clicked();

    void on_pushButton_readFirmwareVersion_clicked();

    void on_checkBox_rangeEnable_stateChanged(int arg1);

    void on_pushButton_saveConfig_clicked();

    void on_pushButton_softReset_clicked();

    void on_pushButton_hardlineConfig_clicked();

    void on_pushButton_sampleRate_clicked();

    void on_pushButton_ldTrigPwidth_clicked();

    void on_pushButton_footDetectParaConfig_clicked();

    void on_pushButton_readConfig_clicked();

    void on_pushButton_walkErrK_clicked();

    void on_pushButton_distOffset_clicked();

    void on_pushButton_eraseFlash_clicked();

    void on_pushButton_dirtyDistTh_clicked();

    void on_pushButton_lowPeakTh_clicked();

    void on_pushButton_ldTrigNum_clicked();

    void on_pushButton_bvdCalib_clicked();

    void on_pushButton_readReg_clicked();

    void on_pushButton_writeReg_clicked();

    void on_checkBox_ledEnable_stateChanged(int arg1);

    void on_pushButton_connectComLin_clicked();

    void on_pushButton_btRssiSet_clicked();

    void on_checkBox_btTestMode_stateChanged(int arg1);

    void on_comboBox_jtxWorkMode_activated(int index);

    void on_pushButton_xtalkCalib_clicked();

    void on_pushButton_offsetCalib_clicked();

    void on_pushButton_ledConfig_clicked();

    void on_pushButton_writeSn_clicked();

    void on_checkBox_jumpRecord_stateChanged(int arg1);

    void on_pushButton_rebootBt_clicked();

    void on_pushButton_factoryResetBt_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort m_serialPort;
    SerialPortHandler m_serialPortReader;
    QTimer m_timer;
    QTimer m_timerJumpRecord;

    QLineSeries *m_lineSeries;
    QChart      *m_chart;
    QValueAxis  *m_axisX;
    QValueAxis  *m_axisY;

    QStandardItemModel *m_tableModel;

    uint8_t mParaBuf[1024];
    SysConfigStruct mDevConfigStruct;

    int mDistBuf[1024];
    int mDistFrameCnt;

    void dispDeviceConfig();

public slots:
    void slotHandleLidarData(QByteArray frameData);
    void slotSwitchImg();
    void slotTimerJumpRecord();
    void slotSetAxisRange(int xmin, int xmax, int ymin, int ymax);
    void slotRecordStop();
    void slotProcDist(int mm);
};
#endif // MAINWINDOW_H
