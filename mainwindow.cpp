#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDir>
#include <QDesktopServices>
#include <QUrl>
#include <QMessageBox>
#include <cstring>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_timer.setInterval(200);
    QObject::connect(&m_timer, &QTimer::timeout, this, &MainWindow::slotSwitchImg);
    QObject::connect(&m_serialPortReader, &SerialPortHandler::sigLidarData, this, &MainWindow::slotHandleLidarData);
    QObject::connect(&m_serialPortReader, &SerialPortHandler::sigSetAxisRange, this, &MainWindow::slotSetAxisRange);
    QObject::connect(&m_serialPortReader, &SerialPortHandler::sigRecordStop, this, &MainWindow::slotRecordStop);
    QObject::connect(&m_serialPortReader, &SerialPortHandler::sigProcDist, this, &MainWindow::slotProcDist);
    qDebug() << "app start!";

    showImg(0);

    m_lineSeries = new QLineSeries();
    for(int i=0; i<2048; i++)
    {
        m_lineSeries->append(i, 0);
    }

    m_chart = new QChart();
    m_chart->addSeries(m_lineSeries);
//    m_chart->createDefaultAxes();
    m_chart->setTitle("直方图");
    m_chart->legend()->setVisible(false);

    m_axisX = new QValueAxis;
    m_axisX->setTickCount(17);
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_lineSeries->attachAxis(m_axisX);

    m_axisY = new QValueAxis;
    m_axisY->setTickCount(11);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
    m_lineSeries->attachAxis(m_axisY);

    slotSetAxisRange(0,2048,0,100);
    ui->graphicsView->setChart(m_chart);

    m_tableModel = new QStandardItemModel(5,5,this);
    // 填充数据
    for (int row = 0; row < 5; ++row) {
        for (int column = 0; column < 5; ++column) {
            m_tableModel->setData(m_tableModel->index(row,column,QModelIndex()), QString::number(0));
        }
    }
   ui->tableView->setModel(m_tableModel);
   ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
   ui->tableView->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
   ui->tableView->verticalHeader()->setVisible(false);
   ui->tableView->horizontalHeader()->setVisible(false);

   m_serialPortReader.setDataPtr(m_lineSeries, m_tableModel);

   ui->comboBox_jtxWorkMode->addItem("Debug");
   ui->comboBox_jtxWorkMode->addItem("Standard");
   ui->comboBox_jtxWorkMode->addItem("L9");
   ui->comboBox_jtxWorkMode->addItem("BMW");

   ui->comboBox_mode->addItem("Range Mode");
   ui->comboBox_mode->addItem("Single Pixel Mode");
   ui->comboBox_mode->addItem("Histogram Mode");

   ui->comboBox_hardlinePinSel->addItem("None");
   ui->comboBox_hardlinePinSel->addItem("UART TX");
   ui->comboBox_hardlinePinSel->addItem("LED1");
   ui->comboBox_hardlinePinSel->addItem("LED2");

   ui->comboBox_hardlinePinMode->addItem("Open-Drain");
   ui->comboBox_hardlinePinMode->addItem("Push-Pull");

//   ui->lineEdit_setFramNum->setText("200");
//   ui->lineEdit_pulseNumList->setText("150 500 1000 1500");
//   ui->lineEdit_scanPulseNumListAtBvd->setText("235");
//   ui->lineEdit_bvdList->setText("225 227 229 231 235 237 239");
//   ui->lineEdit_scanBvdAtPulseNum->setText("500");

   QDir dir("data");
   if (!dir.exists())
   {
        dir.mkpath(".");
   }

   on_pushButton_refreshComList_clicked();

   mDistFrameCnt = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_connectCom_clicked()
{
    if(m_serialPortReader.isConnected())
    {
        m_serialPortReader.disconnectCom();
    }
    else
    {
        m_serialPortReader.connectCom(ui->comboBox_comList->currentText(), ui->lineEdit_baudrate->text().toInt());
    }

    if(m_serialPortReader.isConnected())
    {
        ui->pushButton_connectCom->setText("断开");
    }
    else
    {
        ui->pushButton_connectCom->setText("连接");
    }
}


void MainWindow::on_pushButton_connectComLin_clicked()
{
    if(m_serialPortReader.isConnectedLin())
    {
        m_serialPortReader.disconnectComLin();
    }
    else
    {
        m_serialPortReader.connectComLin(ui->comboBox_comListLin->currentText(), ui->lineEdit_baudrateLin->text().toInt());
    }

    if(m_serialPortReader.isConnectedLin())
    {
        ui->pushButton_connectComLin->setText("断开");
    }
    else
    {
        ui->pushButton_connectComLin->setText("连接");
    }
}

void MainWindow::on_pushButton_refreshComList_clicked()
{
    QStringList comNameList = m_serialPortReader.scanComList();
    ui->comboBox_comList->clear();
    for(int i=0; i<comNameList.size(); i++)
    {
        ui->comboBox_comList->addItem(comNameList[i]);
    }
    ui->comboBox_comListLin->clear();
    for(int i=0; i<comNameList.size(); i++)
    {
        ui->comboBox_comListLin->addItem(comNameList[i]);
    }
}
void MainWindow::slotSwitchImg()
{
    static int cnt = 0;
    if(cnt < 10)
    {
        cnt++;
        showImg(cnt % 2);
    }
    else
    {
        cnt = 0;
        m_timer.stop();
        showImg(2);
    }

}

void MainWindow::slotSetAxisRange(int xmin, int xmax, int ymin, int ymax)
{
    m_axisX->setRange(xmin, xmax);
    m_axisY->setRange(ymin, ymax);
}

void MainWindow::slotRecordStop()
{
    ui->pushButton_record->setText("开始录制");
}

void MainWindow::slotProcDist(int mm)
{
    int win_len = ui->lineEdit_distWinLen->text().toInt();
    if(win_len > 1204)
    {
        ui->lineEdit_distWinLen->setText("1024");
        win_len = 1024;
    }
    if(win_len <= 0)
    {
        ui->lineEdit_distWinLen->setText("100");
        win_len = 100;
    }
    mDistBuf[mDistFrameCnt++] = mm;
    if(mDistFrameCnt>=win_len)
    {
        int distMean = 0;
        int distSum = 0;
        int distSum1 = 0;
        int distStd = 0;
        mDistFrameCnt = 0;
        for(int i = 0; i < win_len; i++)
        {
            distSum += mDistBuf[i];
        }
        distMean = distSum / win_len;

        for(int i = 0; i < win_len; i++)
        {
            distSum1 += (mDistBuf[i]-distMean)*(mDistBuf[i]-distMean);
        }

        distStd = std::sqrt(distSum1/win_len);

        ui->label_distMean->setText(QString::number(distMean));
        ui->label_distStd->setText(QString::number(distStd));
    }


}

void MainWindow::showImg(int idx)
{
    QStringList imagePaths = {"pic/0.PNG",
                              "pic/1.PNG",
                              "pic/2.PNG"};
    if (idx < 3)
    {
        QPixmap pixmap(imagePaths[idx]);
        ui->label_pic->setPixmap(pixmap);
    }
}

void MainWindow::slotHandleLidarData(QByteArray frameData)
{
    static uint8_t paraPackId = 0;

    if(uint8_t(frameData[3])==DATA_ID_DETECT_STATUS)
    { // 脚踩信号
        m_timer.start();
        showImg(0);
    }
    else if(uint8_t(frameData[3])==ID_GET_VERSION)
    { // 版本号
        QString versionStr;
        for(int i=0; i<26; i++)
        {
            versionStr += char(frameData[4+i]);
        }
        ui->label_firmwareVersion->setText(versionStr);
    }
    else if(uint8_t(frameData[3])==ID_SAVE_SETTINGS)
    {
        QMessageBox messageBox;
        if(0 ==frameData[4])
        {
            messageBox.setText("保存成功");
        }
        else
        {
            messageBox.setText("保存失败");
        }
        messageBox.exec();
    }
    else if(uint8_t(frameData[3])==ID_READ_ALL_PARAMS)
    {
        uint8_t curId = frameData[4];
        if(paraPackId == curId)
        {
            
            for(int iByte=0; iByte<51; iByte++)
            {
                mParaBuf[paraPackId*51+iByte] = frameData[5+iByte];
            }
            paraPackId++;
            if(20 == paraPackId)
            {
                std::memcpy(&mDevConfigStruct, mParaBuf, sizeof(SysConfigStruct));
                dispDeviceConfig();
                paraPackId = 0;
            }
        }
        else
        {
            paraPackId = 0;
        }
    }
    else if(uint8_t(frameData[3])==ID_BVD_CALIB)
    {
        int bvd_val = (uint8_t)frameData[4];
        int bvd_temp = (int8_t)frameData[5];
        ui->label_bvdVal->setText("0x24F:"+QString::number(bvd_val));
        ui->label_bvdTemp->setText("temp:"+QString::number(bvd_temp));
    }
    else if(uint8_t(frameData[3])==ID_READ_REG)
    {
        int reg_addr = uint8_t(frameData[4]) + (uint8_t(frameData[5])<<8);
        int reg_val = (uint8_t)frameData[6];
        ui->lineEdit_regAddr->setText(QString::number(reg_addr, 16));
        ui->lineEdit_regVal->setText(QString::number(reg_val));
    }
    else if(uint8_t(frameData[3])==ID_XTALK_CALIB)
    {
        if(uint8_t(frameData[4]) == 0)
        {
            ui->label_xtalkCalibRes->setText("标定结果: 成功");
        }
        else
        {
            ui->label_xtalkCalibRes->setText("标定结果: 失败");
        }

        int VI530x_Cali_CG_Pos = (int8_t)frameData[5];
        int VI530x_Cali_CG_Maxratio = (uint8_t)frameData[6];
        int VI530x_Cali_CG_Peak = uint8_t(frameData[7]) + (uint8_t(frameData[8])<<8);
        ui->label_VI530x_Cali_CG_Pos->setText("VI530x_Cali_CG_Pos:"+QString::number(VI530x_Cali_CG_Pos));
        ui->label_VI530x_Cali_CG_Maxratio->setText("VI530x_Cali_CG_Maxratio:"+QString::number(VI530x_Cali_CG_Maxratio));
        ui->label_VI530x_Cali_CG_Peak->setText("VI530x_Cali_CG_Peak:"+QString::number(VI530x_Cali_CG_Peak));
    }
    else if(uint8_t(frameData[3])==ID_OFFSET_CALIB)
    {
        if(uint8_t(frameData[4]) == 0)
        {
            ui->label_offsetCalibRes->setText("标定结果: 成功");
        }
        else
        {
            ui->label_offsetCalibRes->setText("标定结果: 失败");
        }

        int dist_offset_mm = uint8_t(frameData[5]) + (uint8_t(frameData[6])<<8);
        ui->label_dist_offset_mm->setText("dist_offset_mm:"+QString::number(dist_offset_mm));
    }
}

void MainWindow::dispDeviceConfig()
{
    qDebug() << "\n\n read device config done. ";

    ui->lineEdit_sn->setText(QString::fromLatin1(mDevConfigStruct.sn, 16));

    ui->comboBox_jtxWorkMode->setCurrentIndex(mDevConfigStruct.jtx_work_mode);
    ui->comboBox_mode->setCurrentIndex(mDevConfigStruct.vi4302_mode);

    ui->comboBox_hardlinePinSel->setCurrentIndex(mDevConfigStruct.hardline_pin_sel);
    ui->comboBox_hardlinePinMode->setCurrentIndex(mDevConfigStruct.hardline_pin_mode);
    if(mDevConfigStruct.is_ranging_enable)
    {
        ui->checkBox_rangeEnable->setCheckState(Qt::Checked);
    }
    else
    {
        ui->checkBox_rangeEnable->setCheckState(Qt::Unchecked);
    }
    ui->lineEdit_ldTrigPwidth->setText(QString::number(mDevConfigStruct.ld_trig_pwidth_100ps));
    ui->lineEdit_sampleRate->setText(QString::number(mDevConfigStruct.sample_rate));
    ui->lineEdit_hardlinePulseMs->setText(QString::number(mDevConfigStruct.hardline_pulse_ms));
    ui->lineEdit_gndStableThMm->setText(QString::number(mDevConfigStruct.gnd_stable_th_mm));
    ui->lineEdit_footStableThMm->setText(QString::number(mDevConfigStruct.foot_stable_th_mm));
    ui->lineEdit_footThMinMm->setText(QString::number(mDevConfigStruct.valid_foot_th_min_mm));
    ui->lineEdit_footThMaxMm->setText(QString::number(mDevConfigStruct.valid_foot_th_max_mm));
    ui->lineEdit_dataWinSize->setText(QString::number(mDevConfigStruct.data_win_size));
    ui->lineEdit_footInHoldMaxTimes->setText(QString::number(mDevConfigStruct.foot_in_hold_max_times));
    ui->lineEdit_walkErrK->setText(QString::number(mDevConfigStruct.walk_err_k));
    ui->lineEdit_distOffset->setText(QString::number(mDevConfigStruct.dist_offset_mm));
    ui->lineEdit_dirtyDistTh->setText(QString::number(mDevConfigStruct.dirty_dist_th_mm));
    ui->lineEdit_lowPeakTh->setText(QString::number(mDevConfigStruct.low_peak_th));
    ui->lineEdit_ldTrigNum->setText(QString::number(mDevConfigStruct.spad_int_num));
    ui->lineEdit_btLockRssi->setText(QString::number(mDevConfigStruct.bt_lock_rssi));
    ui->lineEdit_btUnlockRssi->setText(QString::number(mDevConfigStruct.bt_unlock_rssi));
    if(mDevConfigStruct.bt_test_mode)
    {
        ui->checkBox_btTestMode->setCheckState(Qt::Checked);
    }
    else
    {
        ui->checkBox_btTestMode->setCheckState(Qt::Unchecked);
    }

    ui->label_bvdVal->setText("0x24F:"+QString::number(mDevConfigStruct.vi4302_bvd_val));
    ui->label_bvdTemp->setText("temp:"+QString::number(mDevConfigStruct.vi4302_calib_tmpr));
    ui->label_VI530x_Cali_CG_Pos->setText("VI530x_Cali_CG_Pos:"+QString::number(mDevConfigStruct.VI530x_Cali_CG_Pos));
    ui->label_VI530x_Cali_CG_Maxratio->setText("VI530x_Cali_CG_Maxratio:"+QString::number(mDevConfigStruct.VI530x_Cali_CG_Maxratio));
    ui->label_VI530x_Cali_CG_Peak->setText("VI530x_Cali_CG_Peak:"+QString::number(mDevConfigStruct.VI530x_Cali_CG_Peak));
    ui->label_dist_offset_mm->setText("dist_offset_mm:"+QString::number(mDevConfigStruct.dist_offset_mm));

    if(mDevConfigStruct.led_enable)
    {
        ui->checkBox_ledEnable->setCheckState(Qt::Checked);
    }
    else
    {
        ui->checkBox_ledEnable->setCheckState(Qt::Unchecked);
    }

    ui->lineEdit_ledBreathPeak->setText(QString::number((float)mDevConfigStruct.led_breath_peak_x10000/10000.0));
    ui->lineEdit_ledBreathDepth->setText(QString::number((float)mDevConfigStruct.led_breath_depth_x10000/10000.0));
    ui->lineEdit_ledBreathPeriod->setText(QString::number(mDevConfigStruct.led_breath_period_ms));
}

void MainWindow::on_pushButton_test_clicked()
{
    showImg(1);
}


void MainWindow::on_comboBox_jtxWorkMode_activated(int index)
{
    m_serialPortReader.devSetJtxWorkMode(index);
}


void MainWindow::on_comboBox_mode_activated(int index)
{
    m_serialPortReader.devSetVi4302Mode(index);
}


void MainWindow::on_pushButton_record_clicked()
{
    QStringList tmpStrList;
    QString mode = ui->comboBox_mode->currentText();
    tmpStrList = mode.split(" ");
    QString modeStr = tmpStrList[0];

    QString pulseNumListStr = ui->lineEdit_pulseNumList->text();
    QList<int> pulseNumList;
    if(pulseNumListStr.size() > 0)
    {
        tmpStrList = pulseNumListStr.split(" ");
        foreach(const QString &str, tmpStrList)
        {
            pulseNumList.append(str.toInt());
        }
    }
    int atBvd = ui->lineEdit_scanPulseNumListAtBvd->text().toInt();

    QString bvdListStr = ui->lineEdit_bvdList->text();
    QList<int> bvdList;
    if(bvdListStr.size() > 0)
    {
        tmpStrList = bvdListStr.split(" ");
        foreach(const QString &str, tmpStrList)
        {
            bvdList.append(str.toInt());
        }
    }
    int atPulseNum = ui->lineEdit_scanBvdAtPulseNum->text().toInt();

    int frameNum = ui->lineEdit_setFramNum->text().toInt();

    qDebug() << "frameNum=" << frameNum;
    qDebug() << "pulseNumList.size()=" << pulseNumList.size();
    qDebug() << "pulseNumList: " << pulseNumList;
    qDebug() << "bvdList.size()=" << bvdList.size();
    qDebug() << "bvdList:" << bvdList;

    QString filenamePrefix = "data/mode_" + modeStr + "-realdist_" + ui->lineEdit_realDist->text() + "-ref_" + ui->lineEdit_ref->text();

    if(m_serialPortReader.isRecording())
    {
        m_serialPortReader.stopRecord();
        ui->pushButton_record->setText("开始录制");
    }
    else
    {
        m_serialPortReader.startRecord(filenamePrefix, ui->comboBox_mode->currentIndex(), pulseNumList, atBvd, bvdList, atPulseNum, frameNum);
        ui->pushButton_record->setText("停止录制");
    }
}


void MainWindow::on_pushButton_openDataDir_clicked()
{
    QString folderPath = "data";
    QUrl folderUrl = QUrl::fromLocalFile(folderPath);
    // 打开文件资源管理器
    if (QDesktopServices::openUrl(folderUrl)) {
        qDebug() << "File explorer opened successfully.";
    } else {
        qDebug() << "Failed to open file explorer.";
    }
}


void MainWindow::on_pushButton_readFirmwareVersion_clicked()
{
    m_serialPortReader.devReadFirmwareVersion();
    ui->label_firmwareVersion->setText("");
}


void MainWindow::on_checkBox_rangeEnable_stateChanged(int arg1)
{
    if(arg1)
    {
        m_serialPortReader.devRangingEnable(1);
    }
    else
    {
        m_serialPortReader.devRangingEnable(0);
    }

}

void MainWindow::on_pushButton_saveConfig_clicked()
{
    m_serialPortReader.devSaveConfig();
}


void MainWindow::on_pushButton_softReset_clicked()
{
    m_serialPortReader.devSoftReset();
}


void MainWindow::on_pushButton_hardlineConfig_clicked()
{
    int pin_sel = ui->comboBox_hardlinePinSel->currentIndex();
    int pin_mode = ui->comboBox_hardlinePinMode->currentIndex();
    int pwidth = ui->lineEdit_hardlinePulseMs->text().toInt();

    qDebug() << "pin_sel=" << pin_sel << " pin_mode="<< pin_mode << " pwidth="<< pwidth;
    m_serialPortReader.devSetHardLineConfig(pin_sel,pin_mode,pwidth);
}


void MainWindow::on_pushButton_sampleRate_clicked()
{
    int rate = ui->lineEdit_sampleRate->text().toInt();
    qDebug() << "sample rate=" << rate;
    m_serialPortReader.devSetSampleRate(rate);
}


void MainWindow::on_pushButton_ldTrigPwidth_clicked()
{
    int pwidth = ui->lineEdit_ldTrigPwidth->text().toInt();
    qDebug() << "ld trig pulse width =" << pwidth << "*100ps";
    m_serialPortReader.devSetLdTrigPwidth(pwidth);
}


void MainWindow::on_pushButton_footDetectParaConfig_clicked()
{
    FootDetectParaStruct para;
    para.gnd_stable_th_mm = ui->lineEdit_gndStableThMm->text().toInt();
    para.foot_stable_th_mm = ui->lineEdit_footStableThMm->text().toInt();
    para.valid_foot_th_min_mm = ui->lineEdit_footThMinMm->text().toInt();
    para.valid_foot_th_max_mm = ui->lineEdit_footThMaxMm->text().toInt();
    para.data_win_size = ui->lineEdit_dataWinSize->text().toInt();
    para.foot_in_hold_max_times = ui->lineEdit_footInHoldMaxTimes->text().toInt();
    m_serialPortReader.devSetFootDetectPara(para);
}


void MainWindow::on_pushButton_readConfig_clicked()
{
    m_serialPortReader.devReadAllPara();
}


void MainWindow::on_pushButton_walkErrK_clicked()
{
    int k = ui->lineEdit_walkErrK->text().toInt();
    qDebug() << "walk err k=" << k;
    m_serialPortReader.devSetWalkErrK(k);
}


void MainWindow::on_pushButton_distOffset_clicked()
{
    int offset = ui->lineEdit_distOffset->text().toInt();
    qDebug() << "dist offset=" << offset;
    m_serialPortReader.devSetDistOffset(offset);
}


void MainWindow::on_pushButton_eraseFlash_clicked()
{
    m_serialPortReader.devEraseFlash();
}


void MainWindow::on_pushButton_dirtyDistTh_clicked()
{
    int th = ui->lineEdit_dirtyDistTh->text().toInt();
    m_serialPortReader.devSetDirtyDistTh(th);
}


void MainWindow::on_pushButton_lowPeakTh_clicked()
{
    int th = ui->lineEdit_lowPeakTh->text().toInt();
    m_serialPortReader.devSetLowPeakTh(th);
}


void MainWindow::on_pushButton_ldTrigNum_clicked()
{
    int trig_num = ui->lineEdit_ldTrigNum->text().toInt();
    m_serialPortReader.devSetLdTrigNum(trig_num);
}


void MainWindow::on_pushButton_bvdCalib_clicked()
{
    ui->label_bvdVal->setText("0x24F:");
    ui->label_bvdTemp->setText("temp:");
    m_serialPortReader.devBvdCalib();
}


void MainWindow::on_pushButton_readReg_clicked()
{
    bool ok;
    int reg_addr = ui->lineEdit_regAddr->text().toInt(&ok, 16);
    ui->lineEdit_regVal->setText("");
    m_serialPortReader.devReadReg(reg_addr);
    qDebug() << "read reg: addr = " << reg_addr;
}


void MainWindow::on_pushButton_writeReg_clicked()
{
    bool ok;
    int reg_addr = ui->lineEdit_regAddr->text().toInt(&ok, 16);
    int reg_val = ui->lineEdit_regVal->text().toInt();
    m_serialPortReader.devWriteReg(reg_addr, reg_val);
    qDebug() << "write reg: addr = " << reg_addr << "val = " << reg_val;
}


void MainWindow::on_checkBox_ledEnable_stateChanged(int arg1)
{
    if(arg1)
    {
        m_serialPortReader.devLedEnable(1);
    }
    else
    {
        m_serialPortReader.devLedEnable(0);
    }
}


void MainWindow::on_pushButton_btRssiSet_clicked()
{
    int lock_rssi = ui->lineEdit_btLockRssi->text().toInt();
    int unlock_rssi = ui->lineEdit_btUnlockRssi->text().toInt();
    m_serialPortReader.devBtRssiTh(lock_rssi, unlock_rssi);
}

void MainWindow::on_checkBox_btTestMode_stateChanged(int arg1)
{
    if(arg1)
    {
        m_serialPortReader.devBtTestMode(1);
    }
    else
    {
        m_serialPortReader.devBtTestMode(0);
    }
}


void MainWindow::on_pushButton_xtalkCalib_clicked()
{
    ui->label_xtalkCalibRes->setText("标定结果");
    ui->label_VI530x_Cali_CG_Pos->setText("VI530x_Cali_CG_Pos:");
    ui->label_VI530x_Cali_CG_Maxratio->setText("VI530x_Cali_CG_Maxratio:");
    ui->label_VI530x_Cali_CG_Peak->setText("VI530x_Cali_CG_Peak:");
    m_serialPortReader.devXtalkCalib();
}

void MainWindow::on_pushButton_offsetCalib_clicked()
{
    int mm = ui->lineEdit_offsetCalibTrueMm->text().toInt();
    ui->label_offsetCalibRes->setText("标定结果");
    ui->label_dist_offset_mm->setText("dist_offset_mm:");
    m_serialPortReader.devOffsetCalib(mm);
}


void MainWindow::on_pushButton_ledConfig_clicked()
{
    float peak = ui->lineEdit_ledBreathPeak->text().toFloat();
    float depth = ui->lineEdit_ledBreathDepth->text().toFloat();
    int period = ui->lineEdit_ledBreathPeriod->text().toInt();
    m_serialPortReader.devSetLedBreathPara(peak, depth, period);
}


void MainWindow::on_pushButton_writeSn_clicked()
{
    QString sn = ui->lineEdit_sn->text();
    if(sn.size()>16)
    {
        QMessageBox messageBox;
        messageBox.setText("SN长度应为16字节");
        messageBox.exec();
        return;
    }

    sn = sn.leftJustified(16, ' ');

    QByteArray byteArray = sn.toLatin1();
    char sn_char[16];
    qstrncpy(sn_char, byteArray.constData(), 16);
    m_serialPortReader.devWriteSn(sn_char);
}

