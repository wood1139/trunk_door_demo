#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDateTime>
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

   ui->comboBox_mode->addItem("Range Mode");
   ui->comboBox_mode->addItem("Single Pixel Mode");
   ui->comboBox_mode->addItem("Histogram Mode");

   ui->comboBox_hardlinePinSel->addItem("None");
   ui->comboBox_hardlinePinSel->addItem("UART TX");
   ui->comboBox_hardlinePinSel->addItem("LED1");
   ui->comboBox_hardlinePinSel->addItem("LED2");

   ui->comboBox_hardlinePinMode->addItem("Open-Drain");
   ui->comboBox_hardlinePinMode->addItem("Push-Pull");

   ui->lineEdit_hardlinePulseMs->setText("50");


   QDir dir("data");
   if (!dir.exists())
   {
        dir.mkpath(".");
   }

   on_pushButton_refreshComList_clicked();
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


void MainWindow::on_pushButton_refreshComList_clicked()
{
    QStringList comNameList = m_serialPortReader.scanComList();
    ui->comboBox_comList->clear();
    for(int i=0; i<comNameList.size(); i++)
    {
        ui->comboBox_comList->addItem(comNameList[i]);
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
}

void MainWindow::dispDeviceConfig()
{
    qDebug() << "\n\n read device config done. ";
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
    ui->lineEdit_walkErrK->setText(QString::number(mDevConfigStruct.walk_err_k));
    ui->lineEdit_distOffset->setText(QString::number(mDevConfigStruct.dist_offset_mm));

}

void MainWindow::on_pushButton_test_clicked()
{
    showImg(1);
}

void MainWindow::on_comboBox_mode_activated(int index)
{
    m_serialPortReader.devSetVi4302Mode(index);
}


void MainWindow::on_pushButton_record_clicked()
{
    QString mode = ui->comboBox_mode->currentText();
    QStringList words = mode.split(" ");
    QString modeStr = words[0];
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString formattedDateTime = currentDateTime.toString("yyyyMMddhhmmss");
    QString filename = "data/mode_" + modeStr + "-realdist_" + ui->lineEdit_realDist->text() + "-ref_" + ui->lineEdit_ref->text() + "-time_" + formattedDateTime + ".csv";
    qDebug() << filename;

    if(m_serialPortReader.isRecording())
    {
        m_serialPortReader.stopRecord();
        ui->pushButton_record->setText("开始录制");
    }
    else
    {
        m_serialPortReader.startRecord(filename, ui->comboBox_mode->currentIndex());
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

