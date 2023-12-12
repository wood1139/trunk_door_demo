#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_timer.setInterval(200);
    QObject::connect(&m_timer, &QTimer::timeout, this, &MainWindow::slotSwitchImg);
    qDebug() << "app start!";

    m_groundDist = 0;
    m_keyIsOn = false;
    m_doorIsOpen = false;
    m_groundIsFound = false;
    m_dataIsStable = false;
    m_distArrIdx = 0;
    for(int i=0; i<DATA_WIN_SIZE; i++)
    {
        m_distArr[i] = 0;
    }
    showImg(0);
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
        connect(&m_serialPortReader, &SerialPortHandler::sigLidarData, this, &MainWindow::handleLidarData);
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

void MainWindow::handleLidarData(int dist, int amp)
{
//    m_distArr[m_distArrIdx++] = dist;
//    m_distArrIdx %= DATA_WIN_SIZE;
//    updateState();
    // qDebug() << "dist =" << dist << "amp =" << amp;
    m_timer.start();
    showImg(0);
}

void MainWindow::on_pushButton_test_clicked()
{
    m_keyIsOn = true;
    showImg(1);
}

bool MainWindow::isDataStable(int &meanDist)
{
    int maxDist = 0;
    int minDist = qInf();
    int sumDist = 0;
    bool res = false;
    for (int i=0; i<DATA_WIN_SIZE; i++)
    {
        if(m_distArr[i] > maxDist) maxDist = m_distArr[i];
        if(m_distArr[i] < minDist) minDist = m_distArr[i];
        sumDist += m_distArr[i];
    }
    if(maxDist - minDist <= 3)
    {
        res = true;
    }
    meanDist = sumDist / DATA_WIN_SIZE;
    return res;
}

void MainWindow::updateState()
{
    int meanDist;
    if(m_keyIsOn)
    {
        if(m_groundIsFound)
        {
            if(m_doorIsOpen)
            {
//                if(isDataStable(meanDist))
//                {
//                    if(m_groundDist - meanDist < 2)
//                    {
//                        m_doorIsOpen = false;
//                        // blink led and close the door
//                    }
//                }
            }
            else
            {
                if(isDataStable(meanDist))
                {
                    if(m_groundDist - meanDist > 2)
                    {
//                        m_doorIsOpen = true;
                        // blink led and open the door
                        m_keyIsOn = false;
                        qDebug() << "blink led and open the door";
                        m_timer.start();
                        showImg(0);
                    }
                }
            }
        }
        else
        {
            if(isDataStable(meanDist))
            {
                m_groundIsFound = true;
                m_groundDist = meanDist;
                // turn on led
                qDebug() << "ground found, dist=" << meanDist <<"turn on led";
            }
        }
    }
    else
    {
        m_groundIsFound = false;
    }
}


