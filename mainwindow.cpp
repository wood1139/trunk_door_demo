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
    if(cnt < 8)
    {
        showImg(cnt++ % 2);
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
    QStringList imagePaths = {"D:/projects/trunk_door_demo/trunk_door_demo/pic/car.jpg",
                              "D:/projects/trunk_door_demo/trunk_door_demo/pic/2.png",
                              "D:/projects/trunk_door_demo/trunk_door_demo/pic/3.png"};
    if (idx < 3)
    {
        QPixmap pixmap(imagePaths[idx]);
        ui->label_pic->setPixmap(pixmap);
    }
}

void MainWindow::handleLidarData(int dist, int amp)
{
    qDebug() << "dist =" << dist << "amp =" << amp;
}


void MainWindow::on_pushButton_test_clicked()
{
    m_timer.start();
    showImg(0);
}

