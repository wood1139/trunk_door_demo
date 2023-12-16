#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDateTime>

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
//    m_timer.start();
//    showImg(0);
}

void MainWindow::on_pushButton_test_clicked()
{
    showImg(1);
}

void MainWindow::on_comboBox_mode_activated(int index)
{
    m_serialPortReader.setVi4302Mode(index);
}

