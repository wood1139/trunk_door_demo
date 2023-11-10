#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qDebug() << "app start!";
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_connectCom_clicked()
{
    qDebug() << "pushbutton";
    m_serialPortReader.connectCom("COM3", 460800);
}

