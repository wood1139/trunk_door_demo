#include "mainwindow.h"

#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    // 加载 QSS 样式表
    QFile styleFile("qss/Aqua.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream styleStream(&styleFile);
        a.setStyleSheet(styleStream.readAll());
        styleFile.close();
    } else {
        // 输出错误信息
        qDebug() << "Failed to open QSS file:" << styleFile.errorString();
    }

    QIcon icon(":/new/prefix1/pic/vanhymn.ico"); // 替换成你的图标文件路径
    w.setWindowIcon(icon);
    w.show();
    a.setWindowIcon(icon);
    return a.exec();
}
