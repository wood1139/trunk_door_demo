/****************************************************************************
**
** Copyright (C) 2013 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef SERIALPORTHANDLER_H
#define SERIALPORTHANDLER_H

#include <QByteArray>
#include <QSerialPort>
#include <QStringList>
#include <QtCharts/QLineSeries>
#include <QStandardItemModel>
#include <QFile>
#include <QTextStream>

QT_CHARTS_USE_NAMESPACE

QT_BEGIN_NAMESPACE

QT_END_NAMESPACE

typedef struct
{
    int norm_tof;
    int norm_peak;
    int norm_noise;
    int int_num;
    int atten_peak;
    int atten_noise;
    int ref_tof;
    int temp_sensor_x100;
    int temp_mcu_x100;
    int raw_tof_mm;
    int ctof;
    int confidence;
    uint32_t timestamp_ms;
}RawDataStruct;

class SerialPortHandler : public QObject
{
    Q_OBJECT

public:
    explicit SerialPortHandler(QObject *parent = nullptr);
    bool connectCom(QString portName, int baudrate);
    void disconnectCom();
    QStringList scanComList();
    bool isConnected();
    void setDataPtr(QLineSeries *LinePtr, QStandardItemModel *tabPtr);
    void startRecord(QString filename, int mode);
    void stopRecord();
    bool isRecording();

    void devSetVi4302Mode(int mode);
    void devSetHardLineConfig(int pin_sel, int pin_mode, int pwidth_ms);
    void devSetSampleRate(int sample_rate);
    void devSetLdTrigPwidth(int pwidth);
    void devSoftReset();
    void devReadFirmwareVersion();
    void devRangingEnable(int en);
    void devSaveConfig();

private:
    uint8_t calSum(QByteArray data);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort m_serialPort;
    QByteArray m_readData;
    QByteArray m_frameData;
    QVector<QPointF> m_histData;

    QLineSeries *m_lineSeriesPtr;
    QStandardItemModel *m_tableModelPtr;

    RawDataStruct m_rangeRawData;

    bool m_isRecording;
    int m_mode;
    QFile m_hfile;
    QTextStream m_fstream;

signals:
    void sigLidarData(QByteArray frameData);
    void sigSetAxisRange(int xmin, int xmax, int ymin, int ymax);
};

#endif // SERIALPORTHANDLER_H
