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

#include "serialporthandler.h"

#include <QCoreApplication>
#include<QDebug>
#include <QSerialPortInfo>

SerialPortHandler::SerialPortHandler(QObject *parent) :
    QObject(parent)
{

}

bool SerialPortHandler::connectCom(QString portName, int baudrate)
{
    m_serialPort.setPortName(portName);
    m_serialPort.setBaudRate(baudrate);

    if (!m_serialPort.open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open port";
        return false;
    }

    connect(&m_serialPort, &QSerialPort::readyRead, this, &SerialPortHandler::handleReadyRead);
    connect(&m_serialPort, &QSerialPort::errorOccurred, this, &SerialPortHandler::handleError);

//    QByteArray cmd100Hz;
//    cmd100Hz.resize(6);
//    cmd100Hz[0] = 0xAA;
//    cmd100Hz[1] = 0x55;
//    cmd100Hz[2] = 0x64;
//    cmd100Hz[3] = 0x01;
//    cmd100Hz[4] = 0x01;
//    cmd100Hz[5] = 0x65;
//    m_serialPort.write(cmd100Hz);

//    QByteArray cmdStart;
//    cmdStart.resize(5);
//    cmdStart[0] = 0xAA;
//    cmdStart[1] = 0x55;
//    cmdStart[2] = 0x60;
//    cmdStart[3] = 0x00;
//    cmdStart[4] = 0x5F;
//    m_serialPort.write(cmdStart);

    return true;
}
void SerialPortHandler::disconnectCom()
{
//    QByteArray cmdStop;
//    cmdStop.resize(5);
//    cmdStop[0] = 0xAA;
//    cmdStop[1] = 0x55;
//    cmdStop[2] = 0x61;
//    cmdStop[3] = 0x00;
//    cmdStop[4] = 0x60;
//    m_serialPort.write(cmdStop);
//    m_serialPort.waitForBytesWritten();
    m_serialPort.close();
}

QStringList SerialPortHandler::scanComList()
{
    QStringList res;
    QList<QSerialPortInfo> portList = QSerialPortInfo::availablePorts();
    for(int i=0; i<portList.size(); i++)
    {
        qDebug() << portList[i].portName() + " " + portList[i].description();
        res.append(portList[i].portName());
    }
    return res;
}

bool SerialPortHandler::isConnected()
{
    return m_serialPort.isOpen();
}

void SerialPortHandler::setDataPtr(QLineSeries *LinePtr, QStandardItemModel *tabPtr)
{
    m_lineSeriesPtr = LinePtr;
    m_tableModelPtr = tabPtr;
}

void SerialPortHandler::setVi4302Mode(int mode)
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(6);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x06;
        cmd[3] = 0x03;
        cmd[4] = mode;
        cmd[5] = calSum(cmd.mid(0,5));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

uint8_t SerialPortHandler::calSum(QByteArray data)
{
    uint8_t res = 0;
    for(int i=0; i<data.size(); i++)
    {
        res += uint8_t(data[i]);
    }
    return res;
}

void SerialPortHandler::handleReadyRead()
{
    static uint8_t pack_id = 0;
    static int hist_data_max = 0;

    m_readData.append(m_serialPort.readAll());

    while(m_readData.size()>=5)
    {
        if(uint8_t(m_readData[0])!=0x82)
        {
            m_readData.remove(0, 1);
            continue;
        }
        if(uint8_t(m_readData[1])!=0x2A)
        {
            m_readData.remove(0, 1);
            continue;
        }
        uint8_t len = m_readData[2];
        if(m_readData.size() < len)
        {
            break;
        }
        uint8_t sumRes = calSum(m_readData.mid(0, len-1));
        if(uint8_t(m_readData[len-1])!=sumRes)
        {
            m_readData.remove(0, 1);
            continue;
        }
        m_frameData = m_readData.mid(0, len);
        m_readData.remove(0, len);

        if(uint8_t(m_frameData[3])==0x01)
        {// histogram data
            if(0 == pack_id)
            {
                m_histData.clear();
                hist_data_max = 0;
            }
            if(uint8_t(m_frameData[4])==pack_id)
            {
                for(int i=0; i<128; i++)
                {
                    m_histData.append(QPointF(pack_id*128+i, uint8_t(m_frameData[5+i])));
                    if(uint8_t(m_frameData[5+i])>hist_data_max)
                    {
                        hist_data_max = m_frameData[5+i];
                    }
                }
                pack_id++;
                if(pack_id == 2048/128)
                {
                    pack_id = 0;
                    if(nullptr != m_lineSeriesPtr)
                    {
                        m_lineSeriesPtr->replace(m_histData);
                        int y_range = hist_data_max * 1.2;
                        if(y_range < 20)
                        {
                            y_range = 20;
                        }
                        emit sigSetAxisRange(0, 2048, 0, y_range);
                    }
                }
            }
            else
            {
                // something went wrong
                pack_id = 0;
            }
        }
        else if(uint8_t(m_frameData[3])==0x02)
        {// sigle pixel data
            for (int row = 0; row < 5; ++row) {
                for (int column = 0; column < 5; ++column) {
                    uint16_t val = uint8_t(m_frameData[4+row*10+column*2]) + (uint8_t(m_frameData[5+row*10+column*2])<<8);
                    m_tableModelPtr->setData(m_tableModelPtr->index(row,column,QModelIndex()), QString::number(val));
                }
            }
        }
        else if(uint8_t(m_frameData[3])==0x03)
        {// range raw data
            m_rangeRawData.norm_tof = uint8_t(m_frameData[4]) + (uint8_t(m_frameData[5])<<8);
            m_rangeRawData.norm_peak = uint8_t(m_frameData[6]) + (uint8_t(m_frameData[7])<<8);
            m_rangeRawData.norm_noise = uint8_t(m_frameData[8]) + (uint8_t(m_frameData[9])<<8) + (uint8_t(m_frameData[10])<<16);
            m_rangeRawData.int_num = uint8_t(m_frameData[11]) + (uint8_t(m_frameData[12])<<8);
            m_rangeRawData.atten_peak = uint8_t(m_frameData[13]) + (uint8_t(m_frameData[14])<<8);
            m_rangeRawData.atten_noise = uint8_t(m_frameData[15]) + (uint8_t(m_frameData[16])<<8) + (uint8_t(m_frameData[17])<<16);
            m_rangeRawData.ref_tof = uint8_t(m_frameData[18]) + (uint8_t(m_frameData[19])<<8);
            m_rangeRawData.temp_sensor_x100 = uint8_t(m_frameData[20]) + (uint8_t(m_frameData[21])<<8);
            m_rangeRawData.temp_mcu_x100 = uint8_t(m_frameData[22]) + (uint8_t(m_frameData[23])<<8);
            m_rangeRawData.ctof = uint8_t(m_frameData[24]) + (uint8_t(m_frameData[25])<<8);
            m_rangeRawData.confidence = uint8_t(m_frameData[26]);

            int width = 5;
            qDebug().noquote() <<   "norm_tof=" << QString("%1").arg(m_rangeRawData.norm_tof, width) <<
                                    "norm_peak=" << QString("%1").arg(m_rangeRawData.norm_peak, width) <<
                                    "norm_noise=" << QString("%1").arg(m_rangeRawData.norm_noise, width) <<
                                    "int_num=" << QString("%1").arg(m_rangeRawData.int_num, width) <<
                                    "atten_peak=" << QString("%1").arg(m_rangeRawData.atten_peak, width) <<
                                    "atten_noise=" << QString("%1").arg(m_rangeRawData.atten_noise, width) <<
                                    "ref_tof=" << QString("%1").arg(m_rangeRawData.ref_tof, width) <<
                                    "temp_sensor_x100=" << QString("%1").arg(m_rangeRawData.temp_sensor_x100, width) <<
                                    "temp_mcu_x100=" << QString("%1").arg(m_rangeRawData.temp_mcu_x100, width) <<
                                    "ctof=" << QString("%1").arg(m_rangeRawData.ctof, width) <<
                                    "confidence=" << QString("%1").arg(m_rangeRawData.confidence, width);
        }
        else
        {
            emit sigLidarData(m_frameData);
        }
    }
}

void SerialPortHandler::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::ReadError) {
        qDebug() << "An I/O error occurred while reading";
        QCoreApplication::exit(1);
    }
}
