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

void SerialPortHandler::setLineSeriesPtr(QLineSeries *ptr)
{
    m_lineSeriesPtr = ptr;
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
        {
            if(0 == pack_id)
            {
                m_histData.clear();
            }
            if(uint8_t(m_frameData[4])==pack_id)
            {
                for(int i=0; i<128; i++)
                {
                    m_histData.append(QPointF(pack_id*128+i, uint8_t(m_frameData[5+i])));
                }
                pack_id++;
                if(pack_id == 2048/128)
                {
                    pack_id = 0;
                    if(nullptr != m_lineSeriesPtr)
                    {
                        m_lineSeriesPtr->replace(m_histData);
                    }
                }
            }
            else
            {
                // something went wrong
                pack_id = 0;
            }
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
