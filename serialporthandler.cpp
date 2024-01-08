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

    return true;
}
void SerialPortHandler::disconnectCom()
{
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

void SerialPortHandler::startRecord(QString filename, int mode)
{
    m_hfile.setFileName(filename);
    if (m_hfile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        m_mode = mode;
        m_fstream.setDevice(&m_hfile);
        m_isRecording = true;

        if(0 == mode)
        {// Range Mode
            m_fstream << "norm_tof,norm_peak,norm_noise,int_num,atten_peak,atten_noise,ref_tof,temp_sensor_x100,temp_mcu_x100,raw_tof_mm,ctof,confidence,timestamp_ms,\n";
        }
        else if(1 == mode)
        {// Single Pix Mode
            for(int pixId = 1; pixId<=25; pixId++)
            {
                m_fstream << "pix_" << pixId << ",";
            }
            m_fstream << "\n";
        }
        else if(2 == mode)
        {// Histogram Mode
            m_fstream << "idx,data,\n";
        }
    }
}

void SerialPortHandler::stopRecord()
{
    m_isRecording = false;
    m_hfile.close();
}

bool SerialPortHandler::isRecording()
{
    return m_isRecording;
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

        if(uint8_t(m_frameData[3])==0xA1)
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
                    if(m_isRecording)
                    {
                        for(int i=0; i<2048; i++)
                        {
                            m_fstream << m_histData[i].x() << "," << m_histData[i].y() << ",\n";
                        }
                    }
                }
            }
            else
            {
                // something went wrong
                pack_id = 0;
            }
        }
        else if(uint8_t(m_frameData[3])==0xA2)
        {// sigle pixel data
            for (int row = 0; row < 5; ++row) {
                for (int column = 0; column < 5; ++column) {
                    uint16_t val = uint8_t(m_frameData[4+row*10+column*2]) + (uint8_t(m_frameData[5+row*10+column*2])<<8);
                    QStandardItem *item = new QStandardItem(QString::number(val));
                    // 根据数值设置颜色，2200最红，0最蓝
                    qreal ck = (qreal)val/2200.0;
                    if(ck > 1)
                    {
                        ck = 1;
                    }
                    QColor backgroundColor(255*ck,0,(1-ck)*255);
                    item->setData(backgroundColor, Qt::BackgroundRole);
                    m_tableModelPtr->setItem(row, column, item);

                    if(m_isRecording)
                    {
                        m_fstream << val << ",";
                    }
                }
            }
            m_fstream << "\n";
        }
        else if(uint8_t(m_frameData[3])==0xA3)
        {// range raw data
            m_rangeRawData.norm_tof = uint8_t(m_frameData[4]) + (uint8_t(m_frameData[5])<<8);
            m_rangeRawData.norm_peak = uint8_t(m_frameData[6]) + (uint8_t(m_frameData[7])<<8);
            m_rangeRawData.norm_noise = uint8_t(m_frameData[8]) + (uint8_t(m_frameData[9])<<8) + (uint8_t(m_frameData[10])<<16);
            m_rangeRawData.int_num = uint8_t(m_frameData[11]) + (uint8_t(m_frameData[12])<<8);
            m_rangeRawData.atten_peak = uint8_t(m_frameData[13]) + (uint8_t(m_frameData[14])<<8);
            m_rangeRawData.atten_noise = uint8_t(m_frameData[15]) + (uint8_t(m_frameData[16])<<8) + (uint8_t(m_frameData[17])<<16);
            m_rangeRawData.ref_tof = uint8_t(m_frameData[18]) + (uint8_t(m_frameData[19])<<8);
            m_rangeRawData.temp_sensor_x100 = int16_t(uint16_t(uint8_t(m_frameData[20]) + (uint8_t(m_frameData[21])<<8)));
            m_rangeRawData.temp_mcu_x100 = int16_t(uint16_t(uint8_t(m_frameData[22]) + (uint8_t(m_frameData[23])<<8)));
            m_rangeRawData.raw_tof_mm = int16_t(uint16_t(uint8_t(m_frameData[24]) + (uint8_t(m_frameData[25])<<8)));
            m_rangeRawData.ctof = uint8_t(m_frameData[26]) + (uint8_t(m_frameData[27])<<8);
            m_rangeRawData.confidence = uint8_t(m_frameData[28]);
            m_rangeRawData.timestamp_ms = uint8_t(m_frameData[29]) + (uint8_t(m_frameData[30])<<8) + (uint8_t(m_frameData[31])<<16) + (uint8_t(m_frameData[32])<<16);

            int width = 5;
            qDebug().noquote() <<   "ntof=" << QString("%1").arg(m_rangeRawData.norm_tof, width) <<
                                    "npeak=" << QString("%1").arg(m_rangeRawData.norm_peak, width) <<
                                    "nnoise=" << QString("%1").arg(m_rangeRawData.norm_noise, 3) <<
                                    "int_num=" << QString("%1").arg(m_rangeRawData.int_num, 3) <<
                                    "apeak=" << QString("%1").arg(m_rangeRawData.atten_peak, width) <<
                                    "anoise=" << QString("%1").arg(m_rangeRawData.atten_noise, 3) <<
                                    "ref_tof=" << QString("%1").arg(m_rangeRawData.ref_tof, width) <<
                                    "temp_sensor=" << QString("%1").arg(m_rangeRawData.temp_sensor_x100, 4) <<
                                    "temp_mcu=" << QString("%1").arg(m_rangeRawData.temp_mcu_x100, 4) <<
                                    "raw_tof_mm=" << QString("%1").arg(m_rangeRawData.raw_tof_mm, width) <<
                                    "ctof=" << QString("%1").arg(m_rangeRawData.ctof, width) <<
                                    "confidence=" << QString("%1").arg(m_rangeRawData.confidence, 3) <<
                                    "timestamp=" << QString("%1").arg(m_rangeRawData.timestamp_ms, width);
            if(m_isRecording)
            {
                m_fstream << m_rangeRawData.norm_tof << "," <<
                             m_rangeRawData.norm_peak << "," <<
                             m_rangeRawData.norm_noise << "," <<
                             m_rangeRawData.int_num << "," <<
                             m_rangeRawData.atten_peak << "," <<
                             m_rangeRawData.atten_noise << "," <<
                             m_rangeRawData.ref_tof << "," <<
                             m_rangeRawData.temp_sensor_x100 << "," <<
                             m_rangeRawData.temp_mcu_x100 << "," <<
                             m_rangeRawData.raw_tof_mm << "," <<
                             m_rangeRawData.ctof << "," <<
                             m_rangeRawData.confidence << "," <<
                             m_rangeRawData.timestamp_ms << ",\n";
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

void SerialPortHandler::devSetVi4302Mode(int mode)
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

void SerialPortHandler::devSetHardLineConfig(int pin_sel, int pin_mode, int pwidth_ms)
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(9);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x09;
        cmd[3] = 0x04;
        cmd[4] = pin_sel;
        cmd[5] = pin_mode;
        cmd[6] = (uint16_t)pwidth_ms & 0xFF;
        cmd[7] = ((uint16_t)pwidth_ms>>8) & 0xFF;
        cmd[8] = calSum(cmd.mid(0,8));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devSetSampleRate(int sample_rate)
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(7);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x07;
        cmd[3] = 0x06;
        cmd[4] = (uint16_t)sample_rate & 0xFF;
        cmd[5] = ((uint16_t)sample_rate>>8) & 0xFF;
        cmd[6] = calSum(cmd.mid(0,6));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devSetLdTrigPwidth(int pwidth)
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(7);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x07;
        cmd[3] = 0x07;
        cmd[4] = (uint16_t)pwidth & 0xFF;
        cmd[5] = ((uint16_t)pwidth>>8) & 0xFF;
        cmd[6] = calSum(cmd.mid(0,6));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devSoftReset()
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(5);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x05;
        cmd[3] = 0x02;
        cmd[4] = calSum(cmd.mid(0,4));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devReadFirmwareVersion()
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(5);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x05;
        cmd[3] = 0x01;
        cmd[4] = calSum(cmd.mid(0,4));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devRangingEnable(int en)
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(6);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x06;
        cmd[3] = 0x05;
        cmd[4] = en;
        cmd[5] = calSum(cmd.mid(0,5));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

void SerialPortHandler::devSaveConfig()
{
    QByteArray cmd;
    if(m_serialPort.isOpen())
    {
        cmd.resize(5);
        cmd[0] = 0x8F;
        cmd[1] = 0xD4;
        cmd[2] = 0x05;
        cmd[3] = 0x11;
        cmd[4] = calSum(cmd.mid(0,4));
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }
}

