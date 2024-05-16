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
#include <QDateTime>

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

bool SerialPortHandler::connectComLin(QString portName, int baudrate)
{
    m_serialPortLin.setPortName(portName);
    m_serialPortLin.setBaudRate(baudrate);

    if (!m_serialPortLin.open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open port";
        return false;
    }

    connect(&m_serialPortLin, &QSerialPort::readyRead, this, &SerialPortHandler::handleReadyReadLin);
    connect(&m_serialPortLin, &QSerialPort::errorOccurred, this, &SerialPortHandler::handleError);

    return true;
}

void SerialPortHandler::disconnectCom()
{
    m_serialPort.close();
}

void SerialPortHandler::disconnectComLin()
{
    m_serialPortLin.close();
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

bool SerialPortHandler::isConnectedLin()
{
    return m_serialPortLin.isOpen();
}

void SerialPortHandler::setDataPtr(QLineSeries *LinePtr, QStandardItemModel *tabPtr)
{
    m_lineSeriesPtr = LinePtr;
    m_tableModelPtr = tabPtr;
}

void SerialPortHandler::startRecord(QString filenamePrefix, int mode, QList<int> pulseNumList, int atBvd, QList<int> bvdList, int atPulseNum, int frameNum)
{
    m_filenamePrefix = filenamePrefix;
    m_mode = mode;
    m_pulseNumList = pulseNumList;
    m_atBvd = atBvd;
    m_bvdList = bvdList;
    m_atPulseNum = atPulseNum;
    m_frameNum = frameNum;

    m_pulseNumIdx = 0;
    m_bvdIdx = 0;

    scheduleRecord();
}

void SerialPortHandler::scheduleRecord()
{
    int pulseNum, bvd;
    if(m_pulseNumIdx < m_pulseNumList.size())
    {
        pulseNum = m_pulseNumList[m_pulseNumIdx];
        bvd = m_atBvd;
        m_pulseNumIdx++;
    }
    else if(m_bvdIdx < m_bvdList.size())
    {
        pulseNum = m_atPulseNum;
        bvd = m_bvdList[m_bvdIdx];
        m_bvdIdx++;
    }
    else
    {
        stopRecord();
        emit sigRecordStop();
        return;
    }

    devSetLdTrigNum(pulseNum);
    devWriteReg(0x24f, bvd);

    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString formattedDateTime = currentDateTime.toString("yyyyMMddhhmmss");

    QString filename = m_filenamePrefix+"-pulsenum_"+QString::number(pulseNum)+"-bvd_"+QString::number(bvd)+"-time_"+formattedDateTime+".csv";
    m_hfile.setFileName(filename);
    if (m_hfile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        m_fstream.setDevice(&m_hfile);

        if(0 == m_mode)
        {// Range Mode
            m_fstream << "norm_tof,norm_peak,norm_noise,int_num,atten_peak,atten_noise,ref_tof,temp_sensor_x100,temp_mcu_x100,raw_tof_mm,ctof,confidence,timestamp_ms,\n";
        }
        else if(1 == m_mode)
        {// Single Pix Mode
            for(int pixId = 1; pixId<=25; pixId++)
            {
                m_fstream << "pix_" << pixId << ",";
            }
            m_fstream << "\n";
        }
        else if(2 == m_mode)
        {// Histogram Mode
            m_fstream << "idx,data,\n";
        }

        m_frameCnt = -2;  // 丢弃前2帧，可能由于切换参数数据不稳定
        m_isRecording = true;
    }
}

void SerialPortHandler::handleData()
{
    static uint8_t pack_id = 0;
    static int hist_data_max = 0;

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
//                        int y_range = hist_data_max * 1.2;
//                        if(y_range < 20)
//                        {
//                            y_range = 20;
//                        }
//                        emit sigSetAxisRange(0, 2048, 0, y_range);
                }
                if(m_isRecording && m_frameCnt>=0)
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

                if(m_isRecording && m_frameCnt>=0)
                {
                    m_fstream << val << ",";
                }
            }
        }
        if(m_isRecording && m_frameCnt>=0)
        {
            m_fstream << "\n";
        }
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
        qDebug().noquote() <<   "nt=" << QString("%1").arg(m_rangeRawData.norm_tof, width) <<
                                "np=" << QString("%1").arg(m_rangeRawData.norm_peak, width) <<
                                "nn=" << QString("%1").arg(m_rangeRawData.norm_noise, 3) <<
                                "int=" << QString("%1").arg(m_rangeRawData.int_num, 3) <<
                                "ap=" << QString("%1").arg(m_rangeRawData.atten_peak, width) <<
                                "an=" << QString("%1").arg(m_rangeRawData.atten_noise, 3) <<
                                "rt=" << QString("%1").arg(m_rangeRawData.ref_tof, width) <<
                                "tsen=" << QString("%1").arg(m_rangeRawData.temp_sensor_x100, 4) <<
                                "tmcu=" << QString("%1").arg(m_rangeRawData.temp_mcu_x100, 4) <<
                                "raw_mm=" << QString("%1").arg(m_rangeRawData.raw_tof_mm, width) <<
                                "ct=" << QString("%1").arg(m_rangeRawData.ctof, width) <<
                                "conf=" << QString("%1").arg(m_rangeRawData.confidence, 3) <<
                                "ts=" << QString("%1").arg(m_rangeRawData.timestamp_ms, 9);
        emit sigProcDist(m_rangeRawData.ctof);
        if(m_isRecording && m_frameCnt>=0)
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

    m_frameCnt++;
    if(m_frameCnt >= m_frameNum)
    {
        m_isRecording = false;
        m_hfile.close();
        scheduleRecord();
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

void SerialPortHandler::dataForFrameSearch(QByteArray rawData)
{
    m_readData.append(rawData);

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

        if(uint8_t(m_frameData[3])>=0xA0)
        {
            handleData();
        }
        else
        {
            emit sigLidarData(m_frameData);
        }
    }
}

void SerialPortHandler::linPortWrite(QByteArray cmd)
{
    QByteArray buf;

    buf.resize(22);
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 22;    // nLen
    buf[3] = 0x00;  // isCheck
    buf[4] = 0x20;  // 主机发送数据命令码
    buf[5] = 0x00;  // 保留为0
    buf[6] = 0x01;  // 主发送0x01
    buf[7] = 0x00;  // 返回数据正确性？
    buf[8] = 0x3E;  // 下位机固定debug用的PID
    buf[9] = 0x01;  // 标准0, 增强1, 自定义2
    buf[10] = 0x08; // 数据长度

    buf[19] = 0x00; // LIN校验值，校验值，自定义下可写，标准和增强下无意义，但占位
    buf[20] = 0x00; // nCheck，不校验，随便填
    buf[21] = 0x5A;

    int idx = 0;
    int packSize = 0;
    while(idx < cmd.size())
    {
        if(idx+8 <= cmd.size())
        {
            packSize = 8;
        }
        else
        {
            packSize = cmd.size() - idx;
        }

        for(int n = 0; n < packSize; n++)
        {
            buf[11+n] = uint8_t(cmd[idx+n]);
        }

        m_serialPortLin.write(buf);
        m_serialPortLin.waitForBytesWritten();

        idx += packSize;
    }
}

void SerialPortHandler::handleReadyRead()
{
    dataForFrameSearch(m_serialPort.readAll());
}

void SerialPortHandler::handleReadyReadLin()
{

}


void SerialPortHandler::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::ReadError) {
        qDebug() << "An I/O error occurred while reading";
        QCoreApplication::exit(1);
    }
}

void SerialPortHandler::serialSendCmd(QByteArray cmd)
{
    if(m_serialPort.isOpen())
    {
        m_serialPort.write(cmd);
        m_serialPort.waitForBytesWritten();
    }

    if(m_serialPortLin.isOpen())
    {
        linPortWrite(cmd);
    }
}

void SerialPortHandler::devSetVi4302Mode(int mode)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x06;
    cmd[3] = 0x03;
    cmd[4] = mode;
    cmd[5] = calSum(cmd.mid(0,5));
    m_serialPort.write(cmd);
    m_serialPort.waitForBytesWritten();

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetHardLineConfig(int pin_sel, int pin_mode, int pwidth_ms)
{
    QByteArray cmd;
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

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetSampleRate(int sample_rate)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x07;
    cmd[3] = 0x06;
    cmd[4] = (uint16_t)sample_rate & 0xFF;
    cmd[5] = ((uint16_t)sample_rate>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetLdTrigPwidth(int pwidth)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x07;
    cmd[3] = 0x07;
    cmd[4] = (uint16_t)pwidth & 0xFF;
    cmd[5] = ((uint16_t)pwidth>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetLdTrigNum(int trig_num)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x07;
    cmd[3] = 0x0D;
    cmd[4] = (uint16_t)trig_num & 0xFF;
    cmd[5] = ((uint16_t)trig_num>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSoftReset()
{
    QByteArray cmd;
    cmd.resize(5);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x05;
    cmd[3] = 0x02;
    cmd[4] = calSum(cmd.mid(0,4));

    serialSendCmd(cmd);
}

void SerialPortHandler::devReadFirmwareVersion()
{
    QByteArray cmd;
    cmd.resize(5);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x05;
    cmd[3] = 0x01;
    cmd[4] = calSum(cmd.mid(0,4));

    serialSendCmd(cmd);
}

void SerialPortHandler::devRangingEnable(int en)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x06;
    cmd[3] = 0x05;
    cmd[4] = en;
    cmd[5] = calSum(cmd.mid(0,5));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSaveConfig()
{
    QByteArray cmd;
    cmd.resize(5);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x05;
    cmd[3] = 0x11;
    cmd[4] = calSum(cmd.mid(0,4));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetFootDetectPara(FootDetectParaStruct para)
{
    QByteArray cmd;
    cmd.resize(15);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 17;
    cmd[3] = ID_FOOT_DETECT_PARA;
    cmd[4] = para.gnd_stable_th_mm & 0xFF;
    cmd[5] = (para.gnd_stable_th_mm>>8) & 0xFF;
    cmd[6] = para.foot_stable_th_mm & 0xFF;
    cmd[7] = (para.foot_stable_th_mm>>8) & 0xFF;
    cmd[8] = para.valid_foot_th_min_mm & 0xFF;
    cmd[9] = (para.valid_foot_th_min_mm>>8) & 0xFF;
    cmd[10] = para.valid_foot_th_max_mm & 0xFF;
    cmd[11] = (para.valid_foot_th_max_mm>>8) & 0xFF;
    cmd[12] = para.data_win_size & 0xFF;
    cmd[13] = (para.data_win_size>>8) & 0xFF;
    cmd[14] = para.foot_in_hold_max_times & 0xFF;
    cmd[15] = (para.foot_in_hold_max_times>>8) & 0xFF;
    cmd[16] = calSum(cmd.mid(0,16));

    serialSendCmd(cmd);
}

void SerialPortHandler::devReadAllPara()
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 6;
    cmd[3] = ID_READ_ALL_PARAMS;
    cmd[4] = 0;
    cmd[5] = calSum(cmd.mid(0,5));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetJtxWorkMode(int mode)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x06;
    cmd[3] = ID_SET_JTX_WORK_MODE;
    cmd[4] = mode;
    cmd[5] = calSum(cmd.mid(0,5));
    m_serialPort.write(cmd);
    m_serialPort.waitForBytesWritten();

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetWalkErrK(int k)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 7;
    cmd[3] = ID_WALK_ERR_K;
    cmd[4] = (uint16_t)k & 0xFF;
    cmd[5] = ((uint16_t)k>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetDistOffset(int offset)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 7;
    cmd[3] = ID_DIST_OFFSET;
    cmd[4] = (uint16_t)offset & 0xFF;
    cmd[5] = ((uint16_t)offset>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetLowPeakTh(int th)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 7;
    cmd[3] = ID_LOW_PEAK_TH;
    cmd[4] = (uint16_t)th & 0xFF;
    cmd[5] = ((uint16_t)th>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,6));

    serialSendCmd(cmd);
}

void SerialPortHandler::devSetDirtyDistTh(int th)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 6;
    cmd[3] = ID_DIRTY_DIST_TH;
    cmd[4] = (uint8_t)th;
    cmd[5] = calSum(cmd.mid(0,5));

    serialSendCmd(cmd);
}

void SerialPortHandler::devEraseFlash()
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 6;
    cmd[3] = ID_FLASH_ERASE;
    cmd[4] = 0;
    cmd[5] = calSum(cmd.mid(0,5));

    serialSendCmd(cmd);
}

void SerialPortHandler::devBvdCalib()
{
    QByteArray cmd;
    cmd.resize(ID_BVD_CALIB_LEN);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x05;
    cmd[3] = ID_BVD_CALIB;
    cmd[4] = calSum(cmd.mid(0,4));

    serialSendCmd(cmd);
}

void SerialPortHandler::devReadReg(int reg_addr)
{
    QByteArray cmd;
    cmd.resize(ID_READ_REG_LEN);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = ID_READ_REG_LEN;
    cmd[3] = ID_READ_REG;
    cmd[4] = (uint16_t)reg_addr & 0xFF;
    cmd[5] = ((uint16_t)reg_addr>>8) & 0xFF;
    cmd[6] = calSum(cmd.mid(0,ID_READ_REG_LEN-1));

    serialSendCmd(cmd);
}

void SerialPortHandler::devWriteReg(int reg_addr, int reg_val)
{
    QByteArray cmd;
    cmd.resize(ID_WRITE_REG_LEN);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = ID_WRITE_REG_LEN;
    cmd[3] = ID_WRITE_REG;
    cmd[4] = (uint16_t)reg_addr & 0xFF;
    cmd[5] = ((uint16_t)reg_addr>>8) & 0xFF;
    cmd[6] = (uint8_t)reg_val;
    cmd[7] = calSum(cmd.mid(0,ID_WRITE_REG_LEN-1));

    serialSendCmd(cmd);
}

void SerialPortHandler::devLedEnable(int en)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x06;
    cmd[3] = ID_LED_ENABLE;
    cmd[4] = en;
    cmd[5] = calSum(cmd.mid(0,5));
    serialSendCmd(cmd);
}

void SerialPortHandler::devBtTestMode(int en)
{
    QByteArray cmd;
    cmd.resize(6);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x06;
    cmd[3] = ID_BT_TEST_MODE;
    cmd[4] = en;
    cmd[5] = calSum(cmd.mid(0,5));
    serialSendCmd(cmd);
}

void SerialPortHandler::devBtRssiTh(int lock_rssi, int unlock_rssi)
{
    QByteArray cmd;
    cmd.resize(7);
    cmd[0] = 0x8F;
    cmd[1] = 0xD4;
    cmd[2] = 0x07;
    cmd[3] = ID_BT_RSSI_TH;
    cmd[4] = lock_rssi;
    cmd[5] = unlock_rssi;
    cmd[6] = calSum(cmd.mid(0,6));
    serialSendCmd(cmd);
    qDebug() << cmd.toHex();
}


