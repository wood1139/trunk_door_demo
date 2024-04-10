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

typedef struct
{
    uint16_t gnd_stable_th_mm;       // 判定地面稳定的极差阈值
    uint16_t foot_stable_th_mm;      // 判定脚稳定的极差阈值
    uint16_t valid_foot_th_min_mm;   // 识别脚的最小高度阈值
    uint16_t valid_foot_th_max_mm;   // 识别脚的最大高度阈值
    uint16_t data_win_size;          // 用于判定的数据长度
}FootDetectParaStruct;


enum FrameIdEnum
{
    ID_GET_VERSION              = 0x01, ID_GET_VERSION_LEN              = 5,   ID_GET_VERSION_LEN_ACK             = 31,
    ID_SOFT_RESET               = 0x02, ID_SOFT_RESET_LEN               = 5,   ID_SOFT_RESET_LEN_ACK              = 6,
    ID_SET_VI4302_MODE          = 0x03, ID_SET_VI4302_MODE_LEN          = 6,   ID_SET_VI4302_MODE_LEN_ACK         = 6,
    ID_HARDLINE_CONFIG          = 0x04, ID_HARDLINE_CONFIG_LEN          = 9,   ID_HARDLINE_CONFIG_LEN_ACK         = 9,
    ID_RANGING_ENABLE           = 0x05, ID_RANGING_ENABLE_LEN           = 6,   ID_RANGING_ENABLE_LEN_ACK          = 6,
    ID_SAMPLE_RATE              = 0x06, ID_SAMPLE_RATE_LEN              = 7,   ID_SAMPLE_RATE_LEN_ACK             = 7,
    ID_LD_TRIG_PWIDTH           = 0x07, ID_LD_TRIG_PWIDTH_LEN           = 7,   ID_LD_TRIG_PWIDTH_LEN_ACK          = 7,
    ID_FOOT_DETECT_PARA         = 0x08, ID_FOOT_DETECT_PARA_LEN         = 15,  ID_FOOT_DETECT_PARA_LEN_ACK        = 15,
    ID_WALK_ERR_K               = 0x09, ID_WALK_ERR_K_LEN               = 7,   ID_WALK_ERR_K_LEN_ACK              = 7,
    ID_DIST_OFFSET              = 0x0A, ID_DIST_OFFSET_LEN              = 7,   ID_DIST_OFFSET_LEN_ACK             = 7,
    ID_LOW_PEAK_TH              = 0x0B, ID_LOW_PEAK_TH_LEN              = 7,   ID_LOW_PEAK_TH_LEN_ACK             = 7,
    ID_DIRTY_DIST_TH            = 0x0C, ID_DIRTY_DIST_TH_LEN            = 6,   ID_DIRTY_DIST_TH_LEN_ACK           = 6,
    ID_LD_TRIG_NUM              = 0x0D, ID_LD_TRIG_NUM_LEN              = 7,   ID_LD_TRIG_NUM_LEN_ACK             = 7,
    ID_READ_REG                 = 0x0E, ID_READ_REG_LEN                 = 7,   ID_READ_REG_LEN_ACK                = 8,
    ID_WRITE_REG                = 0x0F, ID_WRITE_REG_LEN                = 8,   ID_WRITE_REG_LEN_ACK               = 8,
    ID_RESTORE_DEFAULT          = 0x10, ID_RESTORE_DEFAULT_LEN          = 5,   ID_RESTORE_DEFAULT_LEN_ACK         = 6,
    ID_SAVE_SETTINGS            = 0x11, ID_SAVE_SETTINGS_LEN            = 5,   ID_SAVE_SETTINGS_LEN_ACK           = 6,
    ID_FLASH_ERASE              = 0x12, ID_FLASH_ERASE_LEN              = 6,   ID_FLASH_ERASE_LEN_ACK             = 6,
    ID_FLASH_BACKUP             = 0x13, ID_FLASH_BACKUP_LEN             = 13,  ID_FLASH_BACKUP_LEN_ACK            = 6,
    ID_READ_ALL_PARAMS          = 0x14, ID_READ_ALL_PARAMS_LEN          = 6,   ID_READ_ALL_PARAMS_LEN_ACK         = 6,
    ID_BVD_CALIB                = 0x20, ID_BVD_CALIB_LEN                = 5,   ID_BVD_CALIB_LEN_ACK               = 7,
    ID_LED_ENABLE               = 0x21, ID_LED_ENABLE_LEN               = 6,   ID_LED_ENABLE_LEN_ACK              = 6,
};

enum DataIdEnum
{
	DATA_ID_HISTOGRAM      = 0xA1, DATA_ID_HISTOGRAM_LEN      = 134,
	DATA_ID_SINGLE_PIX     = 0xA2, DATA_ID_SINGLE_PIX_LEN     = 55,
	DATA_ID_RANGE_RAW      = 0xA3, DATA_ID_RANGE_RAW_LEN      = 34,
	DATA_ID_DETECT_STATUS  = 0xA4, DATA_ID_DETECT_STATUS_LEN  = 6,
};

typedef enum
{
    VI4302_MODE_RANGE = 0,
    VI4302_MODE_SINGLE_PIX,
    VI4302_MODE_HISTOTRAM
}SysVi4302ModeEnum;

typedef enum
{
    STATUS_TRIG = 0,
}SysDetectStatusEnum;

typedef struct
{
    int               vi4302_mode;
    uint8_t           is_feet_detect;
    uint8_t           hardline_pin_sel;        // hardline pin select: 0-none, 1-uart tx, 2-led1, 3-led2
    uint8_t           is_ranging_enable;
    uint8_t           hardline_pin_mode;       // open-drain or push-pull
    int32_t           save_config_cnt;         // 保存配置指令计数
    int32_t           restore_default_cnt;     // 恢复出厂配置计数
    uint16_t          ld_trig_pwidth_100ps;    // laser triger pulse width, unit 0.1ns, 0.3ns-12.8ns
    uint16_t          sample_rate;             // 20Hz-4kHz
    uint16_t          hardline_pulse_ms;       // hardline mode pulse signal width in ms
    uint16_t          gnd_stable_th_mm;        // 判定地面稳定的极差阈值
    uint16_t          foot_stable_th_mm;       // 判定脚稳定的极差阈值
    uint16_t          valid_foot_th_min_mm;    // 识别脚的最小高度阈值
    uint16_t          valid_foot_th_max_mm;    // 识别脚的最大高度阈值
    uint16_t          data_win_size;           // 用于判定的数据长度
    uint16_t          walk_err_k;              // 行走误差线性系数，err_mm = -walk_err_k * atten_peak / 10000
    uint16_t          dist_offset_mm;          // 距离偏置
    uint16_t          low_peak_th;             // 当atten_peak小于low_peak_th时，认为测距无效
    uint8_t           dirty_dist_th_mm;        // 当测距值小于dirty_dist_th_mm时，判断近距离遮挡或脏污
    uint8_t           vi4302_bvd_val;          // 初次上电标定的工作电压
    uint8_t           vi4302_tdc_val;          // 初次上电标定的TDC参数
    int8_t            vi4302_calib_tmpr;       // 标定时的温度，单位℃
    uint16_t          vi4302_pulse_num;        // 一次探测的打光次数
    uint8_t           led_enable;              // 投影光使能
    uint8_t           dummy;
}SysConfigStruct;


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
    void startRecord(QString filenamePrefix, int mode, QList<int> pulseNumList, int atBvd, QList<int> bvdList, int atPulseNum, int frameNum);
    void stopRecord();
    bool isRecording();

    void devSetVi4302Mode(int mode);
    void devSetHardLineConfig(int pin_sel, int pin_mode, int pwidth_ms);
    void devSetSampleRate(int sample_rate);
    void devSetLdTrigPwidth(int pwidth);
    void devSetLdTrigNum(int trig_num);
    void devSoftReset();
    void devReadFirmwareVersion();
    void devRangingEnable(int en);
    void devSaveConfig();
    void devSetFootDetectPara(FootDetectParaStruct para);
    void devReadAllPara();
    void devSetWalkErrK(int k);
    void devSetDistOffset(int offset);
    void devSetLowPeakTh(int th);
    void devSetDirtyDistTh(int th);
    void devEraseFlash();
    void devBvdCalib();
    void devReadReg(int reg_addr);
    void devWriteReg(int reg_addr, int reg_val);
    void devLedEnable(int en);

private:
    uint8_t calSum(QByteArray data);
    void scheduleRecord();
    void handleData();

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

    QString m_filenamePrefix;
    QList<int> m_pulseNumList;
    int m_atBvd;
    QList<int> m_bvdList;
    int m_atPulseNum;
    int m_frameNum;
    int m_pulseNumIdx;
    int m_bvdIdx;
    int m_frameCnt;


signals:
    void sigLidarData(QByteArray frameData);
    void sigSetAxisRange(int xmin, int xmax, int ymin, int ymax);
    void sigRecordStop();
};

#endif // SERIALPORTHANDLER_H
