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
    uint32_t norm_tof;
    uint32_t norm_peak;
    uint32_t norm_noise;
    uint32_t int_num;
    uint32_t atten_peak;
    uint32_t atten_noise;
    uint32_t ref_tof;
    int32_t  temp_sensor_x100;
    int32_t  temp_mcu_x100;
    uint32_t raw_tof_mm;
    uint32_t ctof;
    uint32_t confidence;
    uint32_t timestamp_ms;
    uint32_t xtalk_count;
}RawDataStruct;

typedef struct
{
    uint16_t gnd_stable_th_mm;       // 判定地面稳定的极差阈值
    uint16_t foot_stable_th_mm;      // 判定脚稳定的极差阈值
    uint16_t valid_foot_th_min_mm;   // 识别脚的最小高度阈值
    uint16_t valid_foot_th_max_mm;   // 识别脚的最大高度阈值
    uint16_t data_win_size;          // 用于判定的数据长度
    uint16_t foot_in_hold_max_times; // foot in状态最长持续时间，超过这个时间就强制切换回地面状态
    uint16_t foot_pre_peak_mm;       // 脚踏稳定之前，必须有高于脚面的过程
    uint16_t foot_pre_peak_win_size; // 脚踏稳定之前寻找pre_peak_mm的范围
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
    ID_FOOT_DETECT_PARA         = 0x08, ID_FOOT_DETECT_PARA_LEN         = 21,  ID_FOOT_DETECT_PARA_LEN_ACK        = 21,
    ID_WALK_ERR_K               = 0x09, ID_WALK_ERR_K_LEN               = 7,   ID_WALK_ERR_K_LEN_ACK              = 7,
    ID_DIST_OFFSET              = 0x0A, ID_DIST_OFFSET_LEN              = 7,   ID_DIST_OFFSET_LEN_ACK             = 7,
    ID_LOW_PEAK_TH              = 0x0B, ID_LOW_PEAK_TH_LEN              = 7,   ID_LOW_PEAK_TH_LEN_ACK             = 7,
    ID_DIRTY_TH                 = 0x0C, ID_DIRTY_TH_LEN                 = 10,  ID_DIRTY_TH_LEN_ACK                = 10,
    ID_LD_TRIG_NUM              = 0x0D, ID_LD_TRIG_NUM_LEN              = 9,   ID_LD_TRIG_NUM_LEN_ACK             = 9,
    ID_READ_REG                 = 0x0E, ID_READ_REG_LEN                 = 7,   ID_READ_REG_LEN_ACK                = 8,
    ID_WRITE_REG                = 0x0F, ID_WRITE_REG_LEN                = 8,   ID_WRITE_REG_LEN_ACK               = 8,
    ID_RESTORE_DEFAULT          = 0x10, ID_RESTORE_DEFAULT_LEN          = 5,   ID_RESTORE_DEFAULT_LEN_ACK         = 6,
    ID_SAVE_SETTINGS            = 0x11, ID_SAVE_SETTINGS_LEN            = 5,   ID_SAVE_SETTINGS_LEN_ACK           = 6,
    ID_FLASH_ERASE              = 0x12, ID_FLASH_ERASE_LEN              = 6,   ID_FLASH_ERASE_LEN_ACK             = 6,
    ID_FLASH_BACKUP             = 0x13, ID_FLASH_BACKUP_LEN             = 13,  ID_FLASH_BACKUP_LEN_ACK            = 6,
    ID_READ_ALL_PARAMS          = 0x14, ID_READ_ALL_PARAMS_LEN          = 6,   ID_READ_ALL_PARAMS_LEN_ACK         = 6,
    ID_SET_JTX_WORK_MODE        = 0x15, ID_SET_JTX_WORK_MODE_LEN        = 6,   ID_SET_JTX_WORK_MODE_LEN_ACK       = 6,
    ID_WRITE_SN                 = 0x16, ID_WRITE_SN_LEN                 = 21,  ID_WRITE_SN_LEN_ACK                = 6,
    ID_READ_ALL_PARAMS_PACK     = 0x17, ID_READ_ALL_PARAMS_PACK_LEN     = 7,   ID_READ_ALL_PARAMS_PACK_LEN_ACK    = 6,
    ID_BVD_CALIB                = 0x20, ID_BVD_CALIB_LEN                = 5,   ID_BVD_CALIB_LEN_ACK               = 7,
    ID_LED_ENABLE               = 0x21, ID_LED_ENABLE_LEN               = 6,   ID_LED_ENABLE_LEN_ACK              = 6,
    ID_BT_TEST_MODE             = 0x22, ID_BT_TEST_MODE_LEN             = 6,   ID_BT_TEST_MODE_LEN_ACK            = 6,
    ID_BT_RSSI_TH               = 0x23, ID_BT_RSSI_TH_LEN               = 7,   ID_BT_RSSI_TH_LEN_ACK              = 6,
    ID_XTALK_CALIB              = 0x24, ID_XTALK_CALIB_LEN              = 5,   ID_XTALK_CALIB_LEN_ACK             = 10,
    ID_OFFSET_CALIB             = 0x25, ID_OFFSET_CALIB_LEN             = 7,   ID_OFFSET_CALIB_LEN_ACK            = 8,
    ID_LED_BREATH_PARA          = 0x26, ID_LED_BREATH_PARA_LEN          = 17,  ID_LED_BREATH_PARA_LEN_ACK         = 17,
    ID_DEBUG_OUTPUT_ENABLE      = 0x27, ID_DEBUG_OUTPUT_ENABLE_LEN      = 6,   ID_DEBUG_OUTPUT_ENABLE_LEN_ACK     = 6,
    ID_LED_AUTO_JUST            = 0x28, ID_LED_AUTO_JUST_LEN            = 11,  ID_LED_AUTO_JUST_LEN_ACK           = 11,
    ID_HIGH_TMPR_PROTECT        = 0x29, ID_HIGH_TMPR_PROTECT_LEN        = 8,   ID_HIGH_TMPR_PROTECT_LEN_ACK       = 8,
    ID_BT_REBOOT                = 0x2A, ID_BT_REBOOT_LEN                = 5,   ID_BT_REBOOT_LEN_ACK               = 6,
    ID_ACTIVE_TIMEOUT           = 0x2B, ID_ACTIVE_TIMEOUT_LEN           = 9,   ID_ACTIVE_TIMEOUT_LEN_ACK          = 9,
    ID_BT_RESET_FACTORY         = 0x2C, ID_BT_RESET_FACTORY_LEN         = 5,   ID_BT_RESET_FACTORY_LEN_ACK        = 6,
    ID_BT_SET_PASSWORD          = 0x2D, ID_BT_SET_PASSWORD_LEN          = 11,  ID_BT_SET_PASSWORD_LEN_ACK         = 6,
    ID_BT_HOLD_TIME             = 0x2E, ID_BT_HOLD_TIME_LEN             = 7,   ID_BT_HOLD_TIME_LEN_ACK            = 7,
    ID_WATCHDOG_TEST            = 0x2F, ID_WATCHDOG_TEST_LEN            = 13,  ID_WATCHDOG_TEST_LEN_ACK           = 6,
    ID_READ_SYS_STATUS          = 0x30, ID_READ_SYS_STATUS_LEN          = 5,   ID_READ_SYS_STATUS_LEN_ACK         = 21,
    ID_BVD_TMPR_K               = 0x31, ID_BVD_TMPR_K_LEN               = 9,   ID_BVD_TMPR_K_LEN_ACK              = 9,
    ID_NEAR_RANGE_TAB           = 0x32, ID_NEAR_RANGE_TAB_LEN           = 20,  ID_NEAR_RANGE_TAB_LEN_ACK          = 20,
    ID_DIST_CORR_PARA           = 0x33, ID_DIST_CORR_PARA_LEN           = 35,  ID_DIST_CORR_PARA_LEN_ACK          = 35,
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

#define DIST_CORR_TABLE_LEN    20
typedef struct
{
    int               vi4302_mode;
    uint8_t           is_feet_detect;          // dummy
    uint8_t           hardline_pin_sel;        // dummy
    uint8_t           is_ranging_enable;
    uint8_t           hardline_pin_mode;       // dummy
    int32_t           save_config_cnt;         // 保存配置指令计数
    int32_t           restore_default_cnt;     // 恢复出厂配置计数
    uint16_t          ld_trig_pwidth_100ps;    // laser triger pulse width, unit 0.1ns, 0.3ns-12.8ns
    uint16_t          sample_rate;             // 20Hz-4kHz
    uint16_t          hardline_pulse_ms;       // dummy hardline mode pulse signal width in ms
    uint16_t          gnd_stable_th_mm;        // dummy 判定地面稳定的极差阈值
    uint16_t          foot_stable_th_mm;       // dummy 判定脚稳定的极差阈值
    uint16_t          valid_foot_th_min_mm;    // dummy 识别脚的最小高度阈值
    uint16_t          valid_foot_th_max_mm;    // dummy 识别脚的最大高度阈值
    uint16_t          data_win_size;           // dummy 用于判定的数据长度
    uint16_t          walk_err_k;              // dummy
    int16_t           dist_offset_mm;          // 距离偏置
    uint16_t          low_peak_th;             // 当atten_peak小于low_peak_th时，认为测距无效
    uint8_t           dirty_dist_th_mm;        // 当测距值小于dirty_dist_th_mm时，判断近距离遮挡或脏污
    uint8_t           vi4302_bvd_val;          // 初次上电标定的工作电压
    uint8_t           vi4302_tdc_val;          // 初次上电标定的TDC参数
    int8_t            vi4302_calib_tmpr;       // 标定时的温度，单位℃
    uint16_t          hardline_trig_delay_ms;  // dummy 硬线触发信号延时
    uint8_t           led_enable;              // dummy 投影光使能
    uint8_t           jtx_work_mode;           // dummy 整机工作模式
    uint16_t          foot_in_hold_max_times;  // dummy foot in状态最长持续时间，超过这个时间就强制切换回地面状态
    uint8_t           bt_test_mode;            // dummy 蓝牙调试模式：0-蓝牙处于低功耗工作模式，1-蓝牙响应AT指令，调试串口输出蓝牙RSSI
    int8_t            bt_lock_rssi;            // dummy 蓝牙上锁强度值
    int8_t            bt_unlock_rssi;          // dummy 蓝牙解锁强度值
    uint8_t           debug_output_enable;     // 数据输出使能
    uint32_t          spad_int_num;            // 一次探测的打光次数
    int8_t            VI530x_Cali_CG_Pos;      // dummy 串扰标定位置
    uint8_t           VI530x_Cali_CG_Maxratio; // dummy 串扰标定比例
    uint16_t          VI530x_Cali_CG_Peak;     // dummy 串扰标定峰值
    uint32_t          led_breath_peak_x10000;  // dummy LED呼吸的最大亮度，即PWM占空比
    uint32_t          led_breath_depth_x10000; // dummy LED呼吸深度，0不呼吸，1最亮到最暗
    uint32_t          led_breath_period_ms;    // dummy LED呼吸周期
    char              sn[16];
    uint16_t          foot_pre_peak_mm;        // dummy 脚踏稳定之前，必须有高于脚面的过程
    uint16_t          foot_pre_peak_win_size;  // dummy 脚踏稳定之前寻找pre_peak_mm的范围
    uint16_t          led_max_noise;           // dummy 自动亮度调节模式下，将亮度调到最亮时的最低noise值，在这个noise之上，就可以用最高亮度了。
    uint8_t           led_min_peak_x100;       // dummy 0~1。自动亮度调节模式下的最低亮度，即0lux时对应的亮度。
    uint8_t           led_auto_just_enable;    // dummy 投影亮度自动调节使能
    uint8_t           high_tmpr_protect_enable;// dummy 高温保护使能
    uint8_t           high_tmpr_1;             // dummy 摄氏度。高温保护等级1，高于次温度时，降低LED亮度为led_min_peak_x100
    uint8_t           high_tmpr_2;             // dummy 摄氏度。高温保护等级2，高于次温度时，投影和TOF都停止工作
    uint8_t           dummy1;
    uint32_t          dirty_xtalk_th;          // dummy 当xtalk_count超过此值时，判断近距离遮挡或脏污
    uint32_t          active_timeout_ms;       // dummy 工作状态的超时时间
    uint16_t          bt_hold_time_ms;         // dummy 蓝牙信号断开后，LED保持的时间
    uint16_t          led_min_noise;           // dummy 自动亮度调节模式下，将亮度调到最暗时的noise值
    int32_t           bvd_tmpr_k_x10000;       // bvd温漂系数
    int16_t           nearRangeCorrTable[DIST_CORR_TABLE_LEN][7]; // 近距离修正表，7列分别为：距离点、10%板强度、90%板强度、高反板强度、10%板误差、90%板误差、高反板误差
    int16_t           norm_peak_err_p[2];      // norm peak行走误差修正系数，小信号使用
    int16_t           atten_peak_err_p1[2];    // atten peak行走误差修正系数，大信号第一段
    int16_t           atten_peak_err_p2[2];    // atten peak行走误差修正系数，大信号第二段
    uint16_t          peak_transition_zone[2]; // 大小信号的切换区间
    uint16_t          dist_transition_zone[2]; // 远近距离的切换区间
    int16_t           dist_tmpr_drift_para[2]; // 距离温漂参数，二次项系数和一次项系数，x1000
}SysConfigStruct;


class SerialPortHandler : public QObject
{
    Q_OBJECT

public:
    explicit SerialPortHandler(QObject *parent = nullptr);
    bool connectCom(QString portName, int baudrate);
    bool connectComLin(QString portName, int baudrate);
    void disconnectCom();
    void disconnectComLin();
    QStringList scanComList();
    bool isConnected();
    bool isConnectedLin();

    void setDataPtr(QLineSeries *LinePtr, QStandardItemModel *tabPtr);
    void startRecord(QString filenamePrefix, int mode, QList<int> pulseNumList, int frameNum);
    void stopRecord();
    bool isRecording();

    void serialSendCmd(QByteArray cmd);
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
    void devSetJtxWorkMode(int mode);
    void devWriteSn(char sn[16]);
    void devSetWalkErrK(int k);
    void devSetDistOffset(int offset);
    void devSetLowPeakTh(int th);
    void devSetDirtyTh(int dist_th, int xtalk_th);
    void devEraseFlash();
    void devBvdCalib();
    void devReadReg(int reg_addr);
    void devWriteReg(int reg_addr, int reg_val);
    void devLedEnable(int en);
    void devBtTestMode(int en);
    void devBtRssiTh(int lock_rssi, int unlock_rssi);
    void devXtalkCalib();
    void devOffsetCalib(int mm);
    void devSetLedBreathPara(float peak, float depth, int period);
    void devLedAutoJust(int max_noise, float min_peak, int en);
    void devHighTmprProtect(int en, int tmpr1, int tmpr2);
    void devActiveTimeout(int ms);
    void devDebugOutputEnable(int en);
    void devBtReboot();
    void devBtFactoryReset();
    void devDistCorrPara(SysConfigStruct &config);

private:
    uint8_t calSum(QByteArray data);
    void scheduleRecord();
    void handleData();
    void dataForFrameSearch(QByteArray rawData);
    void linPortWrite(QByteArray cmd);
    void printHex(QByteArray cmd);

private slots:
    void handleReadyRead();
    void handleReadyReadLin();
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort m_serialPort;
    QSerialPort m_serialPortLin;
    QByteArray m_readData;
    QByteArray m_readDataLin;
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
    int m_frameNum;
    int m_pulseNumIdx;
    int m_frameCnt;

signals:
    void sigLidarData(QByteArray frameData);
    void sigSetAxisRange(int xmin, int xmax, int ymin, int ymax);
    void sigRecordStop();
    void sigProcDist(int mm);
    void sigSetCenterMarkerPosition(qreal xRatio, qreal yRatio);
};

#endif // SERIALPORTHANDLER_H
