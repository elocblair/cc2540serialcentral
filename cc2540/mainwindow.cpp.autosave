#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtSerialPort>
#include <QDebug>
#include <QIODevice>
#include <QTimer>
#include <QDateTime>

int timesRead = 0;
int parameterDataCount = 0;
int scanResponseCount = 0;
int connectionCount = 0;
int readingCount = 0;
int readingCount2 = 0;
int makeCentral = 0;
int getParam1 = 1;
int getParam2 = 2;
int getParam3 = 3;
int getParam4 = 4;
int discoverDevices = 5;
int connectDevice = 6;
int readSensorData = 7;
//add conditional terminate link
char deviceInit[] = {0x01,0x00,0xFE,0x26,0x08,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
                     0x00,0x00,0x00};
char param1[] = {0x01, 0x31, 0xFE, 0x01, 0x15};
char param2[] = {0x01, 0x31, 0xFE, 0x01, 0x16};
char param3[] = {0x01, 0x31, 0xFE, 0x01, 0x1A};
char param4[] = {0x01, 0x31, 0xFE, 0x01, 0x19};
char deviceDiscovery[] = {0x01, 0x04, 0xFE, 0x03, 0x03, 0x01, 0x00};
char establishLink[] = {0x01, 0x09, 0xFE, 0x09, 0x00, 0x00, 0x01, 0x54, 0x45, 0xC2,0x16,0xE2,
                        0xEC};
char readChar5[] = {0x01, 0xB4, 0xFD, 0x08, 0x00, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xF5,
                    0xFF};
char notificationOn[] = {0x01, 0x92, 0xFD, 0x06, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x00};
char threeToChar1[] = {0x01, 0x92, 0xFD, 0x05, 0x00, 0x00, 0x1E, 0x00, 0x03};
int deviceInitializedFlag = 1;
int getParameterFlag = 0;
int scanningFlag = 0;
int connectingFlag = 0;
int readingFlag = 0;
int char1Write = 0;
qint64 currentMs = 0;
qint64 oldMs = 0;
qint64 interval;
float x1f = 0;
float x2f = 0;
float x3f = 0;
float ax1f = 0;
float ax2f = 0;
float ax3f = 0;
float oldx3f;
float axf =0;
float ayf =0;
float azf =0;
float gxf =0;
float gyf =0;
float gzf =0;
int dataCount = 3;
int waitCount =0;
int sentCount = 0;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serialPort = new QSerialPort();
    QObject::connect(serialPort, SIGNAL(readyRead()), this, SLOT(handleReadyRead()));
    QList<QSerialPortInfo> serialPortInfos = QSerialPortInfo::availablePorts();

    QObject::connect(timer, SIGNAL(timeout()),this, SLOT(readChar()));
    for (int i = 0; i < serialPortInfos.size(); i++){
        //qDebug() << serialPortInfos.at(i).description();
        if (serialPortInfos.at(i).description() == "TI CC2540 USB CDC Serial Port" ){
            //qDebug() << serialPortInfos.at(i).portName();

            serialPort->setPortName("COM11");
            serialPort->setBaudRate(QSerialPort::Baud115200);
        }
    }
       bool opened = serialPort->open(QIODevice::ReadWrite);
              if (!opened)
                  qDebug() << "error opening com6";
              else{
                  qDebug() << "com6 opened";
                  serialPort->clear(QSerialPort::AllDirections);
              }
    switchFxn(makeCentral);

}
void MainWindow::handleReadyRead(){
    QByteArray readRsp;
    QByteArray sensorData;
    QByteArray data;
    QByteArray deviceAddress;
    QByteArray discoveryDoneEvent;
    QByteArray linkEstablished;
    QByteArray notifyEvent;
    data = serialPort->readAll();
    //qDebug() << data;
    char commandSuccess = data.at(5);
    if (commandSuccess == 0x00)
    {
        if (readingFlag){
            if (readingCount == 1){
                oldMs = currentMs;
                currentMs = QDateTime::currentMSecsSinceEpoch();
                if(timesRead != 0){
                    interval = (currentMs - oldMs)-7;
                    notifyEvent[0] = data.at(3);
                    notifyEvent[1] = data.at(4);
                    if (notifyEvent == "\x1B\x05"){
                       //qDebug() << data;
                       //sentCount = data.at(22);
                       //qDebug() << "NUM" << sentCount;
                       // if (dataCount < 32){

                            int16_t ax = (data.at(12)<< 8) | (data.at(11) & 0xff);
                            axf = (float)(ax*0.049);
                            int16_t ay = (data.at(14)<< 8) | (data.at(13) & 0xff);
                            ayf = (float)(ay*0.049);
                            int16_t az = (data.at(16)<< 8) | (data.at(15) & 0xff);
                            azf = (float)(az*0.049);

                            int16_t gx = (data.at(18)<< 8) | (data.at(17) & 0xff);
                            gxf = (float)(gx*0.0625);
                            int16_t gy = (data.at(20)<< 8) | (data.at(19) & 0xff);
                            gyf = (float)(gy*0.0625);
                            int16_t gz = (data.at(22)<< 8) | (data.at(21) & 0xff);
                            gzf = (float)(gz*0.0625);
                            /*for(int i = 11; i<23; i++){
                                uint8_t a = data.at(i);
                                qDebug() << QString::number(a,10);
                            }*/
                            qDebug() << "acc";
                            qDebug() << axf << " x";
                            qDebug() << ayf << " y";
                            qDebug() << azf << " z";
                            qDebug() <<"gyro";
                            qDebug() << gxf << " x";
                            qDebug() << gyf << " y";
                            qDebug() << gzf << " z";
                            uint8_t a = data.at(23);
                            qDebug() << QString::number(a,10);

                   }
                }
                timesRead++;

            }
        }
        if (char1Write){
           // if (waitCount){
                //serialPort->write(notificationOn, sizeof(notificationOn));
                serialPort->write(threeToChar1, sizeof(threeToChar1));
                char1Write = 0;
                readingFlag = 1;

           // }
            //waitCount++;

        }
        if (connectingFlag){
            //qDebug() << "HARAMBE";
            //qDebug() << data;
            if (connectionCount == 2){
                qDebug() << "#CONNECT3D!";
                serialPort->write(notificationOn, sizeof(notificationOn));
                //serialPort->write(threeToChar1, sizeof(threeToChar1));

                connectingFlag = 0;
                char1Write = 1;
                //readingFlag = 1;
                //timer->start(30);

            }
            if (connectionCount == 1){
                //qDebug() << "link established";
                for (int i = 0; i < 2; i++){
                    linkEstablished[i] = data.at(i+3);
                }
                if(linkEstablished == "\x05\x06"){
                    connectionCount++;
                }

            }
            if(connectionCount == 0){
                //qDebug() << "establish Link command success";
                connectionCount++;
            }
        }
        if (scanningFlag){
            //qDebug() << "device scan result";
            //qDebug() << data;
            if (scanResponseCount == 2){
                discoveryDoneEvent[0] = data.at(3);
                discoveryDoneEvent[1] = data.at(4);
                if (discoveryDoneEvent == "\x01\x06"){
                    //establish link to device
                    //qDebug() << "discoverydone";
                    scanningFlag = 0;
                    connectingFlag = 1;
                    switchFxn(connectDevice);
                }
            }
            if (scanResponseCount == 1){
                if(data.size() > 27)
                {
                    for (int i = 0; i < 5; i++){
                        deviceAddress[i] = data.at(i+21);
                    }

                    qDebug() << deviceAddress;
                }
                if (//deviceAddress == "x\x1A\x89q$"){
                        deviceAddress == "JohnC"){
                    scanResponseCount++;
                }

            }
            if (scanResponseCount == 0){
                char command = data.at(3);
                char success = data.at(4);
                if(success == 0x06 && command == 0x7F){
                    scanResponseCount++;
                }
            }



        }
        if (getParameterFlag)
        {
           // qDebug() << "get parameter data";
           // qDebug() << data;
            parameterDataCount++;
            if (parameterDataCount == 4){
                switchFxn(discoverDevices);
                getParameterFlag = 0;
                scanningFlag = 1;
            }
        }
        if (deviceInitializedFlag){
           // qDebug() << "device initialized response data";
           // qDebug() << data;
            char initCommand = data.at(13);
            char initSuccess = data.at(14);
            if(initSuccess == 0x00 && initCommand == 0x06){
                deviceInitializedFlag = 0;
                getParameterFlag = 1;
                switchFxn(getParam1);
            }
        }



    }
    else //if (commandSuccess == 0x1A) // anything that isn't 00 on .at(5)
    {
        qDebug() << "aha!";
        readingCount = 1;
        serialPort->write(threeToChar1, sizeof(threeToChar1));
        qDebug() << data;
    }
}

MainWindow::~MainWindow() 
{
    delete ui;
}

void MainWindow::switchFxn(int step){
    switch( step )
    {
        case (0):
            serialPort->write(deviceInit,sizeof(deviceInit));
            break;
        case (1):
            serialPort->write(param1,sizeof(param1));
            serialPort->write(param2,sizeof(param2));
            serialPort->write(param3,sizeof(param3));
            serialPort->write(param4,sizeof(param4));

        break;
        case (5):
            serialPort->write(deviceDiscovery,sizeof(deviceDiscovery));
        break;
        case (6):
            serialPort->write(establishLink,sizeof(establishLink));
        break;
        case (7):
            serialPort->write(readChar5, sizeof(readChar5));
            readingCount = 0;
        break;

    }
}
void MainWindow::readChar(){
    switchFxn(readSensorData);
}
void MainWindow::calcRps(QByteArray data){

    int16_t x1 = (data.at(11) << 8) | (data.at(12) & 0xff);
    x1 = x1/900;
    qDebug() << x1;
}
