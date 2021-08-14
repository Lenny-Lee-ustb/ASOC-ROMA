#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include "ftd2xx.h"
#include "libft4222.h"
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#define numOfSingle 4
#define numOfMulti 4

ros::Publisher SingleAnglePub;
ros::Publisher MultiAnglePub;
ros::Publisher MultiTurnPub;
ros::Publisher Dir;
ros::Publisher OrianglePub;   //publish the original angle
ros::Publisher MultiAngleSumPub;

std_msgs::Float32MultiArray SingleAngleMsg,MultiAngleMsg,MultiTurnMsg,DirMsg,Orianglemsg,MultiAngleSumMSg;


FT_HANDLE ftHandle[4];
FT4222_STATUS ft4222Status;
FT_STATUS ftStatus;

int count;

struct encoderSingleTurn
{
    uint8 sendData[1] = {0x05};
    uint8 recvData[4];
    uint16 sizeTransferred;
    uint16 angle_16bit;
    double angle_deg;
    double angle_rad;
};
encoderSingleTurn enST[numOfSingle];

struct encoderMultiTurn
{
    uint8 sendData[2] = {0x20,0x00};
    uint8 sendData2[2] = {0x2c,0x00};
    uint8 recvData[2];
    uint16 sizeTransferred;

    uint16 angle_12bit;
    double angle_deg;
    double angle_deg_last;
    double angle_rad;

    int16 turn_12bit;
    double turn;
    float dir;
    float angle_sum;
    float turn_count;
};
encoderMultiTurn enMT[numOfMulti];

void signalCallback(int signum){
    printf("Caught signal %d\n", signum);
    exit(0);
    for (int i = 0;i<4;i++){
        FT4222_UnInitialize(ftHandle[i]);
        FT_Close(ftHandle[i]);
    }
    exit(0);
}

void FT4222Init(){
    
    FT_DEVICE_LIST_INFO_NODE *devInfo = NULL;
    DWORD numDevs = 0;
    ftStatus = FT_CreateDeviceInfoList(&numDevs);
    if (ftStatus != FT_OK) 
    {
        printf("FT_CreateDeviceInfoList failed (error code %d)\n", 
            (int)ftStatus);
        exit(1);
    }
    if (numDevs == 0)
    {
        printf("No devices connected.\n");
        exit(1);
    }
}

void spiInit()
{
    for (int id = 0;id<4;id++){
        ftStatus = FT_Open(id,&ftHandle[id]);
        if (ftStatus != FT_OK)
        {
            printf("FT_Open failed (error %d)\n", 
                (int)ftStatus);
            exit(1);
        }
        ftStatus =  FT4222_SetClock(ftHandle[id], SYS_CLK_24);

        ft4222Status = FT4222_SPIMaster_Init(
                            ftHandle[id], 
                            SPI_IO_SINGLE, // 1 channel
                            CLK_DIV_32, // 24 MHz / 512 == 48kHz
                            CLK_IDLE_HIGH, // clock idles at logic 0
                            CLK_TRAILING, // data captured on rising edge
                            1); 
        if (FT4222_OK != ft4222Status)
        {
            printf("FT4222_SPIMaster_Init failed (error %d)\n",
                (int)ft4222Status);
            exit(1);
        }

        ft4222Status = FT4222_SPI_SetDrivingStrength(ftHandle[id],
                                                    DS_8MA,
                                                    DS_8MA,
                                                    DS_8MA);
        if (FT4222_OK != ft4222Status)
        {
            printf("FT4222_SPI_SetDrivingStrength failed (error %d)\n",
                (int)ft4222Status);
            exit(1);
        }
    }
    

}

void SPI_CS_DESELECTALL(void)
{
    for (int i = 0;i<4;i++){ft4222Status = FT4222_SPIMaster_SetCS(ftHandle[i], CS_ACTIVE_POSTIVE);}
    
}

void SPI_CS_SELECT(int id)
{
    ft4222Status = FT4222_SPIMaster_SetCS(ftHandle[id], CS_ACTIVE_NEGTIVE);
}

void recvData2enST(encoderSingleTurn &enST){
    enST.angle_16bit = (((uint16)enST.recvData[0] << 8) + enST.recvData[1]);
    enST.angle_rad = enST.angle_16bit/65536.0*M_PI;
    enST.angle_deg = enST.angle_16bit/65536.0*360;  
}

void recvData2enMTAngle(encoderMultiTurn &enMT){
    enMT.angle_12bit = (((uint16)(enMT.recvData[0]&0x0f)<<8)+enMT.recvData[1]);
    enMT.angle_rad = enMT.angle_12bit/4096.0*M_PI;
    enMT.angle_deg = enMT.angle_12bit/4096.0*360;
}

void recvData2enMTTurns(encoderMultiTurn &enMT){
    int16 op = (0x07<<8)+0xff;
    int dir;
    enMT.turn_12bit = (((uint16)(enMT.recvData[0]&0x0f)<<8)+enMT.recvData[1]);
    if ((enMT.turn_12bit>>11 == 1)) dir = -1;enMT.turn_12bit = ((~(enMT.turn_12bit))&op);
    if ((enMT.turn_12bit>>11 == 0)) dir = 1;
    enMT.turn = dir*enMT.turn_12bit/8.0;
}
void recvData2enMTTurns2(encoderMultiTurn &enMT){
    enMT.turn_12bit = (((uint16)(enMT.recvData[0]&0x0f)<<8)+enMT.recvData[1]);
    if ((enMT.turn_12bit>>11 == 1)){
        enMT.dir = -1.0;
    }
    if ((enMT.turn_12bit>>11 == 0)){
        enMT.dir = 1.0;
    }
    if ((enMT.turn_12bit == 2048)){
        enMT.turn_12bit = -2048;
        
    } 
    if ((enMT.turn_12bit>>11 == 1)){
        enMT.turn_12bit = -(~(enMT.turn_12bit))-4096;
    
    }
    
    enMT.turn = enMT.turn_12bit/8.0;
}

void printanglesum(){
    //printf("angle_sum are ");
    //for (int i = 0;i<numOfSingle;i++) 
    ROS_INFO("ANGLE SUM are %f, %f, %f, %f ;",enMT[0].angle_sum, enMT[1].angle_sum, enMT[2].angle_sum, enMT[3].angle_sum);
    //printf("%f ;",enST[2].angle_deg);
    //printf("\n");
}

void printenST(){
    printf("enST are ");
    for (int i = 0;i<numOfSingle;i++) printf("%f ;",enST[i].angle_deg);
    //printf("%f ;",enST[2].angle_deg);
    printf("\n");
}

void printenMTAngle(){
    //printf("enMT Angle are ");
    //for (int i = 0;i<numOfMulti;i++) 
    ROS_INFO("enMT ANGLE are %f, %f, %f, %f ;",enMT[0].angle_deg, enMT[1].angle_deg, enMT[2].angle_deg, enMT[3].angle_deg);
    //printf("%f ;",enMT[2].angle_deg);
    //printf("\n");
}

void printenMTTurn(){
    //printf("enMT Turn are ");
    //for (int i = 0;i<numOfMulti;i++) 
    ROS_INFO("enMT TURN are %f, %f, %f, %f ;",enMT[0].turn, enMT[1].turn, enMT[2].turn, enMT[3].turn);
    //printf("%f ;",enMT[2].turn);
    //printf("\n");
}

void printturncount(){
    //printf("enMT turn count are ");
    //for (int i = 0;i<numOfMulti;i++) 
    ROS_INFO("enMT TURN_COUNT are %f, %f, %f, %f ;",enMT[0].turn_count, enMT[1].turn_count, enMT[2].turn_count, enMT[3].turn_count);
    //printf("%f ;",enMT[2].turn);
    //printf("\n");
}

void printfBitData(encoderMultiTurn enMT)
{
	for(int i = 0;i<2;i++)
	{
		printf("%d is %x;",i,enMT.recvData[i]);
	}
}

void printDir(){
    //printf("enMT direction is");
    //for(int i=0; i<numOfSingle; i++) 
    ROS_INFO("enMT DIRECTION is %f, %f, %f, %f ;",enMT[0].dir, enMT[1].dir, enMT[2].dir, enMT[3].dir);
    //printf("%f ;",enMT[2].dir);
    //printf("\n");
}

void spiWriteReadSingleTurn(int id)
{
    SPI_CS_DESELECTALL();
    SPI_CS_SELECT(id);
    ft4222Status = FT4222_SPIMaster_SingleWrite(ftHandle[id], &enST[id].sendData[0], 1, &enST[id].sizeTransferred, FALSE);
    if (FT4222_OK != ft4222Status)
    {
        // spi master write failed
        printf("spi master write failed");
        return;
    }

    ft4222Status = FT4222_SPIMaster_SingleRead(ftHandle[id], &enST[id].recvData[0], 4, &enST[id].sizeTransferred, TRUE);
    if (FT4222_OK != ft4222Status)
    {
        // spi master read failed
        printf("spi master read failed\n");
        return;
    }
    recvData2enST(enST[id]);
    
}

void spiWriteReadMultiTurn(int id){
    SPI_CS_DESELECTALL();
    SPI_CS_SELECT(id);
    
    ft4222Status = FT4222_SPIMaster_SingleReadWrite(ftHandle[id], &enMT[id].recvData[0], &enMT[id].sendData[0], 2, &enMT[id].sizeTransferred, TRUE);
    if((ft4222Status!=FT4222_OK) || (enMT[id].sizeTransferred!=2)){
        printf("single read write failed\n");
        exit(1);
    }
    ft4222Status = FT4222_SPIMaster_SingleReadWrite(ftHandle[id], &enMT[id].recvData[0], &enMT[id].sendData[0], 2, &enMT[id].sizeTransferred, TRUE);
    if((ft4222Status!=FT4222_OK) || (enMT[id].sizeTransferred!=2)){
        printf("single read write failed\n");
        exit(1);
    }
    recvData2enMTAngle(enMT[id]);

    ft4222Status = FT4222_SPIMaster_SingleReadWrite(ftHandle[id], &enMT[id].recvData[0], &enMT[id].sendData2[0], 2, &enMT[id].sizeTransferred, TRUE);
    if((ft4222Status!=FT4222_OK) || (enMT[id].sizeTransferred!=2)){
        
        printf("single read write failed\n");
        exit(1);
    }
    ft4222Status = FT4222_SPIMaster_SingleReadWrite(ftHandle[id], &enMT[id].recvData[0], &enMT[id].sendData2[0], 2, &enMT[id].sizeTransferred, TRUE);
    if((ft4222Status!=FT4222_OK) || (enMT[id].sizeTransferred!=2)){
        
        printf("single read write failed\n");
        exit(1);
    }
    recvData2enMTTurns2(enMT[id]);
}

void getDateSingleTurn(){
    while(1){
        for (int i = 0;i<numOfSingle;i++) {spiWriteReadSingleTurn(i);SingleAngleMsg.data[i] = enST[i].angle_deg;}
        usleep(100);
        printenST();
        SingleAnglePub.publish(SingleAngleMsg);
    }
}

void getOriangle(){
    for (int i = 0;i<numOfMulti;i++) {
            spiWriteReadMultiTurn(i);
            Orianglemsg.data[i] = enMT[i].angle_deg;
    }
    OrianglePub.publish(Orianglemsg);

}

void getDateMultiTurn(){
    while(1){
        for (int i = 0;i<numOfMulti;i++) {
            spiWriteReadMultiTurn(i);
            MultiAngleMsg.data[i] = enMT[i].angle_deg;
            //ROS_INFO("sub=(%.2f)",enMT[1].angle_deg-enMT[1].angle_deg_last);
            if(enMT[i].angle_deg - enMT[i].angle_deg_last > 300){
                enMT[i].turn_count--;
            }
            else if(enMT[i].angle_deg - enMT[i].angle_deg_last < -300){
                enMT[i].turn_count ++;
            }
            MultiTurnMsg.data[i] = float(enMT[i].turn);
            DirMsg.data[i] = enMT[i].dir;
            enMT[i].angle_deg_last = enMT[i].angle_deg;
            if(enMT[i].turn >= 0){
                enMT[i].angle_sum = enMT[i].turn_count*360 + enMT[i].angle_deg;
            }
            else{
                enMT[i].angle_sum = enMT[i].turn_count *360 + enMT[i].angle_deg;
            }
            MultiAngleSumMSg.data[i]=enMT[i].angle_sum;
        }
        if(count==1000){
            ros::param::set("ori0",enMT[0].angle_deg);
            ros::param::set("ori1",enMT[1].angle_deg);
            ros::param::set("ori2",enMT[2].angle_deg);
            ros::param::set("ori3",enMT[3].angle_deg);
        }
        usleep(100);
        printenMTAngle();
        printanglesum();
        printturncount();
        // printfBitData(enMT[0]);
        printenMTTurn();
        printDir();
        MultiAnglePub.publish(MultiAngleMsg);
        MultiTurnPub.publish(MultiTurnMsg);
        Dir.publish(DirMsg);
        MultiAngleSumPub.publish(MultiAngleSumMSg);
        count++;


    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"demo1");
    ros::NodeHandle n;

    SingleAnglePub = n.advertise<std_msgs::Float32MultiArray>("SingleAngle",1);
    MultiAnglePub = n.advertise<std_msgs::Float32MultiArray>("MultiAngle",1);
    MultiTurnPub = n.advertise<std_msgs::Float32MultiArray>("MultiTurn",1);
    Dir = n.advertise<std_msgs::Float32MultiArray>("Dir",1);
    OrianglePub = n.advertise<std_msgs::Float32MultiArray>("Oriangle",1);
    MultiAngleSumPub = n.advertise<std_msgs::Float32MultiArray>("MultiAngleSum",1);

    SingleAngleMsg.data.resize(numOfSingle);
    MultiAngleMsg.data.resize(numOfMulti);
    MultiTurnMsg.data.resize(numOfMulti);
    DirMsg.data.resize(numOfMulti);
    Orianglemsg.data.resize(numOfMulti);
    MultiAngleSumMSg.data.resize(numOfMulti);

    FT4222Init();
    spiInit();
    signal(SIGINT, signalCallback);
    //getDateSingleTurn();//单圈
    //getOriangle();
    getDateMultiTurn();//多圈
    return 0;
}