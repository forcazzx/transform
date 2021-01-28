#include <iostream>
#include <fstream>
#include <cmath>


using namespace std;
#define R2D 57.29577951308232
#define D2R 0.017453292519943295
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int16_t s16;
typedef int8_t  s8;

/* make sure the memory is successive */
#pragma pack(2)
#define IMU_PACK_SIZE 46
typedef struct {
    u8     head1;       /* 1 */
    u8     head2;       /* 2 */
    u16    week;        /* 3~4 */
    double sow;         /* 5~12 */
    float  gyrox;       /* 13~16 */
    float  gyroy;       /* 17~20 */
    float  gyroz;       /* 21~24 */
    float  accelx;      /* 25~28 */
    float  accely;      /* 29~32 */
    float  accelz;      /* 33~36 */
    float  temperature; /* 37~40 */
    s16    odo;         /* 41~42 */
    s16    odo2;        /* 43~44 */
    u8     check1;      /* 45 */
    u8     check2;      /* 46 */
} IMU;
#define AID_PACK_SIZE 54
typedef struct {
    u8     head1;  /* 1 */
    u8     head2;  /* 2 */
    u16    week;   /* 3~4 */
    double sow;    /* 5~12 */
    double magx;   /* 13~20 */
    double magy;   /* 21~28 */
    double magz;   /* 29~36 */
    double press;  /* 37~44 */
    double temp;   /* 45~52 */
    u8     check1; /* 53 */
    u8     check2; /* 54 */
} AidSensor;

typedef struct Result {
    double t;
    double Rn[3];
    double Vn[3];
    double euler[3];
} Result;
typedef struct ImuRaw {
    double sow;       /* s */
    double dtheta[3]; /* 陀螺, rad */
    double dvel[3];   /* 加表, m/s */
} ImuRaw;
typedef struct ODO {
    double t;       /* s */
    double v;
} ODO;
#define PI 3.141592653589793
#define N 10
int main() {
    ImuRaw imu;
    IMU gimu;
    AidSensor gaid;
    ODO odo;
    ifstream file,file1;
    ofstream pfile,pfile1,pfile2,pfilebin;
    FILE *cfile;
    FILE *pcfile;
    char buff[200];

    //imu文件时间修正
    // file.open("/media/zzx/H10/HL_INSPROBE_0_IMU.bin",ifstream::binary);
    // pfilebin.open("/media/zzx/H10/HL_INSPROBE_0_IMU_patch.bin",ifstream::binary);
    // file.read((char*)&imu,sizeof(ImuRaw));
    // double lastsow = imu.sow,dt,compensation=0;
    // while (!file.eof()) {
    //     file.read((char*)&imu,sizeof(ImuRaw));
    //     if(file.eof())
    //         break;
    //     dt = imu.sow - lastsow;
    //     lastsow = imu.sow;
    //     if(dt<-1000)
    //         compensation = compensation-dt+0.005;
    //     imu.sow += compensation;
    //     pfilebin.write((char*)&imu,sizeof(ImuRaw));
    // }



   //ODO文件解码
//    file.open("/media/zzx/STORAGE/FileRecv/Navigation/组合导航数据处理/20190826测试/Rover/ODO_GINS.odo",ifstream::binary);
//    pfile.open("/media/zzx/STORAGE/FileRecv/Navigation/组合导航数据处理/20190826测试/Rover/odo.txt");
//    while (!file.eof()) {
//        file.read((char*)&odo,sizeof(ODO));
//        sprintf(buff,"%.8f %.8f %.8f \n",odo.t,odo.v,0.000);
//        pfile<<buff;
//    }

    //转角数据
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/steerdata_1602207338631.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/steerdata.txt");
//    int64_t time;
//    double sow;
//    int lm,rm;
//    while (!file.eof()) {
//        file>>time>>lm>>rm;
//        time=time-3*24*3600*1000;
//        time=time%(7*24*3600*1000);
//        sow=time;
//        sow=18+sow/1000;
//        sprintf(buff,"%.3f %d %d\n",sow,lm,rm);
//        pfile<<buff;
//    }

    //sixsteerdata解码
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/第1组/steerdata_1608015083957.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/第1组/sixsteer_odo.txt");
//    pfile1.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/第1组/sixsteerdata.txt");
//    pfile2.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/第1组/sixododata.txt");
//    int64_t time;
//    double sow;
//    int data[12];
//    double DATA[12]={0};
//    double gap=1.0/N;
//    int num=0;
//    file>>time>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5]>>data[6]>>data[7]>>data[8]>>data[9]>>data[10]>>data[11];
//    time=time-3*24*3600*1000;
//    time=time%(7*24*3600*1000);
//    sow=time;
//    sow=18+sow/1000;
//    static double starttime = (int64_t)sow+1.0;
//
//    while(sow<(starttime-gap/2)){
//        file>>time>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5]>>data[6]>>data[7]>>data[8]>>data[9]>>data[10]>>data[11];
//        time=time-3*24*3600*1000;
//        time=time%(7*24*3600*1000);
//        sow=time;
//        sow=18+sow/1000;
//    }
//    while (!file.eof()) {
//        double lastsow=sow;
//        int temp[6];
//        memcpy(temp,&data[6],sizeof(temp));
//        while(sow<(starttime+gap/2)){
//            if(!file.eof())
//                file>>time>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5]>>data[6]>>data[7]>>data[8]>>data[9]>>data[10]>>data[11];
//            else
//                return 0;
//            sprintf(buff,"%.3f %d %d %d %d %d %d %d %d %d %d %d %d\n",sow,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]);
//            pfile<<buff;
//            time=time-3*24*3600*1000;
//            time=time%(7*24*3600*1000);
//            sow=time;
//            sow=18+sow/1000;
//            for(int i=0;i<6;i++){
//                DATA[i]+=data[i];
//            }
//            num++;
//        }
//        for(int i=0;i<12;i++){
//            if(i<6)
//                DATA[i]=DATA[i]/num;
//            else{
//                DATA[i]=data[i]-temp[i-6];
//                if(DATA[i]<0)
//                    DATA[i]+=65536;
//                DATA[i]=DATA[i]*PI*0.15/800;
//                DATA[i]=DATA[i]/(sow-lastsow);
//            }
//        }
//        if(sow>0){
//            sprintf(buff,"%.3f %8f %8f %8f %8f %8f %8f\n",starttime,DATA[0],DATA[1],DATA[2],DATA[3],DATA[4],DATA[5]);
//            pfile1<<buff;
//            sprintf(buff,"%.3f %8f %8f %8f %8f %8f %8f\n",starttime,DATA[6],DATA[7],DATA[8],DATA[9],DATA[10],DATA[11]);
//            pfile2<<buff;
//        }
//        for(double & i : DATA){
//            i=0;
//        }
//        starttime+=gap;
//        num=0;
//    }

    //ODO文件降频
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/odo_10Hz2.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/odo_10Hz3.txt");
//    int i=0;
//    double t[N],v[N],nothing,midt=0,midv=0;
//    while(!file.eof()){
//        if(i<N){
//            file>>t[i]>>v[i]>>nothing;
//            midv+=(v[i]+nothing)/2;
//            midt+=t[i];
//            i++;
//        }
//        else{
//            i=0;
//            sprintf(buff,"%.8f %.8f %.8f\n",midt/N,midv/N,0);
//            midv=0;
//            midt=0;
//            pfile<<buff;
//        }
//    }

    //ODO+转角
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/HL_INSPROBE_0_ODO.txt");
//    file1.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/steerdata.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/odo_steer2.txt");
//    pfile1.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/odo_10Hz2.txt");
//    pfile2.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/steer_10Hz2.txt");
//    double gap=1.0/N,starttime=438080.05,odot=0,odovl,odovr,steert=0,steerl,steerr;
//    double vl=0,vr=0,sl=0,sr=0;
//    int num1=0,num2=0;
//    while(odot<(starttime-gap/2))
//        file>>odot>>odovl>>odovr;
//    while(steert<(starttime-gap/2))
//        file1>>steert>>steerl>>steerr;
//    while(!file1.eof()&&!file.eof()){
//        while(odot<(starttime+gap/2)){
//            vl+=odovl;
//            vr+=odovr;
//            num1++;
//            if(!file.eof())
//                file>>odot>>odovl>>odovr;
//            else
//                break;
//        }
//        vl=vl/num1;
//        vr=vr/num1;
//        while(steert<(starttime+gap/2)){
//            sl+=steerl*D2R;
//            sr+=steerr*D2R;
//            num2++;
//            if(!file1.eof())
//                file1>>steert>>steerl>>steerr;
//            else
//                break;
//        }
//        sl=sl/num2;
//        sr=sr/num2;
//        if(!file1.eof()&&!file1.eof()){
//            sprintf(buff,"%.3f %.3f %.3f\n",starttime,vl*cos(sl),vl*sin(sl));
//            pfile<<buff;
//            sprintf(buff,"%.3f %.8f %.8f\n",starttime,vl,vr);
//            pfile1<<buff;
//            sprintf(buff,"%.3f %.8f %.8f\n",starttime,sl,sr);
//            pfile2<<buff;
//        }
//        starttime+=gap;
//        vl=0;
//        vr=0;
//        num1=0;
//        sl=0;
//        sr=0;
//        num2=0;
//    }


    //imu.bin文件解码
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/POS620/group2/IMULog202012150733_7_imu.bin",ifstream::binary);
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/POS620/group2/IMULog202012150733_7_imu.txt");
//    pfilebin.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201215/POS620/group2/IMULog202012150733_7_imuyxz.bin",ifstream::binary);
//    while (!file.eof()) {
//        file.read((char*)&imu,sizeof(ImuRaw));
//        double temp1,temp2;
//        temp1 = imu.dtheta[1];
//        temp2 = imu.dvel[1];
//        imu.dtheta[1]=-imu.dtheta[0];
//        imu.dvel[1]=-imu.dvel[0];
//        imu.dtheta[0]=temp1;
//        imu.dvel[0]=temp2;
//
//        sprintf(buff,"%.8f %.8f %.8f %.8f %.8f %.8f %.8f \n",imu.sow,imu.dtheta[0],imu.dtheta[1],imu.dtheta[2],
//                imu.dvel[0],imu.dvel[1],imu.dvel[2]);
//        pfile<<buff;
//        pfilebin.write((char*)&imu,sizeof(ImuRaw));
//    }

    //.imu文件解码
//    file.open("/media/zzx/H6/HL_INSPROBE_1.imu",ifstream::binary);
//    pfile.open("/media/zzx/H6/imu_1.txt");
//    pfilebin.open("/media/zzx/H6/imu_1.bin",ofstream::binary);
//    printf("size=%ld\n",sizeof(IMU));
//    while (!file.eof()) {
//        file.read((char*)&gimu,sizeof(IMU));
//        imu.sow=gimu.sow;
//        imu.dtheta[0]=gimu.gyrox * D2R;
//        imu.dtheta[1]=gimu.gyroy * D2R;
//        imu.dtheta[2]=gimu.gyroz * D2R;
//        imu.dvel[0]=gimu.accelx;
//        imu.dvel[1]=gimu.accely;
//        imu.dvel[2]=gimu.accelz;
//        sprintf(buff,"%.8f %.8f %.8f %.8f %.8f %.8f %.8f \n",imu.sow,imu.dtheta[0],imu.dtheta[1],imu.dtheta[2],
//                imu.dvel[0],imu.dvel[1],imu.dvel[2]);
//        pfile<<buff;
//        pfilebin.write((char*)&imu,sizeof(ImuRaw));
//    }

    //.aid文件解码
//    file.open("/media/zzx/H6/HL_INSPROBE_0.aid",ifstream::binary);
//    pfile.open("/media/zzx/H6/aid_0.txt");
//    printf("size=%ld\n",sizeof(AidSensor));
//    while (!file.eof()) {
//        file.read((char*)&gaid,sizeof(AidSensor));
//        sprintf(buff,"%.8f %.8f %.8f %.8f %.8f %.8f \n",gaid.sow,gaid.magx,gaid.magy,gaid.magz,gaid.press,gaid.temp);
//        pfile<<buff;
//    }

    //GNSS中断
//    double t, pos[3], var[3];
//    int i=0;
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/HL_INSPROBE_0_GNSS.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/gnssoutage2.txt");
//    while (!file.eof()) {
//        file >> t >> pos[0] >> pos[1] >> pos[2] >> var[0] >> var[1] >> var[2];
//        if (t >= 438080) {
//            i=(i+1)%120;
//            if (i <= 90) {
//                sprintf(buff,"%.3f %.10f %.10f %.3f %.3f %.3f %.3f \n", t, pos[0], pos[1], pos[2], var[0],
//                                      var[1], var[2]);
//                pfile << buff;
//            }
//        } else{
//            sprintf(buff,"%.3f %.10f %.10f %.3f %.3f %.3f %.3f \n", t, pos[0], pos[1], pos[2], var[0], var[1],
//                                  var[2]);
//            pfile << buff;
//        }
//    }

    //文件截断
//    double t, pos[3], var[3];
//    file.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/HL_INSPROBE_0_GNSS.txt");
//    pfile.open("/mnt/Storage/FileRecv/Navigation/组合导航数据处理/机器人数据/20201009-信操数据/gnss3.txt");
//    while (!file.eof()) {
//        file >> t >> pos[0] >> pos[1] >> pos[2] >> var[0] >> var[1] >> var[2];
//        if (t <= 440180) {
//            sprintf(buff,"%.3f %.10f %.10f %.3f %.3f %.3f %.3f \n", t, pos[0], pos[1], pos[2], var[0],
//                                  var[1], var[2]);
//            pfile << buff;
//        }
//    }

    std::cout << "Hello, World!" << std::endl;
    return 0;
}