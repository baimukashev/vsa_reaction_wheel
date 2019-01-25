#include <iostream>
#include <NIDAQmxBase.h>
#include <curses.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <string.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include "pid.h"
#include "EKF.h"
#define DAQmxErrChk(functionCall)                   \
        { if(DAQmxFailed(error=(functionCall)) )    \
            {                                       \
                goto Error;                         \
            }                                       \
        }

inline int32_t DAQmxBaseWriteAnalogScalarF64(TaskHandle taskHandle, bool32 autoStart, float64 timeout, float64 value, bool32 *reserved)
{
    return DAQmxBaseWriteAnalogF64(taskHandle, 1, static_cast<bool32>(false), timeout, DAQmx_Val_GroupByChannel, &value, nullptr, reserved);
}

inline void Sleep(int msec)
{
    std::this_thread::sleep_for(std::chrono::microseconds(msec));
}

using namespace std;

TaskHandle taskHandleM1 = nullptr;
TaskHandle taskHandleM2 = nullptr;

TaskHandle taskHandleEncoder1 = nullptr;  // y link //m1 encoder blue
TaskHandle taskHandleEncoder2 = nullptr;  // x static //m2 encoder red
TaskHandle taskHandleEncoder3 = nullptr;  //m1   // static base encoder
TaskHandle taskHandleEncoder4 = nullptr;  //m2   // link encoder
TaskHandle taskHandleGyroPower1 = nullptr;
TaskHandle taskHandleGyroPower2 = nullptr;

TaskHandle taskHandleGyro1 = nullptr;       //gyro1;
TaskHandle taskHandleGyro2 = nullptr;
void encoderPosition(double *encoderPos);
double readGyro(TaskHandle gyroTask);
void mixer(double x,double y,double &m1,double &m2);




int main() {

    ofstream ferr("error.txt");
    ofstream outputfile("output.txt");

    int32 error=0; // to check the error
    char errBuff[2048] = {0}, errBuff2[2048] ={0};   // buffer to store the error
    double initValue[4]={0,0,0,0};    // this values assigned to encoder as initial value
    auto zIndex = static_cast<bool32>(false); // needed to create sensors
    double Ts = 0.001, Tfinal = 10;

    EKFv encoder1(Ts,100,1e-6,0.0,0.0);
    EKFv encoder2(Ts,100,1e-6,0.0,0.0);
    EKFv encoder3(Ts,100,1e-6,0.0,0.0);
    EKFv encoder4(Ts,100,1e-6,0.0,0.0);

    PID anglex(Ts,5,-5,0.0,80,0,0);
    PID angley(Ts,0.7,-0.7,0.0,40,0,0);

    PID motor1(Ts,10,-10,0.0,0.3,0,0);
    PID motor2(Ts,10,-10,0.0,0.3,0,0);

    std::vector<std::string> devices{"Dev1", "Dev2", "Dev3"};
    for (const auto &dev : devices)
    {
        if (DAQmxBaseResetDevice(dev.c_str())){
            cout << "cannot initialize " << dev << endl;
        }
    }

    // encoders
    DAQmxErrChk(DAQmxBaseCreateTask("",&taskHandleEncoder1));
    DAQmxErrChk(DAQmxBaseCreateCIAngEncoderChan( taskHandleEncoder1,
                                                 "Dev1/ctr0",
                                                 "",
                                                 DAQmx_Val_X4,
                                                 zIndex,
                                                 0,
                                                 DAQmx_Val_ALowBLow,
                                                 DAQmx_Val_Radians,
                                                 1024,
                                                 static_cast<float64>(initValue[0]),
                                                 nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleEncoder1)); // start task 1


    DAQmxErrChk(DAQmxBaseCreateTask("",&taskHandleEncoder2));
    DAQmxErrChk(DAQmxBaseCreateCIAngEncoderChan( taskHandleEncoder2,
                                                 "Dev1/ctr1",
                                                 "",
                                                 DAQmx_Val_X4,
                                                 zIndex,
                                                 0,
                                                 DAQmx_Val_ALowBLow,
                                                 DAQmx_Val_Radians,
                                                 1024,
                                                 static_cast<float64>(initValue[1]),
                                                 nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleEncoder2)); // start task 2

    DAQmxErrChk(DAQmxBaseCreateTask("",&taskHandleEncoder3));
    DAQmxErrChk(DAQmxBaseCreateCIAngEncoderChan( taskHandleEncoder3,
                                                 "Dev2/ctr1",
                                                 "",
                                                 DAQmx_Val_X4,
                                                 zIndex,
                                                 0,
                                                 DAQmx_Val_ALowBLow,
                                                 DAQmx_Val_Radians,
                                                 1024,
                                                 static_cast<float64>(initValue[2]),
                                                 nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleEncoder3)); // start task 1

    DAQmxErrChk(DAQmxBaseCreateTask("",&taskHandleEncoder4));
    DAQmxErrChk(DAQmxBaseCreateCIAngEncoderChan( taskHandleEncoder4,
                                                 "Dev2/ctr0",
                                                 "",
                                                 DAQmx_Val_X4,
                                                 zIndex,
                                                 0,
                                                 DAQmx_Val_ALowBLow,
                                                 DAQmx_Val_Radians,
                                                 1024,
                                                 static_cast<float64>(initValue[3]),
                                                 nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleEncoder4)); // start task 4

    // motor 1
    DAQmxErrChk(DAQmxBaseCreateTask("motor1",&taskHandleM1));

    DAQmxErrChk(DAQmxBaseCreateAOVoltageChan(taskHandleM1,"Dev1/ao0","",-10,10,DAQmx_Val_Volts,nullptr));
    DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM1, 0, 0.0, 0, nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleM1));

    // motor 2
    DAQmxErrChk(DAQmxBaseCreateTask("motor2",&taskHandleM2));

    DAQmxErrChk(DAQmxBaseCreateAOVoltageChan(taskHandleM2,"Dev2/ao0","",-10,10,DAQmx_Val_Volts,nullptr));
    DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM2, 0, 0.0, 0, nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleM2));

    // gyros power 1
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleGyroPower1));
    DAQmxErrChk(
            DAQmxBaseCreateAOVoltageChan(taskHandleGyroPower1, "Dev1/ao1", "", -5.0, 5.0, DAQmx_Val_Volts, nullptr));
    double val = 0.0;
    DAQmxErrChk(
            DAQmxBaseWriteAnalogF64(taskHandleGyroPower1, 1, 0, 0.01, DAQmx_Val_GroupByChannel, &val, nullptr, nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleGyroPower1));
    val = 3.0;
    DAQmxErrChk(
            DAQmxBaseWriteAnalogF64(taskHandleGyroPower1, 1, 0, 0.01, DAQmx_Val_GroupByChannel, &val, nullptr, nullptr));


    // gyros power 2
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleGyroPower2));
    DAQmxErrChk(
            DAQmxBaseCreateAOVoltageChan(taskHandleGyroPower2, "Dev2/ao1", "", -5.0, 5.0, DAQmx_Val_Volts, nullptr));
    val = 0.0;
    DAQmxErrChk(
            DAQmxBaseWriteAnalogF64(taskHandleGyroPower2, 1, 0, 0.01, DAQmx_Val_GroupByChannel, &val, nullptr, nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleGyroPower2));
    val = 3.0;
    DAQmxErrChk(
            DAQmxBaseWriteAnalogF64(taskHandleGyroPower2, 1, 0, 0.01, DAQmx_Val_GroupByChannel, &val, nullptr, nullptr));

    //gyros 1
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleGyro1));
    DAQmxErrChk(
            DAQmxBaseCreateAIVoltageChan(taskHandleGyro1, "Dev1/ai2", "", DAQmx_Val_RSE, 0.4, 2.6, DAQmx_Val_Volts,
                                         nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleGyro1));

    // gyros 2
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleGyro2));
    DAQmxErrChk(
            DAQmxBaseCreateAIVoltageChan(taskHandleGyro2, "Dev2/ai2", "", DAQmx_Val_RSE, 0.4, 2.6, DAQmx_Val_Volts,
                                         nullptr));
    DAQmxErrChk(DAQmxBaseStartTask(taskHandleGyro2));


    cout << "Loop start" << endl;

    // loop >> run motor

    double encoderPos[4];
    double x[2] = {0.0,0.0};
    double m1=1,m2=1,a=0,b=0;

    Sleep(2e6);

    for(int j=0;j<(int)(Tfinal/Ts);j++){
        auto t1 = std::chrono::high_resolution_clock::now(); // timingvalueM[1]
        encoderPosition(encoderPos); // read sensor data;
        encoder1.calculate(encoderPos[0]);
        encoder2.calculate(encoderPos[1]);
        encoder3.calculate(encoderPos[2]);
        encoder4.calculate(encoderPos[3]);
            //ddt = j/1000.0;
            //valueM = 8*(sin(2*3.14/1.1*ddt));
        encoder2.get_x(x);
        a=anglex.calculate(0.0,x[0]);

        mixer(a,b,m1,m2);

        //m1=motor1.calculate(3.0,x[0]);
        //encoder4.get_x(x);
        //m2=motor2.calculate(3.0,x[0]);
        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM1, 0, 0.0, m1, nullptr)); //0.5
        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM2, 0, 0.0, m2, nullptr)); //0.5


        double gyroValue1 = readGyro(taskHandleGyro1);
        double gyroValue2 = readGyro(taskHandleGyro2);
        cout<<Ts*j <<" "<<encoderPos[0]<<" "<<x[0]<<" "<<m1<<"   "<<m2<<endl;//<<" "<<encoderPos[0]<<" "<<encoderPos[1]<<" "<<encoderPos[2]<<" "<<encoderPos[3]<< " "<<m1<<" "<<m2<<endl; //<< valueM
        outputfile<<Ts*j <<" "<<encoderPos[0]<<" "<<encoderPos[1]<<" "<<encoderPos[2]<<" "<<encoderPos[3]<<" "<<gyroValue1<<" "<<gyroValue2<<endl;
        //pos = pos + gyroValue*0.001;
            //cout <<" "<<gyroValue<<" "<< endl;

        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-9; // elapsed time in seconds for computations
        Sleep((Ts-(int)elapsed)*1e6);
    }


    // stop motors
    DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM1, 0, 0.0, 0, nullptr)); //0.5
    DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM2, 0, 0.0, 0, nullptr)); //0.5

    cout << "Loop end" << endl;

    Error :

        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM1, 0, 0.0, 0, nullptr));
        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleM2, 0, 0.0, 0, nullptr));
        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleGyroPower1, 0, 0, 0.0, nullptr));
        DAQmxErrChk(DAQmxBaseWriteAnalogScalarF64(taskHandleGyroPower2, 0, 0, 0.0, nullptr));

        if (DAQmxFailed(error)) {
            cout << "Error!" << endl;
            DAQmxBaseGetExtendedErrorInfo(errBuff, 2048);
            DAQmxBaseGetExtendedErrorInfo(errBuff2, 2048);
        }
        if (taskHandleEncoder1 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleEncoder1);  // stop task 1
            DAQmxBaseClearTask(taskHandleEncoder1); // stop task 1
        }
        if (taskHandleEncoder2 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleEncoder2);  // stop task 2
            DAQmxBaseClearTask(taskHandleEncoder2); // stop task 2
        }

        if (taskHandleEncoder3 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleEncoder3);  // stop task 3
            DAQmxBaseClearTask(taskHandleEncoder3); // stop task 3
        }

        if (taskHandleEncoder4 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleEncoder4);  // stop task 4
            DAQmxBaseClearTask(taskHandleEncoder4); // stop task 4
        }

        if (taskHandleM1 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleM1);  // stop task 1
            DAQmxBaseClearTask(taskHandleM1); // stop task 1
        }

        // motor 2

        if (taskHandleM2 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleM2);  // stop task 1
            DAQmxBaseClearTask(taskHandleM2); // stop task 1
        }

        if (taskHandleGyroPower1 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleGyroPower1);  // stop task 1
            DAQmxBaseClearTask(taskHandleGyroPower1); // stop task 1
        }

        if (taskHandleGyroPower2 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleGyroPower2);  // stop task 1
            DAQmxBaseClearTask(taskHandleGyroPower2); // stop task 1
        }

        if (taskHandleGyro1 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleGyro1);  // stop task 1
            DAQmxBaseClearTask(taskHandleGyro1); // stop task 1
        }

        if (taskHandleGyro2 != nullptr)
        {
            DAQmxBaseStopTask(taskHandleGyro2);  // stop task 1
            DAQmxBaseClearTask(taskHandleGyro2); // stop task 1
        }

    if(DAQmxFailed(error))
            ferr<< "DAQmxBase Error " << error << ": " << errBuff << endl;

        ferr.close();
        outputfile.close();
        return 0;
}

void encoderPosition(double *encoderPos){

    float64 data;

    DAQmxBaseReadCounterScalarF64(taskHandleEncoder1,-1.0,&data, nullptr);
    encoderPos[0] = data;
    DAQmxBaseReadCounterScalarF64(taskHandleEncoder2,-1.0,&data, nullptr);
    encoderPos[1] = data;
    DAQmxBaseReadCounterScalarF64(taskHandleEncoder3,-1.0,&data, nullptr);
    encoderPos[2] = data;
    DAQmxBaseReadCounterScalarF64(taskHandleEncoder4,-1.0,&data, nullptr);
    encoderPos[3] = data;
}
void mixer(double x,double y,double &m1,double &m2){
    m1 = -x +y;
    m2 = -x -y;
}

double readGyro(TaskHandle gyroTask) {
    float64 timeout = 0.001;
    float64 readArr[1];
    int32 sampsPerChanRead;
    constexpr double offset = 2.273;
    constexpr double coef = 0.7;
    DAQmxBaseReadAnalogF64(gyroTask, 1, timeout, DAQmx_Val_GroupByChannel, readArr, 1, &sampsPerChanRead, nullptr);
    return (readArr[0]-offset)/coef;

    /*
    int64 pointsToRead = 1;
    float64 data;
    int64 samplesPerChan = 1;
    int32 pointsRead;

    constexpr double kGyroOffset = 2.273;
    constexpr double kGyroCoef = 0.09;
    constexpr double dt = 0.001;
    DAQmxBaseReadAnalogF64(gyroTask, pointsToRead, dt, DAQmx_Val_GroupByChannel, &data,
                                        samplesPerChan, &pointsRead, nullptr);
    double gyro_state = static_cast<double>(data - kGyroOffset) / kGyroCoef * 2.0;

    return data;
     */
}