/**
 * @file DJIMotor.hpp
 * @author - GUO, Zilin
 *         - Your Name
 * @brief This is the DJIMotor template codes for RM2024-Tutorial PA3 and RDC
 * @note  You could directly transplant your code to the RDC after finishing PA3
 * @note  If you do not like the template I provide for you, you could remove
 * all of them and use your own
 * @copyright This file is only for HKUST Enterprize RM2024 internal
 * competition. All Rights Reserved.
 *
 */

#pragma once
#include "AppConfig.h"

#if USE_DJI_MOTOR

#ifndef DJI_MOTOR_CAN
#define DJI_MOTOR_CAN hcan
#endif

#include "main.h"
#include "stdint.h"
#include "PID.hpp"
#include "DR16.hpp"
#include "PID.hpp"

#define UP 1
#define DOWN -1
#define REST 0
#define AXISSPEED1 3000
#define AXISSPEED2 1500
#define SPEEDLIMIT 3960

namespace DJIMotor
{

    #define TX_ID 0x200
    #define EX_TX_ID 0x1FF
    typedef unsigned int ticks;
/**
 * @brief A motor's handle. We do not require you to master the cpp class
 * syntax.
 * @brief However, some neccessary OOP thought should be shown in your code.
 * @brief For example, if you have multiple motors, which is going to happen in
 * RDC (You have at least 4 wheels to control)
 * @brief You are able to write a "template" module for all the abstract motors,
 * and instantiate them with different parameters
 * @brief Instead of copy and paste your codes for four times
 * @brief This is what we really appreiciate in our programming
 */ 

    enum class TYPE{
        WHEEL,
        ARM
    };

    class DJIMotor
    {
        private:
            uint16_t canID;  // You need to assign motor's can ID for different motor
            
            // Below are the types of measurements 

            int convertedUART;
            int realAngle;

            // Below are types of 

            int16_t mechanicalAngle;
            int16_t rotationalSpeed;
            int16_t current;

            Control::PID motorPID{1,0,0}; //uncomment the code once the PID has a proper constructor and then update DJIMotor accordingly
            // ticks lastUpdated;
            TYPE motorType;

        public:
            DJIMotor(const int& ID);

            void updateInfoFromCAN(const uint8_t* rxBuffer);
            void updateTargetRPM(const int);
            void setPID(const float*);

            // void getValues(int* container);
            int getPIDRPM();
            int getPIDSpeed();
            void setRealAngle(const int&);
            void RealAngleInit(){realAngle=0;}
            // int getrotationalSpeed(){return rotationalSpeed;}
            // int getconvertedUART(){return convertedUART;}
            // int getrealAngle(){return realAngle;}
    };

    class MotorPair{
        private:
            DJIMotor motor[4];
            int size;
        public:
            MotorPair(const int IDStart, const int s);
            MotorPair(const int IDStart,const int s, const float pid[][3]);

            void transmit(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,CAN_FilterTypeDef*);

            void errorHandler(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,CAN_FilterTypeDef*);

            DJIMotor& operator[](const int);
            void init(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* filter);
            
            void updateTargetRPM(const int*);
            int getsize(){return size;}
    };

    class motorMechanics{
        public:
    
        int motor1;
        int motor2;
        int motor3;
        int motor4;

        // motorMechanics(const int*);
        motorMechanics(const int, const int, const int, const int);
        void operator=(const motorMechanics&);
        // void operator=(const int*);

        motorMechanics operator+(const motorMechanics& matrix);
        motorMechanics operator*(const float multiple);

        void matrixRotateLeft(); 
        void matrixRotateRight();

        void normalise();
        void cpyMotorVals(int*);
        void reduceCornerRotate();

    };

    int max(const int a, const int b);

    int abs(const int& a);
    int absmax(int,int,int,int);

extern void UART_ConvertMotor(const DR16::RcData&,MotorPair&);
extern void UART_ConvertArm(const DR16::RcData&,MotorPair&,const float&);
double sqrt(double);

/*===========================================================*/
}  // namespace DJIMotor
#endif  // USE_DJI_MOTOR