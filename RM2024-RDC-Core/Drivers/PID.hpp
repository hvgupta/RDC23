/**
 * @file PID.hpp
 * @author - GUO, Zilin
 *         - JIANG, Yicheng
 *         - Your name
 * @date 2023-10-20
 * @brief This file gives out the template skeleton code for you to implement a
 * PID algorithm
 */
#pragma once
#include "AppConfig.h"

#if USE_PID
#include "FreeRTOS.h"
#include "task.h"


namespace Control
{

    /*
        - motor, we need to control the speed,
        - arm, we would have to control the rotation.
    */

class PID
{
   public:
    /**
     * @brief Delete the default constructor.
     */
    PID() = delete;

    /**
     * @brief Constructor function
     * @param Kp_ The P term of the PID
     * @param Ki_ The I term of the PID
     * @param Kd_ The D term of the PID
     */
    PID(float Kp_ = 0, float Ki_ = 0, float Kd_ = 0)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_){};
    /**
     * @brief Update the PID status and return
     * @note  You need to implement the specific definition in the PID.cpp file
     * @param target        The target value
     * @param measurement   The feedback value
     * @param dt            The time interval between two updates
     */
    float update(float target, float measurement, float dt = 0.001f);

    /*===================*/
    // Your self-defined functions begin here
    // You might wish to add more functions for your PID module
    /*===================*/

    /*===================*/
    // Your self-defined functions end here
    /*===================*/

   private:
    float Kp = 0;  // The P param of the PID
    float Ki = 0;  // The I param of the PID
    float Kd = 0;  // The D param of the PID

    float error     = 0;  // The error in this update
    float lastError = 0;  // The error from last update
    float accumalatedError = 0; // this is the overall error the wheel has from moving, allows for more refined controll of I error

    float pOut = 0;  // The P term output of the PID
    float iOut = 0;  // The I term output of the PID
    float dOut = 0;  // The D term output of the PID

    float output = 0;  // The current output of the PID

    /*===================*/
    // Your self-defined variables begin here
    /*===================*/

    /*===================*/
    // Your self-defined variables end here
    /*===================*/
};
}  // namespace Control

#endif
