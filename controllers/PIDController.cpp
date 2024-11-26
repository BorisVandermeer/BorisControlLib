/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of PID Controller
 * 
*********************************************************************/

#include<controllers/PIDController.h>

namespace Controller
{
    double PIDController::KernelFunction(double error,double ts,WarningType& _warnings){
        double ans = 0;
        // kp
        ans -= k_.kp * error;
        // ki
        integration += (error+lasterr_)/2*ts;
        ans -= k_.ki * integration;
        // kd
        if(status != Waiting){
            ans -= k_.kd * (error-lasterr_)/ts;
        }
        status = Running;
        lasterr_ = error;

        // Safty
        if(integration > limits.max_int){
            warnings =  warnings|PID_WARNING_MAX_INT;
            integration = limits.max_int;
        }

        if(integration < -limits.max_int){
            warnings =  warnings|PID_WARNING_MAX_INT;
            integration = -limits.max_int;
        }

        if(ans > limits.max_out){
            warnings =  warnings|PID_WARNING_MAX_OUT;
            ans = limits.max_out;
        }

        return ans;
    }

    double PIDController::KernelFunction(double error_input,WarningType& warnings) {
        if(ts_<0){
            throw "PID Kernel : Invalid default ts.";
        }
        return KernelFunction(error_input,ts_,warnings);
    }

}// namespace CarControll