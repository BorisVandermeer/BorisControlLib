/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of PID Controller
 * 
*********************************************************************/
#pragma once

#include<vector>
#include<memory>

#include<Interplot/SplineCurve.h>

#define PID_WARNING_NO      (0x00U)
#define PID_WARNING_MAX_INT (0x01U)
#define PID_WARNING_MAX_OUT (0x02U)

namespace Controller
{
    class PIDController{
    public:
        struct Parameter{
            double kp,ki,kd;
        };

        struct SaftyLimits{
            double max_int,max_out;
        };

        enum Status{
            // Stopping Status
            Waiting,
            // Running Status
            Running,
        };
        typedef unsigned char WarningType;
        
        
    public:
        PIDController() = default;
        PIDController(Parameter _k){k_=_k;}
        PIDController(Parameter _k,SaftyLimits _limits){k_=_k,limits=_limits;}
        // Configure
        void SetDefaultTs(double ts){ts_=ts;}
        void SetParameters(Parameter _k){k_=_k;}
        void SetParameters(Parameter _k,SaftyLimits _limits){k_=_k,limits=_limits;}
        void SetSaftyLimits(SaftyLimits _limits){limits=_limits;}
        void init(){status = Waiting, integration =0, lasterr_ = 0;}
    

        double KernelFunction(double error_input,WarningType& warnings) {return KernelFunction(error_input,ts_,warnings);}
        double KernelFunction(double error_input,double ts,WarningType& _warnings);
        Status status = Waiting;

        void ClearIntegration(){integration = 0;}
        void ClearWarnings(){warnings = PID_WARNING_NO;}
        
    private:
        Parameter k_;
        SaftyLimits limits;
        WarningType warnings = PID_WARNING_NO;
        double integration = 0;
        double lasterr_ = 0;
        double ts_;

    };
} // namespace CarControll