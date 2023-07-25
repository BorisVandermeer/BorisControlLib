/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of A SimpleTracker whit PID+PP Controller
 * 
*********************************************************************/

# pragma once

#include<controllers/PIDController.h>
#include<controllers/PurePursuit.h>


namespace Controller{
    class SimpleTracker{
    public:
        struct ConfigType{
            
            Models:: VehicleShape CarShape;
            double lookahead;
            double l_fw,l_bcw;
            
            PIDController::Parameter kpid;

            double ts;

            double max_v;
            double max_a;
            double max_v_err_int;
            double max_steering;
        };

        struct InitType{
            Points::Pos2D FirstPos;
            double min_s;
            double max_s;
        };

        struct KernalInputType{
            double speed;
            Points::Pos2D pos;            
        };

        struct KernalReturnType{
            double acc;
            double steering;
        };

        SimpleTracker() = default;
        SimpleTracker(ConfigType const & config);

        bool Configure(ConfigType const & config);
        bool SetTrajectory(PNC_Common::TrajectorySegment _traj);
        bool Init(InitType config); // Init And Refresh
        KernalReturnType KernalFunction(KernalInputType const & state);

    private:
        PIDController SpeedController;
        PurePursuit   SteerController;
        PNC_Common::TrajectorySegment Traj;

        double last_s;
        double ts = -1;


    };
}// namespace Controller

