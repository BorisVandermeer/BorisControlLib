/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of A SimpleTracker whit PID+PP Controller
 * 
*********************************************************************/

#include<controllers/SimpleTracker.h>


namespace Controller{
    SimpleTracker::SimpleTracker(ConfigType const & config){
        bool status = this->Configure(config);
        if(!status){
            throw "SimpleTracker Construct : Failed To Configure.";
        }
    }

    bool SimpleTracker::Configure(ConfigType const & config){
        SteerController.SetVehicle(config.CarShape,config.max_steering,config.l_fw,config.l_bcw);
        SteerController.SetLookahead(config.lookahead);

        SpeedController.SetDefaultTs(config.ts);
        SpeedController.SetParameters(config.kpid);
        Controller::PIDController::SaftyLimits PIDlimit{.max_int = config.max_v_err_int,.max_out = config.max_a};
        SpeedController.SetSaftyLimits(PIDlimit);
        return true;
    }

    bool SimpleTracker::SetTrajectory(PNC_Common::TrajectorySegment _traj){
        Traj = _traj;
        return true;
    }

    bool SimpleTracker::Init(InitType config){
        SpeedController.init();
        SteerController.SetFirstPos(config.FirstPos, config.min_s,config.max_s);
        return true;
    }
}
