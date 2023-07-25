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
        ts = config.ts;
        return true;
    }

    bool SimpleTracker::SetTrajectory(PNC_Common::TrajectorySegment _traj){
        Traj = _traj;
        return true;
    }

    bool SimpleTracker::Init(InitType config){
        SpeedController.init();
        last_s = SteerController.SetFirstPos(config.FirstPos, config.min_s,config.max_s);
        return true;
    }

    SimpleTracker::KernalReturnType SimpleTracker::KernalFunction(KernalInputType const & state){
        
        double cur_vcc = state.speed;
        double cur_s;
        using PNC_Common::PathSegment;
        // double tardir = Traj.getDirection(last_s);
        double v_abs = fabs(cur_vcc);
        double min_step = -v_abs*ts*2-1;
        double max_step = v_abs*ts*2+1;

        cur_s = Traj.getProjection(state.pos,cur_s+min_step,cur_s+max_step);

        double tar_vcc = Traj.getSpeed(cur_s);
        PIDController::WarningType tmp;
        double acc = SpeedController.KernelFunction(cur_vcc-tar_vcc,tmp);
        double steer = SteerController.KernelFunction(state.pos,min_step,max_step);

        KernalReturnType ans{.acc = acc, .steering = steer};
        return ans;
    }
}
