/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of PurePursuit Controller
 * 
*********************************************************************/
#pragma once

#include<vector>
#include<memory>

#include<Trajectory/Trajectory.h>
#include<Models/VehicleModel.h>

#include<controllers/PIDController.h>

namespace Controller{
    class PurePursuit{
    public:
        PurePursuit() = default;
        typedef PNC_Common::PathSegment PathSegment;
        typedef PNC_Common::PathSegment::Pos2D Pos2D;
        typedef PNC_Common::PathSegment::Point2D Point2D;
        typedef Models::VehicleShape   VehicleShape;

        void SetPath(PathSegment const & _path){path = _path;}
        // l_fw  - anchor distance for forward driving (configurable)
        // l_bck - anchor distance for backwards driving (configurable)
        void SetVehicle(VehicleShape veh,double maxsteer,double _l_fw,double _l_bck);
        void SetStepSize(double max,double min){maxsteps = max;minsteps = min;}
        void SetLookahead(double lookahead){default_lahead = lookahead;}

        double SetFirstPos(Pos2D pos,double mins,double maxs);

        // pos for backwheel pos
        double KernelFunction(Pos2D pos);
        double KernelFunction(Pos2D pos, double lookahead);
        double KernelFunction(Pos2D pos, double minmove,double maxmove);
        double KernelFunction(Pos2D pos, double lookahead, double minmove,double maxmove);

    private:
        PathSegment path;
        VehicleShape Vehicle;
        double max_steer;
        double last_s;
        double maxsteps = 1.0;
        double minsteps = -1.0;
        double default_lahead = -1.0;
    
    private:
        double l_fw,l_bck;
        
    };
} // namespace CarControll
