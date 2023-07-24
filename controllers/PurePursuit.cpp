/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of PurePursuit Controller
 * 
*********************************************************************/

#include<Geometry/Line2D.h>
#include<Geometry/Circle2D.h>

#include<controllers/PurePursuit.h>


using namespace std;
using Vectors::Vector2D;
using Geometry::Line;


#define PRIVATE_EPSILON (1e-5)

namespace Controller{

    void PurePursuit::SetVehicle(PurePursuit::VehicleShape veh,double maxsteer,double _l_fw,double _l_bck){
        Vehicle = veh;
        max_steer = maxsteer;
        l_fw = _l_fw;
        l_bck = _l_bck;
    }

    double PurePursuit::SetFirstPos(Pos2D pos,double mins,double maxs){
        Point2D tmpp(pos.x,pos.y);
        last_s = path.getProjection(tmpp,mins,maxs);
        return last_s;
    }

    double PurePursuit::KernelFunction(Pos2D pos, double lookahead,double minmove,double maxmove){
        Point2D tmpp(pos.x,pos.y);
        double cur_s = path.getProjection(tmpp,last_s+minmove,last_s+maxmove);
        double target_s = cur_s+lookahead;
        double Radius;
        if(l_fw<PRIVATE_EPSILON){
            Vector2D p1 = path(target_s);
            double & phi = pos.phi;
            Line l1(Vector2D(tmpp),Vector2D(-sin(phi),cos(phi)));
            Vector2D mid = (Vector2D(tmpp)+p1)/2;
            Vector2D dir = p1-Vector2D(tmpp);
            dir = Vector2D(-dir.y,dir.x);
            Line l2(mid,dir);
            Vector2D center = Geometry::GetIntersection(l1,l2);
            Radius = Vector2D(sin(phi),-cos(phi)) * (center-Vector2D(tmpp));
        } else if(path.Type == PathSegment::Forward) {
            Vector2D p1 = path(target_s);
            double & phi = pos.phi;
            Vector2D p2 = Vector2D(tmpp) + Vector2D(l_fw*cos(phi),l_fw*sin(phi));
            Vector2D p3 = Vector2D(tmpp)  + Vector2D(tmpp) - p2;
            if(Vectors::isParallel(p2-p3,p3-p1)) return .0;
            Vector2D center = Geometry::solveCenterPointOfCircle(p1,p2,p3);
            Radius = Vectors::det(Vectors::normalize(p2-p3) , (center-Vector2D(tmpp)));
        } else if((path.Type == PathSegment::BackWard)){
            Vector2D p1 = path(target_s);
            double & phi = pos.phi;
            Vector2D p2 = Vector2D(tmpp) + Vector2D(l_bck*cos(phi),l_bck*sin(phi));
            Vector2D p3 = Vector2D(tmpp)  + Vector2D(tmpp) - p2;
            if(Vectors::isParallel(p2-p3,p3-p1)) return .0;
            Vector2D center = Geometry::solveCenterPointOfCircle(p1,p2,p3);
            Radius = Vectors::det(Vectors::normalize(p2-p3) , (center-Vector2D(tmpp)));
        } else {
            throw "KernelFunction in PurePuesuit : Invaild PathSegmentType.";
        }

        last_s = cur_s;
        double ans = atan2(Vehicle.wheelbase,Radius);
        if(ans>M_PI_2) ans -= M_PI;
        if(ans>max_steer) ans = max_steer;
        if(ans<-max_steer) ans = -max_steer;
        return ans;
    }

    double PurePursuit::KernelFunction(Pos2D pos, double lookahead){
        return KernelFunction(pos, lookahead, minsteps,maxsteps);
    }
    
} // namespace Controller
