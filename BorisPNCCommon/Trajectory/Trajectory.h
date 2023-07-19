/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of Trajectory
 * 
*********************************************************************/

#include<vector>
#include<memory>

#include<Points/PointsType.h>
#include<Interplot/SplineCurve.h>

namespace PNC_Common{
    class PathSegment{
    public:
        PathSegment() = default;
        typedef std::shared_ptr<Interplot::SplineCurve>  CurvePtr;
        enum PathSegmentType{
            Forward, BackWard,
        };

        typedef Points::PosPoint2D Point2D;
        typedef Points::Pos2D Pos2D;

        void SetCurve(CurvePtr ptr){Data = ptr;}
        Point2D operator() (double s) const {return Data->operator()(s);};
        double getDirection(double s) const {return Data->getHeading(s);}
        Pos2D  getPos(double s) const {auto tmp = Data->operator()(s); return Pos2D(tmp.x,tmp.y,Data->getHeading(s));}

        PathSegmentType Type;
        
    private:
        CurvePtr Data;
        double length;
        double FromHeading;
        double ToHeading;
    };
} // namespace PNC_Common