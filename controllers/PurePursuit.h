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
#include<Interplot/SplineCurve.h>

namespace Controller
{
    class PurePursuit{
    public:
        void SetPath(std::shared_ptr<std::vector<Interplot::SplineCurve>> _CurvesPtr){CurvesPtr = _CurvesPtr;};
        double getSpeed(){return max_speed;};
    
    protected:
        std::shared_ptr<std::vector<Interplot::SplineCurve>> CurvesPtr;
        

    private:
        const double max_speed;
        const double max_steering_angle;
        
    };
} // namespace CarControll
