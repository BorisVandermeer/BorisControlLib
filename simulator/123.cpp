
#include<iostream>
#include<Geometry/Line2D.h>
#include<Geometry/Circle2D.h>


int main(){
    Geometry::Line l1;
    Geometry::Line l2;
    l1.point = Vectors::Vector2D(3,1);
    l1.direction = Vectors::Vector2D(1,-1);
    l2.point = Vectors::Vector2D(0,0);
    l2.direction = Vectors::Vector2D(1,1);
    std::cout<<Vectors::Vector2D(Geometry::GetIntersection(l1,l2))<<std::endl;
    

    Vectors::Vector2D p1(0,0);
    Vectors::Vector2D p2(10,10);
    Vectors::Vector2D p3(1,-1);

    std::cout<<Geometry::solveCenterPointOfCircle(p1,p2,p3)<<std::endl;

    return 0;
}