/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple Simulator of PurePursuit Controller
 * 
*********************************************************************/

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#include<Vectors/Vector2D.h>
#include<controllers/PurePursuit.h>

#define COLOR_WHITE Vec3b(255,255,255)
#define COLOR_BLACK Vec3b(0,0,0)
#define COLOR_GREEN Vec3b(0,255,0)
#define COLOR_RED   Vec3b(0,0,255)
#define COLOR_BLUE  Vec3b(255,0,0)
#define COLOR_DARK_RED Vec3b(0,0,155)

#define IMG_HIGHT 1000
#define IMG_WIDTH 1500

#define RESOLUION (0.2)
#define LOOKAHEAD (15)



using namespace std;
using namespace cv;

using PNC_Common::PathSegment;
using Vectors::Vector2D;

Controller::PurePursuit PP;

const int times = 1000;
const double ts = 0.05;
const double speed = 5;
const double totalt = ts*times;
const double max_steer = (15*M_PI/180);

Points::Pos2D CarPos;


Points::PosPoint2Ds pts;
vector<double> angles;
vector<int> xlist;
vector<int> ylist;
PathSegment Path;
Points::Pos2Ds ps;
Models::VehicleShape VehicleData(4.337,1.825,0.889,2.560);
Models::VehicleMoveModel CarDynamic(VehicleData);


Mat Image2Show(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));

static void dilate(Vec3b Color,int n,Mat & img)
{
    Mat img_backup = img.clone();
    int & rows = img_backup.rows;
    int & cols = img_backup.cols;
    for(int row = n;row<rows-n;row++)
    {
        for(int col = n;col<cols-n;col++)
        {
            bool flag = false;
            for(int i = -n;i<=n;i++)
            {
                for(int j=-n;j<=n;j++)
                {
                    // if(   img_backup.at<Vec3b>(row+i,col+j)[0]==Color[0]
                    //    && img_backup.at<Vec3b>(row+i,col+j)[1]==Color[1]
                    //    && img_backup.at<Vec3b>(row+i,col+j)[2]==Color[2])
                    if(img_backup.at<Vec3b>(row+i,col+j) == Color)
                    {
                        img.at<Vec3b>(row,col) = Color;
                        flag = true;
                        break;
                    }
                }
                if(flag) break;
            }
        }
    }
}

enum Mode{
    Launch,
    PickingRef, PickingCar1, PickingCar2,
    ShowingResult,
};

void Simulate();

Mode curmode;
void EventMouseClick(int event, int x, int y, int flags, void* ustc){
    switch(curmode){
        case Launch:
            if(event == EVENT_LBUTTONDOWN){
                xlist.resize(1);xlist[0] = x;
                ylist.resize(1);ylist[0] = y;
                Image2Show = Mat(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
                Image2Show.at<Vec3b>(y,x) = COLOR_BLACK;
                dilate(COLOR_BLACK,2,Image2Show);
                imshow("Window",Image2Show);
                curmode = PickingRef;
            }
            break;
        case PickingRef:
            if(event == EVENT_LBUTTONDOWN){
                xlist.push_back(x);
                ylist.push_back(y);
                Image2Show = Mat(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
                for(int i=0;i<xlist.size();i++)
                    Image2Show.at<Vec3b>(ylist[i],xlist[i]) = COLOR_BLACK;
                dilate(COLOR_BLACK,2,Image2Show);
                imshow("Window",Image2Show);
            } else if(event == EVENT_RBUTTONDOWN){
                pts.x.resize(xlist.size());
                pts.y.resize(ylist.size());
                for(int i=0;i<pts.size();i++){
                    pts.x[i] = xlist[i]*RESOLUION;
                    pts.y[i] = ylist[i]*RESOLUION;
                }
                Path.SetCurve(pts,1);
                Path.Type = PathSegment::Forward;
                PP.SetPath(Path);
                Image2Show = Mat(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
                for(double s =0;s<Path.length+1;s+=RESOLUION){
                    auto ans = Vector2D(Path(static_cast<double>(s)))/RESOLUION;
                    if(ans.x>0&&ans.x<IMG_WIDTH&&ans.y>0&&ans.y<IMG_HIGHT){
                        Image2Show.at<Vec3b>(ans.y,ans.x) = COLOR_BLACK;
                    }
                }
                dilate(COLOR_BLACK,1,Image2Show);
                imshow("Window",Image2Show);
                curmode = PickingCar1;
            }
            break;
        case PickingCar1:
            if(event == EVENT_LBUTTONDOWN){
                CarPos.x = x*RESOLUION;
                CarPos.y = y*RESOLUION;
                Image2Show.at<Vec3b>(y,x) = COLOR_BLUE;
                imshow("Window",Image2Show);
                curmode = PickingCar2;
                
            }
            break;
        case PickingCar2:
            if(event == EVENT_LBUTTONDOWN){
                CarPos.phi = atan2(y*RESOLUION-CarPos.y,x*RESOLUION-CarPos.x);
                Image2Show.at<Vec3b>((CarPos.y+VehicleData.wheelbase*sin(CarPos.phi))/RESOLUION
                                    ,(CarPos.x+VehicleData.wheelbase*cos(CarPos.phi))/RESOLUION) = COLOR_GREEN;
                imshow("Window",Image2Show);
                dilate(COLOR_GREEN,2,Image2Show);

                PP.SetFirstPos(CarPos,-10,Path.length);
                double angle = PP.KernelFunction(CarPos,LOOKAHEAD);
                auto v = Vector2D(CarPos.x,CarPos.y)+Vector2D(cos(CarPos.phi-M_PI_2),sin(CarPos.phi-M_PI_2))*(VehicleData.wheelbase/std::tan(angle));
                if(v.x>0&&v.x/RESOLUION<IMG_WIDTH&&v.y>0&&v.y/RESOLUION<IMG_HIGHT){
                    Image2Show.at<Vec3b>(v.y/RESOLUION,v.x/RESOLUION) = COLOR_BLUE;
                }
                dilate(COLOR_BLUE,2,Image2Show);

                Simulate();
                for(int i=0;i<ps.size();i++){
                    int x = ps.x[i]/RESOLUION;
                    int y = ps.y[i]/RESOLUION;
                    if(x>0&&x<IMG_WIDTH&&y>0&&y<IMG_HIGHT){
                        Image2Show.at<Vec3b>(y,x) = COLOR_RED;
                    }
                }
                for(int i=0;i<angles.size();i++){
                    double x = ps.x[i];
                    double y = ps.y[i];
                    double R = VehicleData.wheelbase/tan(angles[i]);
                    Vector2D v = Vector2D(x,y)+Vector2D(sin(ps.phi[i]),-cos(ps.phi[i]))*R;
                     x = v.x/RESOLUION;
                     y = v.y/RESOLUION;
                    if(x>0&&x<IMG_WIDTH&&y>0&&y<IMG_HIGHT){
                        Image2Show.at<Vec3b>(y,x) = COLOR_DARK_RED;
                    }
                }
                // dilate(COLOR_RED,2,Image2Show);
                imshow("Window",Image2Show);
                curmode = ShowingResult;
            }
            break;
        case ShowingResult:
            if(event == EVENT_RBUTTONDOWN){
                Simulate();
            }
            if(event == EVENT_LBUTTONDOWN){
                Image2Show = Mat(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
                imshow("Window",Image2Show);
                curmode = Launch;
            }
            break;

    }
}

void Simulate(){
    Points::Pos2D CurPos = CarPos;
    PP.SetFirstPos(CurPos,-10,Path.length);
    ps.x.resize(times);ps.y.resize(times);ps.phi.resize(times);angles.resize(times);
    ps.x[0] = CurPos.x;
    ps.y[0] = CurPos.y;
    ps.phi[0] = CurPos.phi;
    for(int i=1;i<times;i++){
        double angle = PP.KernelFunction(CurPos,LOOKAHEAD);
        CurPos = CarDynamic.MoveBySteerting(CurPos,angle,speed*ts);
        ps.x[i] = CurPos.x;
        ps.y[i] = CurPos.y;
        ps.phi[i] = CurPos.phi;
        angles[i-1] = angle;
    }
    angles[times] = PP.KernelFunction(CurPos,LOOKAHEAD);
}


int main(){

    PP.SetVehicle(VehicleData,max_steer,0.10,0);
    PP.SetStepSize(ts*speed*2,-ts*speed*2);

    namedWindow("Window",WINDOW_NORMAL);
    imshow("Window",Image2Show);
    setMouseCallback("Window", EventMouseClick);
    while(cv::waitKey(0)!=27);
}