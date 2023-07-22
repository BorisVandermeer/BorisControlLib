/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple Simulator of PID Controller
 * 
*********************************************************************/
#include<vector>
#include<random>
#include<ctime>
#include<assert.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#include<controllers/PIDController.h>

using namespace std;
using namespace cv;

#define IMG_HIGHT 2000
#define IMG_WIDTH 3000

#define COLOR_WHITE Vec3b(255,255,255)
#define COLOR_BLACK Vec3b(0,0,0)
#define COLOR_GREEN Vec3b(0,255,0)
#define COLOR_RED   Vec3b(0,0,255)
#define COLOR_BLUE  Vec3b(255,0,0)

Controller::PIDController PID;
const int times = 3000;
const double ts = 0.05;
const double totalt = ts*times;

vector<double> t(times);
vector<double> v(times);
vector<double> errs(times);
vector<double> outs(times);
vector<double> target(times);

template<class T> 
T max(vector<T> const & nums ){
    T ans = t[0];
    for(auto & num : nums){
        if(ans<num) ans = num;
    }
    return ans;
}

template<class T> 
T min(vector<T> const & nums ){
    T ans = t[0];
    for(auto & num : nums){
        if(ans>num) ans = num;
    }
    return ans;
}

static void dilate(Vec3b Color,int n,Mat & img)
{
    Mat img_backup = img.clone();
    int & rows = img_backup.rows;
    int & cols = img_backup.cols;
    for(int row = n;row<rows-n;row++){
        for(int col = n;col<cols-n;col++){
            bool flag = false;
            for(int i = -n;i<=n;i++){
                for(int j=-n;j<=n;j++){
                    if(img_backup.at<Vec3b>(row+i,col+j) == Color){
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

class SISO{

public:
    double operator()(double input){
        double ans;
        cur_v += input*ts + noise();
        ans = cur_v;
        return ans;
    }

private:
    double noise(){
        double ans = norm(engine);
        // double ans = engine();
        ans = max(ans,-max_noise);
        ans = min(ans,max_noise);
        return ans;
    }
    double cur_v = 0;
    minstd_rand engine{static_cast<long unsigned int>(time(0))};
    const double max_noise = 0.1;
    const double sigma = max_noise/3;
    normal_distribution<double> norm{0,sigma};
};

void SetTarget(){
    for(int i = 0;i<(times/10);i++){
        target[i] = 0;
    }
    for(int i = (times/10);i<times;i++){
        target[i] = 10;
    }
}

int main(){
    Controller::PIDController::Parameter ks{.kp = 3, .ki = 0.1, .kd = -0.1};
    Controller::PIDController::SaftyLimits limits{.max_int = 4,.max_out = 3};
    PID.SetParameters(ks);
    PID.SetDefaultTs(ts);
    PID.SetSaftyLimits(limits);
    SetTarget();

    SISO Car;

    double cur_v = 0;
    unsigned char warn;
    // Simulator
    for(int i = 0; i<times; i++){
        t[i] = i*ts;
        v[i] = cur_v;
        errs[i] = cur_v-target[i];
        outs[i] = PID.KernelFunction(errs[i],warn);
        cur_v = Car(outs[i]);
    }

    
    namedWindow("Window",WINDOW_NORMAL);
    Mat img2show(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));

    double v_max = max(v);
    double v_min = min(v);
    double target_max  = max(target);
    double target_min  = min(target);
    v_max = max(v_max,target_max);
    v_min = min(v_min,target_min);
    double ppvale = v_max - v_min;
    double kshow = 1.1;
    double offset = (kshow-1)/2;
    cout<<"ShowingV, vmax = "<<v_max<<", vmin = "<<v_min<<endl;

    for(int i = 0;i<times;i++){
        int x = i*IMG_WIDTH/times;
        int y = ((v[i]-v_min)/ppvale/kshow+offset) *IMG_HIGHT;
        if(y>=0&&y<IMG_HIGHT)
            img2show.at<Vec3b>(IMG_HIGHT-1-y,x) = COLOR_RED;
    }
    
    dilate(COLOR_RED,1,img2show);
    imshow("Window",img2show);
    while(cv::waitKey()!=27);

    img2show = Mat(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
    
    double a_max = max(outs);
    double a_min = min(outs);
    ppvale = a_max - a_min;
    kshow = 1.1;
    offset = (kshow-1)/2;
    cout<<"ShowingA, amax = "<<a_max<<", amin = "<<a_min<<endl;

    for(int i = 0;i<times;i++){
        int x = i*IMG_WIDTH/times;
        int y = ((outs[i]-a_min)/ppvale/kshow+offset) *IMG_HIGHT;
        if(y>=0&&y<IMG_HIGHT)
            img2show.at<Vec3b>(IMG_HIGHT-1-y,x) = COLOR_RED;
    }
    
    // dilate(COLOR_RED,1,img2show);/
    imshow("Window",img2show);
    while(cv::waitKey()!=27);

    return 0;
}