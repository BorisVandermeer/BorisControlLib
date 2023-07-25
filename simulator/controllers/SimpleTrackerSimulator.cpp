/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple Simulator of SimpleSimulator Controller
 * 
*********************************************************************/


#include<iostream>
#include<PNCUtils/Timer.h>
#include<PNCUtils/SimpleTimer.h>

using namespace std;
using namespace PNCUtils;

std::atomic<int> counter(0);

SimpleTimer timer;
Timer recorder;

void Func(){
    counter++;
    long long a = 0;
    while(a<100000000) a++;
    recorder.Toc("123");
}

int main(){

    cout<<"start"<<endl;
    recorder.Begin();

    timer.startSync(100,Func);

    while(counter<10);

    timer.stop();

    auto data = recorder.getData();

    for(auto  p : data){
        cout << p.first << " : " << p.second <<endl;
    }

    return 0;



}


