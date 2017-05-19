#include "PID.h"
#include <iostream>
#include<math.h>
#include <uWS/uWS.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp=Kp;
	this->Kd=Kd;
	this->Ki=Ki;

    //Kp=0.0;
    //Kd=0.0;
    //Ki=0.0;

    dkp=0.05;
    dkd=0.5;
    dki=0.0;

    p_error=0.0;
    i_error=0.0;
    d_error=0.0;

    period_count=1250;
    period_index=0;

    tolerance=0.01;

    previous_cte=0.0;
    period_error=0.0;
    //total_error=0.0;

    twiddle_param_switch=0;
    is_twiddle_on=false;
    twiddle_mode=0;
    twiddle_count=0;

    buffer_size=6;
    buffer_index=0.0;
    for (int i=0;i<buffer_size;i++)     buffer[i]=0.0;
	
	
}



void PID::UpdateError(double cte) {

	double square_cte=cte*cte;
	period_index++;
	period_error+=square_cte;

    p_error=cte;
    d_error = cte - previous_cte;
    //total_cte+=fabs(cte);
    i_error+=cte;

    previous_cte = cte;

    //if(fabs(cte)<0.2) Ki=0.01;
    //else Ki=0;

		
}


void PID::Twiddle()
{

    if(twiddle_mode==0){   //mode 0 ,first  increase twiddle loop
        switch(twiddle_param_switch){
        case 0: Kp+=dkp;break;
        case 1: Kd+=dkd;break;
        case 2: Ki+=dki;
        }
        twiddle_mode=1;
        return;
    }

    if(twiddle_mode==1){   //mode 1 ,twiddle 1.1 ratio increase
        if(period_error<twiddle_best_err) {
            twiddle_best_err=period_error;
            switch(twiddle_param_switch){
            case 0: dkp*=1.1;break;
            case 1: dkd*=1.1;break;
            case 2: dki*=1.1;
            }
        }
        else{
            switch(twiddle_param_switch){
            case 0: Kp-=2.0*dkp;break;//Kp-=dkp;dkp*=0.9;break; //mode 2 ,decrease twiddle 0.9 ratio
            case 1: Kd-=2.0*dkd;break;//Kd-=dkd;dkd*=0.9;break;
            case 2: Ki-=2.0*dki;//Ki-=dki;dki*=0.9;
            }
        }
         twiddle_mode=2;
        return;
    }

    if(twiddle_mode==2){   //mode 2 ,decrease twiddle loop
        if(period_error<twiddle_best_err){
            twiddle_best_err=period_error;
            switch(twiddle_param_switch){
            case 0: dkp*=1.1;break;
            case 1: dkd*=1.1;break;
            case 2: dki*=1.1;
            }
        }
        else{
            switch(twiddle_param_switch){
            case 0: Kp+=dkp;dkp*=0.9;break;          //kp+dkp-2dkp+dkp==kp
            case 1: Kd+=dkd;dkd*=0.9;break;
            case 2: Ki+=dki;dki*=0.9;
            }
        }

    }

    //if(dkp<0.3*tolerance) twiddle_param_switch=1;
    //if(dkd<0.3*tolerance) twiddle_param_switch=2;
    //if(dki<0.3*tolerance) twiddle_param_switch=3;

    if(dkp+dkd+dki<tolerance){
    twiddle_mode=3;  // twiddle finished
     std::cout << "final Kp: " << Kp << "   final Ki: " << Ki<<  "    final Kd:" << Kd<<std::endl;
    }
    else{

            twiddle_mode=0;                 //next loop
            twiddle_param_switch=(twiddle_param_switch+1)%2;

           std::cout << "dkp: " << dkp << "   dkd: " << dkd <<std::endl;

        }

    }




double PID::TotalError() {
        double control_output;
        double sum=0.0;

        control_output = 0.25*(-Kp * p_error - Kd * d_error - Ki * i_error);
/*
        if(buffer_index==buffer_size) buffer_index=0;
        buffer[buffer_index]=control_output;
        buffer_index++;
        for (int i=0;i<buffer_size;i++)     sum+=buffer[i];
        control_output=sum/buffer_size;

        if(cte*control_output>0|| fabs(cte)>2.2)
        {
             //Init(previous_Kp,previous_Ki,previous_Kd);
             control_output = -0.2*cte;
             if(cte*control_output>0)
            std::cout << "RESET PID: positive feedback occurs " <<std::endl;
             if(fabs(cte)>2.2)
             std::cout << "RESET PID: too high CTE occurs " <<std::endl;
             Twiddle();
        }

*/

        if(control_output>1.0) control_output=1.0;
        if(control_output<-1.0) control_output=-1.0;


        return control_output;

}

