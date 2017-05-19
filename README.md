# CarND-Controls-PID
## 1.Introduction
    The objective of this project was to implement a PID controller in c++ to make the car driving safely on track in the simulator provided from Udacity. The implementation of the project is in the `PID.h`,'PID.cpp','main.cpp', and the reflection  is in this REFLECTION.MD ,and the output video is in the file `output_video.mp4` .

## 2. Throtte setting
	In my idea,like a human drives, the throttle value should be changed according to the present position and  speed.
	To simplize the manipulation,i just make the throttle function of cte:the more the car leave the middle of road,the smaller the throttle value.
	I assume the following:
         if(fabs(cte)<1.8) msgJson["throttle"] =0.4*(1.0-fabs(cte)/1.8);
          else  msgJson["throttle"] =0;
    
	
## 3 EFFECT of PID
	--Manually trying
     To get the first idea how the PID infulence the car driving, I mannually select different PID parameters.
	 and i found as follows:
	P: react quickly to CTE ,good for sharp turns ,but easily lead to oscillation
	I: make compensation to system bias, in our project ,the introducion a high I parameter will make the car react latently,so ether set it to zero or a very little value
	D: Reduce the oscillation and make a smoother turns
	 
	 At the end ,i make a PID controller works well around the track with PID parameters equaling to {0.3, 0.0, 4.0}
	
## 4 Twiddle algorithm
    Then i introduce the twidlle algorithm to try to find a optimal pid value with the least CTE error of a whole round.
	i found a large dki will lead the car drive out of road, so i choose a smaller PID jump gap with   dkp=0.05;    dkd=0.5;    dki=0.0;
	and i set the tollerance to 0.01 to make the twiddle funtion finishing tuning.
	
	After 18 rounds of car driving ,i got a steady PID parameters{0.48,0.0,5.0}

## 5 Reflections

    So this is a very intuitive project to see the PID influence. The car can react quickly and smoothy on the track .But i still think maybe several predefined PID parameters will performance better than one PID parameters for all the situation.And i am reading some fuzzy control paper.I think it will be interesting to compare the fuzzy controller and the next model predictive controller. 
    
    



