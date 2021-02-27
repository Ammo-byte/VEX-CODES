      /*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       AutonReplay.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include "vex.h"
#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

template <typename T>
std::string to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

// Variables
int previous [8] = {0,0,0,0}; //Previous motor value for each motor
bool ValuesChanged = false; // Test If Values Changed
int relativeTime = 0;
int AR_Motor_Plus_1;
std::string changes [8] = {"","","",""};

void AR_BEGINSegment(){
	ValuesChanged = false;
}

// Prints the Motor Power to the Debug Stream. USE ONE PER MOTOR
void AR_GenerateMotorPower(int AR_Motor, int AR_Power){
  //string ToPrint = strcat("motor[ %d ",AR_Motor," ] = ",AR_Power,";");
	if ((previous[AR_Motor] != AR_Power) && (abs(previous[AR_Motor]-AR_Power) > 15)){
		if(ValuesChanged == false){
		relativeTime = relativeTime + 10;
		std::string RelativeTimeFormat = "wait(";
		std::string RelativeTimeString = to_string(relativeTime);
    RelativeTimeFormat +=  RelativeTimeString;
    RelativeTimeFormat += ", msec);";
		printf("%s \n", RelativeTimeFormat.c_str());
	}
		std::string ToPrint = "motor";
		AR_Motor_Plus_1 = AR_Motor + 1; //All motors shifted off by one
		std::string AR_Motor_String = to_string(AR_Motor_Plus_1);
    ToPrint += AR_Motor_String;
    ToPrint += ".spin(forward, ";
		std::string AR_Power_String = to_string(AR_Power);
    ToPrint += AR_Power_String;
    ToPrint += ", velocityUnits::pct);";
		changes[AR_Motor] = ToPrint;
		previous[AR_Motor] = AR_Power;
		ValuesChanged = true;

	}
}

// RUN THIS AT THE END OF ONE LOOP OF YOUR DRIVER CONTROL RECORDER.
// This prevents your code and Cortex from being overloaded.

void AR_ENDSegment(){
	if(ValuesChanged){
		int i;
		for(i=0;i<=4;i++){
			if(changes[i]!=""){
				printf("%s \n", changes[i].c_str());
				changes[i]="";
				relativeTime=0;
			}

		}
	}
	wait(40, msec);
	relativeTime = relativeTime + 40;
}