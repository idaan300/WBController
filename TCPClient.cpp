// File:          my_controller.cpp
#include <webots/Supervisor.hpp>
#include <webots/LED.hpp>
#include <vector>
#include <webots/distance_sensor.h>
#include <webots/DistanceSensor.hpp>

#include "TCPClient.h"


// All the webots classes are defined in the "webots" namespace
using namespace webots;

//--------------------
//  enums
//--------------------

typedef enum {
    None,
    North,
    East,
    South,
    West
} CardinalDirection;

//--------------------
//  Structs
//--------------------

//TODO

//--------------------
//  Vars
//--------------------

//~~~~~ Constants
constexpr int TIME_STEP_MS = 1;                //Update Interval In Milliseconds
constexpr double LedOn[3] = { 0.0, 1.0, 0.0 };  //Const Color Vector For LedOn, Color Green
constexpr double LedOff[3] = { 1.0, 1.0, 1.0 }; //Const Color Vector For LedOn, Color White
constexpr double sensSens = 500;
//~~~~~ Mutables
Node* RobotNode;
Field* RobotLocation;

//LED
Field* RLedN;
Field* RLedE;
Field* RLedS;
Field* RLedW;

//TCP Client
Field* Port;
Field* Address;

//Distance sensor
webots::DistanceSensor* dsNorth;
webots::DistanceSensor* dsEast;
webots::DistanceSensor* dsSouth;
webots::DistanceSensor* dsWest;

//Supervisor
Supervisor* robot = new Supervisor();

char* RobotDEF;

//--------------------
//  Prototypes
//--------------------

void Init_Intern(char* Name);
void Init_Robot();
void MoveDir(CardinalDirection Dir);
void SetLedDir(CardinalDirection Dir);
void MainLoop();
bool getSensorData(CardinalDirection dir);
int Test();


//--------------------
//  Main
//--------------------

int main(int argc, char** argv) {

    Init_Intern(argv[1]);
    Init_Robot();




    //TCPMain();

    Test();

    MainLoop();





    return 0;
}

void Init_Intern(char* Name) {

    // Init Objects
    printf("NAMEL: %s", Name);
    RobotNode = robot->getFromDef(Name);

    RobotLocation = RobotNode->getField("translation");
    RLedN = RobotNode->getField("ledN");
    RLedE = RobotNode->getField("ledE");
    RLedS = RobotNode->getField("ledS");
    RLedW = RobotNode->getField("ledW");

    Port = RobotNode->getField("Port");
    Address = RobotNode->getField("Address");

    dsNorth = robot->getDistanceSensor("dsNorth");
    dsEast = robot->getDistanceSensor("dsEast");
    dsSouth = robot->getDistanceSensor("dsSouth");
    dsWest = robot->getDistanceSensor("dsWest");

    printf("Init Intern OK\n");
}

void Init_Robot() {
    SetLedDir(None);
    dsNorth->enable(TIME_STEP_MS);
    dsEast->enable(TIME_STEP_MS);
    dsSouth->enable(TIME_STEP_MS);
    dsWest->enable(TIME_STEP_MS);
    printf("Init Robot OK\n");
}

//--------------------
//  Functions
//--------------------


void MainLoop() {
    unsigned long long i = 0;
    while (robot->step(TIME_STEP_MS) != -1) {
        printf("Frame: %lld \n", i++);         //Print Current Frame
        SetLedDir(North);


        std::cout << "Distance sensor north: " << getSensorData(North);
        printf("\n");
        std::cout << "Distance sensor east: " << getSensorData(East);
        printf("\n");
        std::cout << "Distance sensor south: " << getSensorData(South);
        printf("\n");
        std::cout << "Distance sensor west: " << getSensorData(West);
        printf("\n");
    }

}

//-----------------------
//  Robot Functionality
//-----------------------


void SetLedDir(CardinalDirection dir) {

    RLedN->setSFColor(LedOff);
    RLedE->setSFColor(LedOff);
    RLedS->setSFColor(LedOff);
    RLedW->setSFColor(LedOff);
    switch (dir) {

    case North: {
        RLedN->setSFColor(LedOn);
        break;
    }
    case East: {
        RLedE->setSFColor(LedOn);
        break;
    }
    case South: {
        RLedS->setSFColor(LedOn);
        break;
    }
    case West: {
        RLedW->setSFColor(LedOn);
        break;
    }
    default: return;

    }


}


void MoveDir(CardinalDirection dir) {
    const double* oldPos;                           //Declare Temp AddrPtr to store current pos 
    double newPos[3];                               //New Array for updated Pos
    oldPos = RobotLocation->getSFVec3f();           //Get Current Pos

    //Pos Coordinates are [0] = Left / Right, [2] = Up / Down
    //[1] is unused

    memcpy(&newPos, oldPos, sizeof(double) * 3);    //Copy CurrentPos Onto New Pos, Needed because Oldpos is immutable 

    switch (dir) {
    case North: {
        if (oldPos[2] - 0.1 <= 0) return;
        //printf("North\n");
        newPos[2] -= 0.1;
        break;
    }
    case East: {
        if (oldPos[0] + 0.1 >= 1.0) return;
        //printf("East\n");
        newPos[0] += 0.1;
        break;
    }
    case South: {
        if (oldPos[2] + 0.1 >= 1.0) return;
        //printf("South\n");
        newPos[2] += 0.1;
        break;
    }
    case West: {
        if (oldPos[0] - 0.1 <= 0) return;
        //printf("West\n");
        newPos[0] -= 0.1;
        break;
    }
    }
    RobotLocation->setSFVec3f(newPos);
    printf("Moved To: %.1f, %.1f, %.1f \n", oldPos[0], oldPos[1], oldPos[2]);
}


bool getSensorData(CardinalDirection dir) {

    switch (dir) {

    case North: {
        if (dsNorth->getValue() <= sensSens) return true;
        return false;
    }
    case East: {
        if (dsEast->getValue() <= sensSens) return true;
        return false;
    }
    case South: {
        if (dsSouth->getValue() <= sensSens) return true;
        return false;
    }
    case West: {
        if (dsWest->getValue() <= sensSens) return true;
        return false;
    }
    default: return false;

    }

}