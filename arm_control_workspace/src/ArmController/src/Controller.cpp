//
// Created by mojo on 03/06/15.
//

#include "Controller.h"

Controller::Controller()
:base(std::tr1::shared_ptr<Motor>(new Motor(static_cast<__uint8_t >(BASE_ID),0,1020))),
 shoulder(std::tr1::shared_ptr<Motor>(new Motor(static_cast<__uint8_t>(SHOULDER_ID),185,700))),
 elbow(std::tr1::shared_ptr<Motor>(new Motor(static_cast<__uint8_t>(ELBOW_ID),171,714))),
 wrist(std::tr1::shared_ptr<Motor>(new Motor(static_cast<__uint8_t>(WRIST_ID)))),
 NO_ERROR(0),
 currentState(0),
 BASE_START_POSITION(70),
 SHOULDER_START_POSITION(177),
 ELBOW_START_POSITION(177),
 WRIST_START_POSITION(0)
{
    if( dxl_initialize(USB2DYNAMIXEL_ID,1) != 0 ){
        base->initializeMotor();
        shoulder->initializeMotor();
        elbow->initializeMotor();
        wrist->initializeMotor();
    }

    if( base->isOpened())
        base->setCurrentAngle(BASE_START_POSITION);
    if( shoulder->isOpened())
        shoulder->setCurrentAngle(SHOULDER_START_POSITION);
    if( elbow->isOpened())
        elbow->setCurrentAngle(ELBOW_START_POSITION);
    if( wrist->isOpened())
        wrist->setCurrentAngle(WRIST_START_POSITION);

}

Controller::~Controller() {
    try {
        dxl_terminate();
    } catch (...) {
        ROS_ERROR("An exception occured while trying to close the Dynamixel driver.");
    }
}


bool Controller::moveAbsoluteMotor(const Controller::MOTOR_ID& motor, const float targetAngle) {
    ROS_INFO("Moving absolute motor");
    if(motor.id<BASE_ID || motor.id>WRIST_ID ) {
        ROS_WARN("Motor ID is no within boundaries");
        return false;
    }
    getMotor(motor)->setCurrentAngle(targetAngle);
    return true;
}

bool Controller::moveRelativeTool(const float tX, const float tY, const float tZ,
                                  const float roll, const float pitch, const float yaw) {
    return false;
}


Motor* Controller::getMotor(const Controller::MOTOR_ID& motor)const{
    Motor* motorPtr;
    switch(motor.id){
        case (BASE_ID):{
            motorPtr = base.get();
            break;
        }
        case (SHOULDER_ID):{
            motorPtr = shoulder.get();
            break;
        }
        case (ELBOW_ID):{
            motorPtr = elbow.get();
            break;
        }
        case (WRIST_ID):{
            motorPtr = wrist.get();
            break;
        }
    }
    return motorPtr;
}