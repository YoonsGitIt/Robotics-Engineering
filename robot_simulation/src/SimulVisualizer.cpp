//4
// Created by hs on 22. 10. 28.
//

#include <robot_simulation/SimulVisualizer.hpp>

extern pSHM sharedMemory;

SimulVisualizer::SimulVisualizer(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
        : mWorld(world)
        , mRobot(robot)
        , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(LOW_CONTROL_dT);
    auto ground = mWorld->addGround();
    ground->setAppearance("simple");
    mRobot->setName("Canine");

    mBasePositionBox = mServer->addVisualBox("base_position", 0.02, 0.02, 0.01, 1, 1, 1);
    mDesiredFootPosBox[0] = mServer->addVisualBox("LF_desired_foot", 0.02, 0.02, 0.01, 1, 0, 0);
    mDesiredFootPosBox[1] = mServer->addVisualBox("RF_desired_foot", 0.02, 0.02, 0.01, 0, 1, 0);
    mDesiredFootPosBox[2] = mServer->addVisualBox("LB_desired_foot", 0.02, 0.02, 0.01, 1, 0, 1);
    mDesiredFootPosBox[3] = mServer->addVisualBox("RB_desired_foot", 0.02, 0.02, 0.01, 0, 1, 1);

    mGlobalFootPosBox[0] = mServer->addVisualBox("LF_global_foot", 0.02, 0.02, 0.01, 0.5, 0, 0);
    mGlobalFootPosBox[1] = mServer->addVisualBox("RF_global_foot", 0.02, 0.02, 0.01, 0, 0.5, 0);
    mGlobalFootPosBox[2] = mServer->addVisualBox("LB_global_foot", 0.02, 0.02, 0.01, 0.5, 0, 0.5);
    mGlobalFootPosBox[3] = mServer->addVisualBox("RB_global_foot", 0.02, 0.02, 0.01, 0, 0.5, 0.5);

    mLocalFootPosBox[0] = mServer->addVisualBox("LF_local_foot", 0.02, 0.02, 0.01, 0.5, 0, 0);
    mLocalFootPosBox[1] = mServer->addVisualBox("RF_local_foot", 0.02, 0.02, 0.01, 0, 0.5, 0);
    mLocalFootPosBox[2] = mServer->addVisualBox("LB_local_foot", 0.02, 0.02, 0.01, 0.5, 0, 0.5);
    mLocalFootPosBox[3] = mServer->addVisualBox("RB_local_foot", 0.02, 0.02, 0.01, 0, 0.5, 0.5);
    initRobotPose();
}

SimulVisualizer::~SimulVisualizer()
{
    mServer->killServer();
}

void SimulVisualizer::UpdateFootStep()
{
    mBasePositionBox->setPosition(sharedMemory->basePosition[0], sharedMemory->basePosition[1], sharedMemory->basePosition[2]);
    for (int i = 0; i < 4; i++)
    {
//        mDesiredFootPosBox[i]->setPosition(sharedMemory->desiredFootPosition[i][0],sharedMemory->desiredFootPosition[i][1],sharedMemory->desiredFootPosition[i][2]);
        mGlobalFootPosBox[i]->setPosition(sharedMemory->globalFootPosition[i][0],sharedMemory->globalFootPosition[i][1],sharedMemory->globalFootPosition[i][2]);
//        mLocalFootPosBox[i]->setPosition(sharedMemory->bodyFootPosition[i][0],sharedMemory->bodyFootPosition[i][1],sharedMemory->bodyFootPosition[i][2]+0.5);
    }
}

void SimulVisualizer::initRobotPose()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(mRobot->getGeneralizedVelocityDim());
    initialJointPosition.setZero();
    initialJointVelocity.setZero();

    initialJointPosition[2] = 0.1;
    initialJointPosition[3] = 1.0;

    for (int idx=0; idx<4; idx++)
    {
        initialJointPosition[idx*3+7] = 0.0;//hip roll
        initialJointPosition[idx*3+8] = 126*D2R;//hip pitch
        initialJointPosition[idx*3+9] = -160*D2R;//knee pitch
    }

    initialJointPosition[19] = 0.0;
    initialJointPosition[20] = 0.0;
    initialJointPosition[21] = 0.0;
    initialJointPosition[22] = 0.0;
    initialJointPosition[23] = 0.0;

    Eigen::VectorXd jointPgain(mRobot->getDOF());
    Eigen::VectorXd jointDgain(mRobot->getDOF());

    jointPgain[19] = 40.0;
    jointPgain[20] = 40.0;
    jointPgain[21] = 40.0;
    jointPgain[22] = 15.0;

    jointDgain[19] = 1.0;
    jointDgain[20] = 1.0;
    jointDgain[21] = 1.0;
    jointDgain[22] = 0.2;

    mRobot->setPdGains(jointPgain, jointDgain);

/*
    /// for checking urdf state
    initialJointPosition.setZero();
    initialJointPosition[2] =1;
    initialJointPosition[3] =1.0;
*/

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    mRobot->setGeneralizedVelocity(initialJointVelocity);
}
