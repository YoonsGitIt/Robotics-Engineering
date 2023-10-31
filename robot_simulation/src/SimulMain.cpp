//
// Created by hs on 22. 10. 27.
//

#include <robot_simulation/SimulMain.hpp>

pthread_t RTThreadController;
pthread_t RTThreadStateEstimator;
pthread_t RTThreadMPC;
pthread_t NRTThreadCommand;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/robot_torso_arm/canineV4/urdf/canineV4_with_arm.urdf");

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR) + "canine/canineV4/urdf/canineV4.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);

SimulCommand userCommand;
SimulVisualizer Visualizer(&world, robot, &server);
SimulControlPanel ControlPanel(&world, robot);
SimulControlPanelHigh ControlPanelHigh;
SimulStateEstimator StateEstimator(robot,model);

void* RTControllerThread(void* arg)
{
    std::cout << "entered #Low Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(LOW_CONTROL_dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
//    std::cout << "bf #while" << std::endl;
//    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanel.ControllerFunction();
            StateEstimator.StateEstimatorFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
//            std::cout << "RT Deadline Miss, Low Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void* RTStateEstimator(void* arg)
{
//    std::cout << "entered #rt_State_Estimator_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    const long PERIOD_US = long(ESTIMATOR_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_gettime(CLOCK_REALTIME, &TIME_1);
//        StateEstimator.StateEstimatorFunction();
        clock_gettime(CLOCK_REALTIME, &TIME_2);
//        std::cout<<"elasped time : "<<timediff_us(&TIME_1, &TIME_2)*1e-6<<std::endl;


        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
//            std::cout << "RT Deadline Miss, State estimator thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* RTMPCThread(void* arg)
{
//    std::cout << "entered #High Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(HIGH_CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
//    std::cout << "bf #while" << std::endl;
//    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanelHigh.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
//            std::cout << "RT Deadline Miss, High Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* NRTCommandThread(void* arg)
{
//    std::cout << "entered #Command_NRT_thread" << std::endl;
    while (true)
    {
        userCommand.commandFunction();
        Visualizer.UpdateFootStep();
        usleep(CMD_dT * 1e6);
    }
}

void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->motorStatus = false;
    sharedMemory->gaitChangeFlag = false;
    sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
    sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->gaitState = STAND;
    sharedMemory->localTime = 0;

    for (int i =0 ; i<MPC_HORIZON*4;i++)
    {
        sharedMemory->gaitTable[i] = 1;
    }

    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorVoltage[index] = 0;

        sharedMemory->motorDesiredPosition[index] = 0;
        sharedMemory->motorPrevDesiredPosition[index] = 0;
        sharedMemory->motorDesiredVelocity[index] = 0;
    }

    for(int armIdx = 0; armIdx < ARM_MOTOR_NUM; armIdx++)
    {
        sharedMemory->armMotorPosition[armIdx] = 0;
        sharedMemory->armMotorVelocity[armIdx] = 0;
        sharedMemory->armMotorTorque[armIdx] = 0;
        sharedMemory->armMotorDesiredTorque[armIdx] = 0;
    }

    sharedMemory->basePosition.setZero();
    sharedMemory->baseVelocity.setZero();
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }

    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;

    sharedMemory->baseDesiredQuartPosition[0] = 1.0;
    sharedMemory->baseDesiredQuartPosition[1] = 0.0;
    sharedMemory->baseDesiredQuartPosition[2] = 0.0;
    sharedMemory->baseDesiredQuartPosition[3] = 0.0;
    sharedMemory->isNan = false;

    sharedMemory->gaitIteration = 0;

    sharedMemory->baseDesiredPosition.setZero();
    sharedMemory->baseDesiredVelocity.setZero();
    sharedMemory->baseDesiredEulerPosition.setZero();
    sharedMemory->baseDesiredEulerVelocity.setZero();

    sharedMemory->FSMState = INITIAL;
}

void StartSimulation()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    server.launchServer(8080);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 5, 0, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadStateEstimator, RTStateEstimator, "rt_thread2", 6, 0,NULL);
    int thread_id_rt3 = generate_rt_thread(RTThreadMPC, RTMPCThread, "rt_thread3", 7, 0, NULL);

}