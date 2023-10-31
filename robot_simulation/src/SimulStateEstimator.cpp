//
// Created by hs on 22. 10. 27.
//

#include <robot_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

CanineFilter::Vec3LPF vecLPF(ESTIMATOR_dT,1000);

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot, RigidBodyDynamics::Model* model)
    : mDt(0.001)
    , mGain(Eigen::Matrix<double,23,23>::Identity() * 50)
    , mModel(model)
    , mRobot(robot)
    , mPosition(raisim::VecDyn(24))
    , mVelocity(raisim::VecDyn(23))
    , mGeneralizedForce(Eigen::VectorXd::Zero(23))
    , mbIsFirstFiltering(true)
    , mbIsFirstRun(true)
{
    for(int i=0; i<4; i++)
    {
        mbContactState[i] = true;
        mbIsFirstCon[i] = true;
    }
    mPrevBasePos[0] = 0.003;
    mPrevBasePos[1] = 0.001;
    mPrevBasePos[0] = 0.04413;
    mBeta = Eigen::VectorXd::Zero(18);
    mCalcTorque = Eigen::VectorXd::Zero(23);
    mH = Eigen::MatrixXd::Zero(18,18);
    mMomentum = Eigen::VectorXd::Zero(18);
    mPrevH = Eigen::MatrixXd::Zero(18,18);
    mPrevQ = Eigen::VectorXd::Zero(19);
    mPrevQd = Eigen::VectorXd::Zero(18);
    mQ = Eigen::VectorXd::Zero(24);
    mQd = Eigen::VectorXd::Zero(23);
    mQdd = Eigen::VectorXd::Zero(18);
    mResidual = Eigen::VectorXd::Zero(18);
    mTau = Eigen::VectorXd::Zero(18);

    mEstState = Eigen::VectorXd::Zero(12);
    mState = Eigen::VectorXd::Zero(12);
    mErrorCov = Eigen::Matrix<double,12,12>::Identity();
    mKalmanGain = Eigen::Matrix<double,12,6>::Identity();
    mMeasureMat.setZero();
    mMeasurements = Eigen::VectorXd::Zero(6);
    tempIMU = Eigen::VectorXd::Zero(3);
    tempPrevIMU = Eigen::VectorXd::Zero(3);
    tempPrevV = Eigen::VectorXd::Zero(3);

    std::cout << "Degree of freedom overview : " << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*mModel);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*mModel);

    std::cout << "q : " << mModel->q_size << ", qdot : " << mModel->qdot_size << std::endl;

//    for( int i=0; i<18; i++)
//    {
//        std::cout << "body name " << i << " : "<< mModel->GetBodyName(i) << std::endl;
//    }
//    std::cout << mGain << std::endl;
}

void SimulStateEstimator::StateEstimatorFunction()
{
    updateState();
    updateRBDL();
    getJointState();
    getRobotAngulerState();
    getRobotFootPosition();
    getContactState();
    getRobotLinearState();
}

void SimulStateEstimator::updateState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();
    for(int i=0; i<18; i++)
    {
        mGeneralizedForce[i] = mRobot->getGeneralizedForce()[i];
    }
}

void SimulStateEstimator::getJointState()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
    for (int i = 0; i < ARM_MOTOR_NUM; i++)
    {
        sharedMemory->armMotorPosition[i] = mPosition[i+7+MOTOR_NUM];
        sharedMemory->armMotorVelocity[i] = mVelocity[i+6+MOTOR_NUM];
    }
}

void SimulStateEstimator::getRobotAngulerState()
{
    for(int idx=0; idx<3; idx++)
    {
        sharedMemory->baseEulerVelocity[idx] = mVelocity[idx+3];
    }
    for(int idx=0; idx<4; idx++)
    {
        sharedMemory->baseQuartPosition[idx] = mPosition[idx+3];

    }
    TransformQuat2Euler(sharedMemory->baseQuartPosition, mRawEulerAngle);

    if(tempV[2] > 3.14 && mRawEulerAngle[2] < 3.1/2.0)
    {
        mMulti++;
    }
    if(tempV[2] < -3.14 && mRawEulerAngle[2] > -3.1/2.0)
    {
        mMulti--;
    }

    tempV[2] = mRawEulerAngle[2];
    sharedMemory->baseEulerPosition[2] = mRawEulerAngle[2] + mMulti * 2.0 * 3.141592;
}

void SimulStateEstimator::getRobotFootPosition()
{
    TransMatBody2Foot(&mTransMat[LF_IDX], LF_IDX,
                      sharedMemory->motorPosition[LF_IDX*3],
                      sharedMemory->motorPosition[LF_IDX*3+1],
                      sharedMemory->motorPosition[LF_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[RF_IDX], RF_IDX,
                      sharedMemory->motorPosition[RF_IDX*3],
                      sharedMemory->motorPosition[RF_IDX*3+1],
                      sharedMemory->motorPosition[RF_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[LB_IDX], LB_IDX,
                      sharedMemory->motorPosition[LB_IDX*3],
                      sharedMemory->motorPosition[LB_IDX*3+1],
                      sharedMemory->motorPosition[LB_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[RB_IDX], RB_IDX,
                      sharedMemory->motorPosition[RB_IDX*3],
                      sharedMemory->motorPosition[RB_IDX*3+1],
                      sharedMemory->motorPosition[RB_IDX*3+2]);

    Mat3<double> Rot = GetBaseRotationMat(sharedMemory->baseQuartPosition);
    for (int leg=0; leg<4; leg++)
    {
        sharedMemory->bodyFootPosition[leg] = mTransMat[leg].block(0,3,3,1);
        sharedMemory->globalFootPosition[leg] = sharedMemory->basePosition + Rot*sharedMemory->bodyFootPosition[leg];
    }

    if(mbIsFirstRun)
    {
        this->setOffset();
        mbIsFirstRun = 0;
    }
}

void SimulStateEstimator::getRobotLinearState()
{
    Mat3<double> Rot = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition);
    for (int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
    }

//    this->tempImuUpdate();
//    this->doEKF();
//
//    for(int i=0; i<3; i++)
//    {
//        sharedMemory->basePosition[i] = mEstState[i];
//        sharedMemory->baseVelocity[i] = mEstState[i+3];
//    }
}

void SimulStateEstimator::getContactState()
{
//    mTau.setZero();
//    double res[4];
//    RigidBodyDynamics::CompositeRigidBodyAlgorithm(* mModel, mQ, mH, true);
//    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQd, Eigen::VectorXd::Zero(18),mTau);
//
//    mBeta = mTau - (mH - mPrevH) / mDt * mQd;
//    mMomentum = mMomentum + mCalcTorque * mDt - mBeta * mDt + mResidual * mDt;
//    mResidual = mGain * (-mMomentum + mH * mQd);
//
//    res[LF_IDX] = sqrt(pow(mResidual[6],2) + pow(mResidual[7],2) + pow(mResidual[8],2));
//    res[LB_IDX] = sqrt(pow(mResidual[9],2) + pow(mResidual[10],2) + pow(mResidual[11],2));
//    res[RF_IDX] = sqrt(pow(mResidual[12],2) + pow(mResidual[13],2) + pow(mResidual[14],2));
//    res[RB_IDX] = sqrt(pow(mResidual[15],2) + pow(mResidual[16],2) + pow(mResidual[17],2));
//
//    mPrevH = mH;
//    mPrevQ = mQ;
//    mPrevQd = mQd;
//
//    if(mbContactState[LF_IDX] == 1 && res[LF_IDX] < 4)
//    {
//        mbContactState[LF_IDX] = 0;
//    }
//    else if(mbContactState[LF_IDX] == 0 && res[LF_IDX] > 8)
//    {
//        mbContactState[LF_IDX] = 1;
//    }
//
//    if(mbContactState[RF_IDX] == 1 && res[RF_IDX] < 4)
//    {
//        mbContactState[RF_IDX] = 0;
//    }
//    else if(mbContactState[RF_IDX] == 0 && res[RF_IDX] > 8)
//    {
//        mbContactState[RF_IDX] = 1;
//    }
//
//    if(mbContactState[LB_IDX] == 1 && res[LB_IDX] < 4)
//    {
//        mbContactState[LB_IDX] = 0;
//    }
//    else if(mbContactState[LB_IDX] == 0 && res[LB_IDX] >10)
//    {
//        mbContactState[LB_IDX] = 1;
//    }
//
//    if(mbContactState[RB_IDX] == 1 && res[RB_IDX] < 4)
//    {
//        mbContactState[RB_IDX] = 0;
//    }
//    else if(mbContactState[RB_IDX] == 0 && res[RB_IDX] >10)
//    {
//        mbContactState[RB_IDX] = 1;
//    }

    for(int i=0; i<4; i++)
    {
        mbContactState[i] = sharedMemory->gaitTable[i];
        sharedMemory->contactState[i] = mbContactState[i];
    }
}

void SimulStateEstimator::updateRBDL()
{
    mQ[0] = sharedMemory->basePosition[0];
    mQ[1] = sharedMemory->basePosition[1];
    mQ[2] = sharedMemory->basePosition[2];

    mQ[3] = sharedMemory->baseQuartPosition[1]; /// q_x
    mQ[4] = sharedMemory->baseQuartPosition[2]; /// q_y
    mQ[5] = sharedMemory->baseQuartPosition[3]; /// q_z
    mQ[23] = sharedMemory->baseQuartPosition[0]; /// q_w

    mQ[6] = sharedMemory->motorPosition[0];
    mQ[7] = sharedMemory->motorPosition[1];
    mQ[8] = sharedMemory->motorPosition[2];

    mQ[9] = sharedMemory->motorPosition[6];
    mQ[10] = sharedMemory->motorPosition[7];
    mQ[11] = sharedMemory->motorPosition[8];

    mQ[12] = sharedMemory->motorPosition[3];
    mQ[13] = sharedMemory->motorPosition[4];
    mQ[14] = sharedMemory->motorPosition[5];

    mQ[15] = sharedMemory->motorPosition[9];
    mQ[16] = sharedMemory->motorPosition[10];
    mQ[17] = sharedMemory->motorPosition[11];

//    mQ[18] = sharedMemory->armMotorPosition[0];
//    mQ[19] = sharedMemory->armMotorPosition[1];
//    mQ[20] = sharedMemory->armMotorPosition[2];
//    mQ[21] = sharedMemory->armMotorPosition[3];
//    mQ[22] = sharedMemory->armMotorPosition[4];

    mQd[0] = sharedMemory->baseVelocity[0]; /// base_TX
    mQd[1] = sharedMemory->baseVelocity[1]; /// base_TY
    mQd[2] = sharedMemory->baseVelocity[2]; /// base_TZ

    mQd[3] = sharedMemory->baseEulerVelocity[2]; /// base_RZ
    mQd[4] = sharedMemory->baseEulerVelocity[1]; /// base_RY
    mQd[5] = sharedMemory->baseEulerVelocity[0]; /// base_RX

    mQd[6] = sharedMemory->motorVelocity[0]; /// LF_hip_RX
    mQd[7] = sharedMemory->motorVelocity[1];/// LF_thigh_RY
    mQd[8] = sharedMemory->motorVelocity[2];/// LF_calf_RY

    mQd[9] = sharedMemory->motorVelocity[6]; /// LB_hip_RX
    mQd[10] = sharedMemory->motorVelocity[7];/// LB_thigh_RY
    mQd[11] = sharedMemory->motorVelocity[8];/// LB_calf_RY

    mQd[12] = sharedMemory->motorVelocity[3];/// RF_hip_RX
    mQd[13] = sharedMemory->motorVelocity[4];/// RF_thigh_RX
    mQd[14] = sharedMemory->motorVelocity[5];/// RF_calf_RX

    mQd[15] = sharedMemory->motorVelocity[9];/// RB_hip_RX
    mQd[16] = sharedMemory->motorVelocity[10];/// RB_thigh_RX
    mQd[17] = sharedMemory->motorVelocity[11];/// RB_calf_RX

//    mQd[18] = sharedMemory->armMotorVelocity[0];
//    mQd[19] = sharedMemory->armMotorVelocity[1];
//    mQd[20] = sharedMemory->armMotorVelocity[2];
//    mQd[21] = sharedMemory->armMotorVelocity[3];
//    mQd[22] = sharedMemory->armMotorVelocity[4];

    mCalcTorque[0] = 0;
    mCalcTorque[1] = 0;
    mCalcTorque[2] = 0;

    mCalcTorque[3] = 0;
    mCalcTorque[4] = 0;
    mCalcTorque[5] = 0;

    mCalcTorque[6] = sharedMemory->motorDesiredTorque[0];
    mCalcTorque[7] = sharedMemory->motorDesiredTorque[1];
    mCalcTorque[8] = sharedMemory->motorDesiredTorque[2];

    mCalcTorque[9] = sharedMemory->motorDesiredTorque[6];
    mCalcTorque[10] = sharedMemory->motorDesiredTorque[7];
    mCalcTorque[11] = sharedMemory->motorDesiredTorque[8];

    mCalcTorque[12] = sharedMemory->motorDesiredTorque[3];
    mCalcTorque[13] = sharedMemory->motorDesiredTorque[4];
    mCalcTorque[14] = sharedMemory->motorDesiredTorque[5];

    mCalcTorque[15] = sharedMemory->motorDesiredTorque[9];
    mCalcTorque[16] = sharedMemory->motorDesiredTorque[10];
    mCalcTorque[17] = sharedMemory->motorDesiredTorque[11];

//    mCalcTorque[18] = sharedMemory->armMotorDesiredTorque[0];
//    mCalcTorque[19] = sharedMemory->armMotorDesiredTorque[1];
//    mCalcTorque[20] = sharedMemory->armMotorDesiredTorque[2];
//    mCalcTorque[21] = sharedMemory->armMotorDesiredTorque[3];
//    mCalcTorque[22] = sharedMemory->armMotorDesiredTorque[4];
}

void SimulStateEstimator::setOffset()
{
    mOffset[LF_IDX] = -sharedMemory->bodyFootPosition[LF_IDX];
    mOffset[RF_IDX] = -sharedMemory->bodyFootPosition[RF_IDX];
    mOffset[LB_IDX] = -sharedMemory->bodyFootPosition[LB_IDX];
    mOffset[RB_IDX] = -sharedMemory->bodyFootPosition[RB_IDX];
}

void SimulStateEstimator::doEKF()
{
    Vec3<double> globalBodyFootPosition[4];
    Mat3<double> Rot = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition);
    for(int leg=0; leg<2; leg++)
    {
        globalBodyFootPosition[leg] = Rot.inverse() * sharedMemory->bodyFootPosition[leg];
    }

    if(mbIsFirstFiltering)
    {
        mState[0] = 0;
        mState[1] = 0;
        mState[2] = 0.044;

        mState[3] = 0;
        mState[4] = 0;
        mState[5] = 0;

        mState[6] = globalBodyFootPosition[0][0];
        mState[7] = globalBodyFootPosition[0][1];
        mState[8] = 0;

        mState[9] = globalBodyFootPosition[1][0];
        mState[10] = globalBodyFootPosition[1][1];
        mState[11] = 0;

        for(int i=0; i<12; i++)
        {
            mEstState[i] = mState[i];
        }

        ///predict matrix (A)
        mPredictMat = Eigen::Matrix<double,12,12>::Identity();
        for(int i=0; i<3; i++)
        {
            mPredictMat(i,i+3) = mDt;
        }
        ///input matrix (B)
        mInputMat.setZero();
        for(int i=0; i<3; i++)
        {
            mInputMat(i,i) = (1.0/2.0) * mDt * mDt;
            mInputMat(i+3,i) = mDt;
        }

        ///error cov
        for(int i=0; i<12; i++)
        {
            mErrorCov(i,i) = 0.005*0.005;
        }

        mNoiseQ = Eigen::Matrix<double,12,12>::Identity() * 0.9;
        mNoiseR = Eigen::Matrix<double,6,6>::Identity() * 0.1;

        mbIsFirstFiltering = false;
    }

    for(int i=0; i<2; i++)
    {
        if(mbContactState[i])
        {
            if(mbIsFirstCon[i])
            {
                mEstState[3*i + 6] = mEstState[0] + globalBodyFootPosition[i][0];
                mEstState[3*i + 7] = mEstState[1] + globalBodyFootPosition[i][1];
                mEstState[3*i + 8] = mEstState[2] + globalBodyFootPosition[i][2];
                mbIsFirstCon[i] = 0;
            }
            mNoiseQ(3*i+6,3*i+6) = 0.1;
            mNoiseQ(3*i+7,3*i+7) = 0.1;
            mNoiseQ(3*i+8,3*i+8) = 0.1;
        }
        else
        {
            mbIsFirstCon[i] = 1;
            mNoiseQ(3*i+6,3*i+6) = 1000000;
            mNoiseQ(3*i+7,3*i+7) = 1000000;
            mNoiseQ(3*i+8,3*i+8) = 1000000;
        }
    }

    ///measurement matrix update
    for(int roww=0; roww<3; roww++)
    {
        for(int column=0; column<3; column++)
        {
            mMeasureMat(roww,column) =
            mMeasureMat(roww + 3,column) = -Rot(roww,column);

            mMeasureMat(roww,column + 6) =
            mMeasureMat(roww + 3,column + 9) = Rot(roww,column);
        }
    }

    ///time update
    mState = mPredictMat * mEstState + mInputMat * tempIMU;
    mErrorCov = mPredictMat * mErrorCov * mPredictMat.transpose() + mNoiseQ;

    ///calculate kalman gain
    mKalmanGain = mErrorCov * mMeasureMat.transpose() * (mMeasureMat * mErrorCov * mMeasureMat.transpose() + mNoiseR).inverse();

    ///measurement update
    for(int leg=0; leg<2; leg++)
    {
        for(int row=0; row<3; row++)
        {
            mMeasurements(3*leg+row) = sharedMemory->bodyFootPosition[leg][row];
        }
    }
    mEstState = mState + mKalmanGain * (mMeasurements - mMeasureMat * mState);

    ///error covariance update
    mErrorCov = mErrorCov - mKalmanGain * mMeasureMat * mErrorCov;

    ///show the outputs

}

/**
 * just for simulation.(to get acceleration)
 */
void SimulStateEstimator::tempImuUpdate()
{
    tempIMU = (sharedMemory->baseVelocity - tempPrevV)/mDt;

    if(tempIMU[0] == 0 && tempIMU[1] == 0 && tempIMU[2] == 0)
    {
        for(int i=0; i<3; i++)
        {
            tempIMU[i] = tempPrevIMU[i];
        }
    }

    for(int i=0; i<3; i++)
    {
        tempPrevIMU[i] = tempIMU[i];
        sharedMemory->tempIMU[i] = tempIMU[i];
        tempPrevV[i] = sharedMemory->baseVelocity[i];
    }
}

void SimulStateEstimator::showOutputs()
{
    for(int i=0; i<12; i++)
    {
        std::cout << std::setw(14) << std::right << mEstState[i];
    }
    std::cout << std::endl;
}