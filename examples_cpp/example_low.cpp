#include "RotationTools.h"
#include "aolion_driver.h"
#include "lowcontroller.h"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "yaml-cpp/yaml.h"

using namespace legged;

LowController *ctrl;

// Controllerbase controllerbase;
std::vector<std::string> walkjointNames{
    "leg_l1_joint", "leg_r1_joint",  "waist_1_joint", "leg_l2_joint",
    "leg_r2_joint", "waist_2_joint", "leg_l3_joint",  "leg_r3_joint",
    "arm_l1_joint", "arm_r1_joint",  "leg_l4_joint",  "leg_r4_joint",
    "arm_l2_joint", "arm_r2_joint",  "leg_l5_joint",  "leg_r5_joint",
    "arm_l3_joint", "arm_r3_joint",  "leg_l6_joint",  "leg_r6_joint",
    "arm_l4_joint", "arm_r4_joint",  "arm_l5_joint",  "arm_r5_joint",
};
std::vector<std::string> jointNames{
    "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
    "leg_r2_joint", "arm_l1_joint", "arm_r1_joint",  "leg_l3_joint",
    "leg_r3_joint", "arm_l2_joint", "arm_r2_joint",  "leg_l4_joint",
    "leg_r4_joint", "arm_l3_joint", "arm_r3_joint",  "leg_l5_joint",
    "leg_r5_joint", "arm_l4_joint", "arm_r4_joint",  "leg_l6_joint",
    "leg_r6_joint", "arm_l5_joint", "arm_r5_joint",  "waist_2_joint"};

std::string policyFilePath_;
std::string modelname;
int64_t count;
RobotCfg robotconfig;
std::shared_ptr<Ort::Env> onnxEnvPrt_;
std::unique_ptr<Ort::Session> policySessionPtr;
std::unique_ptr<Ort::Session> estSessionPtr;
std::vector<const char *> policyInputNames_;
std::vector<const char *> policyOutputNames_;
std::vector<const char *> estInputNames_;
std::vector<const char *> estOutputNames_;
std::vector<Ort::AllocatedStringPtr> policyInputNodeNameAllocatedStrings;
std::vector<Ort::AllocatedStringPtr> policyOutputNodeNameAllocatedStrings;
std::vector<Ort::AllocatedStringPtr> estInputNodeNameAllocatedStrings;
std::vector<Ort::AllocatedStringPtr> estOutputNodeNameAllocatedStrings;
std::vector<std::vector<int64_t>> policyInputShapes_;
std::vector<std::vector<int64_t>> policyOutputShapes_;
std::vector<std::vector<int64_t>> estInputShapes_;
std::vector<std::vector<int64_t>> estOutputShapes_;
vector3_t baseLinVel_;
vector3_t basePosition_;
vector_t lastActions_;
vector_t defaultJointAngles_;
vector_t walkdefaultJointAngles_;
int actuatedDofNum_;
bool *isfirstRecObs_;
int actionsSize_;
int motionSize;
int observationSize_;
int stackSize_;
float scalez;
float scalex;
float scaley;
std::vector<tensor_element_t> actions_;
std::vector<tensor_element_t> policyObservations_;
std::vector<tensor_element_t> estObservations_;

Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
bool isfirstCompAct_{true};
vector_t command_;
Proprioception propri_;
double phase_;
int64_t loopcount_;
NingImuData imu_data_;
joydata remote_data_;
joydata remote_data;
NingImuData imu_data;
std::array<MotorState, 24> motorstate_;
double standPercent;
scalar_t standDuration;
vector_t lieJointAngles;
JointState standjointState_{0.0,    0.0,     0.0,     0.0,     0.0,     0.0,
                            0.0,    -0.1495, -0.1495, 0.0,     0.0,     0.3215,
                            0.3215, 0.0000,  0.0000,  -0.1720, -0.1720, 0.0,
                            0.0,    0.0,     0.0,     0.0,     0.0,     0.0};

JointState liejointState_{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};
WorkMode mode_;
bool isChangeMode_ = false;
bool startcontrol = false;
int initfinish = 0;
int model_type;
long long statetimestamp;
std::mutex state_mutex;
std::array<MotorCmd, 24> usermotorcmd;

vector_t lieJointAngles_;
vector_t standJointAngles_;

std::thread process_thread_;
std::thread send_thread_;
int new_state_arrived = false;
bool found_joint_names{false};
bool found_default_joint_pos{false};
bool found_action_scale{false};
bool found_joint_stiffness{false};
bool found_joint_damping{false};
std::vector<double> action_scale;
std::vector<double> joint_stiffness;
std::vector<double> joint_damping;
std::vector<double> default_joint_pos;
std::vector<std::string> joint_names;
Ort::MemoryInfo memoryInfo =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

std::vector<scalar_t> currentJointAngles_;
bool init() {
        standDuration = 1000;
        standPercent = 0;
        lieJointAngles_.resize(24);
        standJointAngles_.resize(24);
        currentJointAngles_.resize(24);
        auto &LieState = liejointState_;
        auto &StandState = standjointState_;
        lieJointAngles_ << LieState.leg_l1_joint, LieState.leg_r1_joint,
            LieState.waist_1_joint, LieState.leg_l2_joint,
            LieState.leg_r2_joint, LieState.arm_l1_joint, LieState.arm_r1_joint,
            LieState.leg_l3_joint, LieState.leg_r3_joint, LieState.arm_l2_joint,
            LieState.arm_r2_joint, LieState.leg_l4_joint, LieState.leg_r4_joint,
            LieState.arm_l3_joint, LieState.arm_r3_joint, LieState.leg_l5_joint,
            LieState.leg_r5_joint, LieState.arm_l4_joint, LieState.arm_r4_joint,
            LieState.leg_l6_joint, LieState.leg_r6_joint, LieState.arm_l5_joint,
            LieState.arm_r5_joint, LieState.waist_2_joint;

        standJointAngles_ << StandState.leg_l1_joint, StandState.leg_r1_joint,
            StandState.waist_1_joint, StandState.leg_l2_joint,
            StandState.leg_r2_joint, StandState.arm_l1_joint,
            StandState.arm_r1_joint, StandState.leg_l3_joint,
            StandState.leg_r3_joint, StandState.arm_l2_joint,
            StandState.arm_r2_joint, StandState.leg_l4_joint,
            StandState.leg_r4_joint, StandState.arm_l3_joint,
            StandState.arm_r3_joint, StandState.leg_l5_joint,
            StandState.leg_r5_joint, StandState.arm_l4_joint,
            StandState.arm_r4_joint, StandState.leg_l6_joint,
            StandState.leg_r6_joint, StandState.arm_l5_joint,
            StandState.arm_r5_joint, StandState.waist_2_joint;
        return true;
}

void setparameter(Command &cmd, bool *isfirst) {
        isfirstRecObs_ = isfirst;
        isfirstCompAct_ = *isfirstRecObs_;
        command_[0] = cmd.x;
        command_[1] = cmd.y;
        command_[2] = cmd.yaw;
}

void computeActions() {
        std::vector<Ort::Value> policyInputValues;
        policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
            memoryInfo, policyObservations_.data(), policyObservations_.size(),
            policyInputShapes_[0].data(), policyInputShapes_[0].size()));
        // run inference
        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputValues = policySessionPtr->Run(
            runOptions, policyInputNames_.data(), policyInputValues.data(), 1,
            policyOutputNames_.data(), 1);
        if (isfirstCompAct_) {
                for (int i = 0; i < policyObservations_.size(); ++i) {
                        std::cout << policyObservations_[i] << " ";
                        if ((i + 1) % observationSize_ == 0) {
                                std::cout << std::endl;
                        }
                }
                isfirstCompAct_ = false;
        }

        for (int i = 0; i < actionsSize_; i++) {
                actions_[i] =
                    *(outputValues[0].GetTensorMutableData<tensor_element_t>() +
                      i);
        }
}

void computeObservation() {
        std::atomic<scalar_t> comm_x;
        std::atomic<scalar_t> comm_y;
        std::atomic<scalar_t> comm_z;

        vector_t command(3);
        comm_x = command_[0] * scalex;
        comm_y = command_[1] * scaley;
        comm_z = command_[2] * scalez;

        // 绝对值小于0.3的都置0
        if (abs(comm_x) < 0.3)
                comm_x = 0.0;
        if (abs(comm_y) < 0.3)
                comm_y = 0.0;
        if (abs(comm_z) < 0.3)
                comm_z = 0.0;
        if (comm_x < 0)
                comm_z = 0;
        // 如果y的command大于0.3，其余command为0
        if (abs(comm_y) > 0.3) {
                comm_x = 0.0;
                comm_z = 0.0;
        }
        if (abs(comm_x) > 0.3 && abs(comm_z) > 0.3) {
                comm_y = 0.0;
        }

        // x的command限定在-1.0-1.2
        if (comm_x < -1.0)
                comm_x = -1.0;
        // if (command_.x > 1.2) command_.x = 1.2;

        command[0] = comm_x;
        command[1] = comm_y;
        command[2] = comm_z;
        std::vector<int> jointIndices = {
            0, 1, 3, 4, 6, 7, 10, 11, 14, 15, 18, 19,
        };
        vector_t jointPosSel(jointIndices.size());
        vector_t jointVelSel(jointIndices.size());
        for (size_t i = 0; i < jointIndices.size(); i++) {
                int idx = jointIndices[i];
                jointPosSel[i] = propri_.jointPos[idx];
                jointVelSel[i] = propri_.jointVel[idx];
                // defaultAngleSel[i] = defaultJointAngles_[idx];
        }
        vector_t actions(lastActions_);
        vector_t proprioObs(observationSize_);

        proprioObs << command,                   // 3
            propri_.baseAngVel,                  // 3
            propri_.projectedGravity(0),         // 1
            propri_.projectedGravity(1),         // 1
            propri_.projectedGravity(2),         // 1
            (jointPosSel - defaultJointAngles_), // 12
            jointVelSel,                         // 12
            actions;                             // 12

        if (*isfirstRecObs_) {
                for (int i = observationSize_ - actionsSize_;
                     i < observationSize_; i++) {
                        proprioObs(i, 0) = 0.0;
                }

                for (size_t i = 0; i < stackSize_; i++) {
                        proprioHistoryBuffer_.segment(i * observationSize_,
                                                      observationSize_) =
                            proprioObs.cast<tensor_element_t>();
                }
                *isfirstRecObs_ = false;
                std::fill(policyObservations_.begin(),
                          policyObservations_.end(), 0.0f);
        }
        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() -
                                   observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() -
                                       observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) =
            proprioObs.cast<tensor_element_t>();

        //clang-format on

        for (size_t i = 0; i < (observationSize_ * stackSize_); i++) {
                policyObservations_[i] =
                    static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
        }
        // Limit observation range
        scalar_t obsMin = -robotconfig.clipObs;
        scalar_t obsMax = robotconfig.clipObs;
        std::transform(policyObservations_.begin(), policyObservations_.end(),
                       policyObservations_.begin(),
                       [obsMin, obsMax](scalar_t x) {
                               return std::max(obsMin, std::min(obsMax, x));
                       });
}

bool updateStateEstimation() {
        vector_t jointPosnoarm(24), jointVelnoarm(24), jointTornoarm(24);
        quaternion_t quat;
        vector3_t angularVel, linearAccel;
        static int num = 0;
        std::array<MotorState, 24> joint_state = ctrl->get_joint_state();

        for (size_t i = 0; i < actuatedDofNum_; ++i) {
                int index = ctrl->getJointsIndex(walkjointNames[i]);
                jointPosnoarm(i) = joint_state[index].pos;
                jointVelnoarm(i) = joint_state[index].vel;
                jointTornoarm(i) = joint_state[index].tau;
        }

        NingImuData imudata = ctrl->get_imu_data();
        for (size_t i = 0; i < 4; ++i) {
                quat.coeffs()(i) = imudata.ori[i];
        }
        for (size_t i = 0; i < 3; ++i) {
                angularVel(i) = imudata.angular_vel[i];
                linearAccel(i) = imudata.linear_acc[i];
        }

        propri_.jointPos = jointPosnoarm;
        propri_.jointVel = jointVelnoarm;
        propri_.baseAngVel = angularVel;

        vector3_t gravityVector(0, 0, -1);
        vector3_t zyx = quatToZyx(quat);
        matrix_t inverseRot =
            getRotationMatrixFromZyxEulerAngles(zyx).inverse();
        propri_.projectedGravity = inverseRot * gravityVector;
        propri_.baseEulerXyz = quatToXyz(quat);

        std::chrono::microseconds now =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch());
        double seconds = now.count();
        phase_ = seconds / 1000000.0;

        return true;
}

void handleDefautMode() {
        // MotorCmd  motorcmd;
        std::array<MotorCmd, 24> motorcmd;
        for (int j = 0; j < 24; j++) {
                motorcmd[j].kd = 0.1;
                motorcmd[j].pos = 0;
                motorcmd[j].kp = 0;
                motorcmd[j].motor_id = j;
                motorcmd[j].vel = 0;
                motorcmd[j].tau = 0;
                // controllerbase.set_joint(motorcmd);
        }

        ctrl->set_joint(motorcmd);
}

void handleStandMode() {
        std::array<MotorCmd, 24> motorcmd{};
        // init all motor_ids
        for (int j = 0; j < 24; j++) {
                motorcmd[j].motor_id = ctrl->getJointsIndex(jointNames[j]);
        }
        if (standPercent <= 1) {
                for (int j = 0; j < 24; j++) {
                        scalar_t pos_des =
                            currentJointAngles_[j] * (1 - standPercent) +
                            standJointAngles_[j] * standPercent;
                        int index = ctrl->getJointsIndex(jointNames[j]);
                        if (j == 23) {
                                motorcmd[index].kd = 5;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        } else if (j == 15 || j == 16 || j == 19 || j == 20) {
                                motorcmd[index].pos = pos_des;
                                motorcmd[index].kp = 50;
                                motorcmd[index].kd = 1;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        } else {
                                motorcmd[index].pos = pos_des;
                                motorcmd[index].kp = 50;
                                motorcmd[index].kd = 5;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        }
                }
                ctrl->set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, scalar_t(1));
        }
}

void handleLieMode() {
        std::array<MotorCmd, 24> motorcmd{};
        // init all motor_ids
        for (int j = 0; j < 24; j++) {
                motorcmd[j].motor_id = ctrl->getJointsIndex(jointNames[j]);
        }
        if (standPercent <= 1) {
                for (int j = 0; j < 24; j++) {
                        scalar_t pos_des =
                            currentJointAngles_[j] * (1 - standPercent) +
                            lieJointAngles_[j] * standPercent;
                        int index = ctrl->getJointsIndex(jointNames[j]);
                        if (j == 23) {
                                motorcmd[index].pos = pos_des;
                                motorcmd[index].kp = 200;
                                motorcmd[index].kd = 5;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        } else if (j == 15 || j == 16 || j == 19 || j == 20) {
                                motorcmd[index].pos = pos_des;
                                motorcmd[index].kp = 50;
                                motorcmd[index].kd = 1;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        } else {
                                motorcmd[index].pos = pos_des;
                                motorcmd[index].kp = 70;
                                motorcmd[index].kd = 5;
                                motorcmd[index].vel = 0;
                                motorcmd[index].tau = 0;
                        }
                }
                ctrl->set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, double(1));
        }
}

bool handleUserMode() {
        if (updateStateEstimation() == false)
                return false;
        if (count % robotconfig.controlCfg.decimation == 0) {
                count = 0;
                computeObservation();
                computeActions();

                // limit action range
                scalar_t actionMin = -robotconfig.clipActions;
                scalar_t actionMax = robotconfig.clipActions;
                std::transform(
                    actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x) {
                            return std::max(actionMin, std::min(actionMax, x));
                    });
        }
        // set action
        int j = 0;
        // MotorCmd motorcmd;
        std::vector<int> jointIndicesLeg = {
            0, 1, 3, 4, 6, 7, 10, 11, 14, 15, 18, 19,
        };
        std::vector<int> jointIndicesArm = {2,  5,  8,  9,  12, 13,
                                            16, 17, 20, 21, 22, 23};
        std::array<MotorCmd, 24> motorcmd;
        for (int i = 0; i < jointIndicesArm.size(); i++) {
                double kp = 150;
                double kd = 5;
                double pos = 0;
                int jointIdx = jointIndicesArm[i];
                if (i < 2) {
                        kp = 400;
                        kd = 5;
                } else if (jointIdx == 12) {
                        pos = 0.267;
                } else if (jointIdx == 13) {
                        pos = -0.267;
                }

                int index = ctrl->getJointsIndex(walkjointNames[jointIdx]);
                scalar_t pos_des;
                std::array<MotorState, 24> joint_state =
                    ctrl->get_joint_state();
                double cur_pos = joint_state[index].pos;
                if (abs(pos - cur_pos) > 0.5)
                        pos_des = 0.98 * cur_pos + 0.02 * pos;
                else
                        pos_des = 0.95 * cur_pos + 0.05 * pos;
                motorcmd[index].pos = pos_des;
                motorcmd[index].kp = kp;
                motorcmd[index].kd = kd;
                motorcmd[index].motor_id = index;
                motorcmd[index].vel = 0;
                motorcmd[index].tau = 0;
                // hw.set_joint(motorcmd);
        }
        for (int j = 0; j < 12; j++) {
                int jointIdx = jointIndicesLeg[j];

                int index = ctrl->getJointsIndex(walkjointNames[jointIdx]);
                scalar_t action_value = actions_[j] * action_scale[j];
                scalar_t pos_des = action_value + defaultJointAngles_(j);
                double stiffness = joint_stiffness[j]; // 根据关节名称获取刚度
                double damping = joint_damping[j];     // 根据关节名称获取阻尼
                // std::cout << "joint_name:" << partName << "kp:" << stiffness
                // << " kd:" << damping << std::endl;
                motorcmd[index].pos = pos_des;
                motorcmd[index].kp = stiffness;
                motorcmd[index].kd = damping;
                motorcmd[index].motor_id = index;
                motorcmd[index].vel = 0;
                motorcmd[index].tau = 0;
                // hw.set_joint(motorcmd);
                lastActions_(j, 0) = actions_[j];
        }
        ctrl->set_joint(motorcmd);
        count++;
        return true;
}

void process() {
        static int keyflag[14];
        if (initfinish == 0)
                return;
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

        remote_data = ctrl->from_dds_get_joydata();
        cmd.x = remote_data.axes[1];
        cmd.y = 0;
        cmd.yaw = remote_data.axes[0];

        if ((remote_data.button[9] == 1) && (keyflag[9] == 0)) {
                if (!startcontrol) {
                        startcontrol = true;
                        standPercent = 0;
                        mode_ = WorkMode::LIE;
                        keyflag[9] = 1;
                        std::array<MotorState, 24> joint_state =
                            ctrl->get_joint_state();
                        int index = 0;
                        for (size_t i = 0; i < actuatedDofNum_; i++) {
                                index = ctrl->getJointsIndex(jointNames[i]);
                                currentJointAngles_[i] = joint_state[index].pos;
                        }
                        printf("start control\n");

                } else {
                        startcontrol = false;
                        mode_ = WorkMode::DEFAULT;
                        keyflag[9] = 1;
                        printf("stop control\n");
                }

        } else if (remote_data.button[9] == 0)
                keyflag[9] = 0;
        if ((remote_data.button[10] == 1) && (remote_data.button[2] == 1) &&
            (keyflag[10] == 0)) {
                if (startcontrol == true) {
                        if (mode_ != WorkMode::STAND) {
                                standPercent = 0;
                                mode_ = WorkMode::STAND;
                                std::array<MotorState, 24> joint_state =
                                    ctrl->get_joint_state();
                                int index = 0;
                                for (size_t i = 0; i < actuatedDofNum_; i++) {
                                        index =
                                            ctrl->getJointsIndex(jointNames[i]);
                                        currentJointAngles_[i] =
                                            joint_state[index].pos;
                                }
                                printf("STAND2LIE\n");
                        } else if (mode_ == WorkMode::LIE) {
                                standPercent = 0;
                                mode_ = WorkMode::STAND;
                                printf("LIE2STAND\n");
                        }
                }
        } else if (remote_data.button[10] == 0)
                keyflag[10] = 0;
        if ((remote_data.button[5] == 1) && (remote_data.button[2] == 1) &&
            (keyflag[5] == 0)) {

                if (mode_ == WorkMode::STAND) {
                        standPercent = 0;
                        isChangeMode_ = true;
                        mode_ = WorkMode::USERMODE;
                        keyflag[5] = 1;
                        printf("TO USERWALK MODE\n");
                }

        } else if (remote_data.button[5] == 0)
                keyflag[5] = 0;
        if ((remote_data.button[11] == 1) && (keyflag[11] == 0)) {
                if (mode_ == WorkMode::USERMODE) {
                        isChangeMode_ = true;
                        mode_ = WorkMode::STAND;
                        printf("WALK2STAND\n");

                } else if (mode_ == WorkMode::DEFAULT) {
                        standPercent = 0;
                        printf("deftolie\n");
                        isChangeMode_ = true;
                        mode_ = WorkMode::LIE;
                }
        }

        switch (mode_) {
        case WorkMode::STAND:
                handleStandMode();
                break;
        case WorkMode::LIE:
                handleLieMode();
                break;
        case WorkMode::DEFAULT:
                handleDefautMode();
                break;
        case WorkMode::USERMODE:
                setparameter(cmd, &isChangeMode_);
                handleUserMode();
                break;
        default:
                printf("Unexpected mode encountered: %d\n",
                       static_cast<int>(mode_));
                break;
        }
}

void onnxdatainit() {
        Ort::AllocatorWithDefaultOptions allocator;
        // ROS_INFO_STREAM("count: " <<
        // poliycfg->policySessionPtr->GetOutputCount());
        for (int i = 0; i < policySessionPtr->GetInputCount(); i++) {
                auto policyInputnamePtr =
                    policySessionPtr->GetInputNameAllocated(i, allocator);
                policyInputNodeNameAllocatedStrings.push_back(
                    std::move(policyInputnamePtr));
                policyInputNames_.push_back(
                    policyInputNodeNameAllocatedStrings.back().get());
                // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i,
                // allocator).get());
                policyInputShapes_.push_back(
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }

        for (int i = 0; i < policySessionPtr->GetOutputCount(); i++) {
                auto policyOutputnamePtr =
                    policySessionPtr->GetOutputNameAllocated(i, allocator);
                policyOutputNodeNameAllocatedStrings.push_back(
                    std::move(policyOutputnamePtr));
                policyOutputNames_.push_back(
                    policyOutputNodeNameAllocatedStrings.back().get());
                // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i,
                // allocator).get());
                std::cout << policySessionPtr
                                 ->GetOutputNameAllocated(i, allocator)
                                 .get()
                          << std::endl;
                policyOutputShapes_.push_back(
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }
}

bool getmodelparam() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string conpath = std::string(buf);
        std::string path = modelname;
        // RobotCfg::InitState &initState = robotconfig.initState;
        RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
        RobotCfg::ObsScales &obsScales = robotconfig.obsScales;

        YAML::Node acconfig = YAML::LoadFile(conpath + "/config/e1_ac.yaml");

        int error = 0;

        action_scale =
            acconfig[modelname]["action_scale"].as<std::vector<double>>();
        default_joint_pos =
            acconfig[modelname]["default_joint_pos"].as<std::vector<double>>();
        joint_damping =
            acconfig[modelname]["joint_damping"].as<std::vector<double>>();
        joint_stiffness =
            acconfig[modelname]["joint_stiffness"].as<std::vector<double>>();
        joint_names =
            acconfig[modelname]["joint_names"].as<std::vector<std::string>>();
        standDuration = 1000;
        standPercent = 0;
        // controlCfg.actionScale =
        // acconfig[modelname]["control"]["action_scale"].as<float>();
        controlCfg.decimation =
            acconfig[modelname]["control"]["decimation"].as<int>();
        controlCfg.cycle_time =
            acconfig[modelname]["control"]["cycle_time"].as<float>();

        robotconfig.clipObs = acconfig[modelname]["normalization"]
                                      ["clip_scales"]["clip_observations"]
                                          .as<double>();
        robotconfig.clipActions =
            acconfig[modelname]["normalization"]["clip_scales"]["clip_actions"]
                .as<double>();

        actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
        observationSize_ =
            acconfig[modelname]["size"]["observations_size"].as<int>();

        stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

        scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
        scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
        scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();

        actions_.resize(actionsSize_);

        actuatedDofNum_ = 24;

        policyObservations_.resize(observationSize_ * stackSize_);

        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        lastActions_.resize(actionsSize_);
        lastActions_.setZero();
        const int inputSize = stackSize_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        defaultJointAngles_.resize(actionsSize_);
        walkdefaultJointAngles_.resize(actionsSize_);
        for (int i = 0; i < actionsSize_; i++) {
                defaultJointAngles_(i) = default_joint_pos[i];
                // printf("defaultJointAngles[%d]
                // %f\n",i,modelcfg->defaultJointAngles_(i));
        }

        return true;
}

bool loadModel(std::string modelpath) {
        std::string estFilePath;
        // create session
        Ort::SessionOptions sessionOptions;
        bool ret;
        onnxEnvPrt_.reset(
            new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        sessionOptions.SetInterOpNumThreads(1);
        if (onnxEnvPrt_ == NULL) {
                printf("onnxEnvPrt_  is null\n");
                return false;
        }

        printf("Load Onnx model from path : %s\n", modelpath.c_str());

        policySessionPtr = std::make_unique<Ort::Session>(
            *onnxEnvPrt_, modelpath.c_str(), sessionOptions);
        if (policySessionPtr == NULL) {
                printf("load run model failed\n");
                return false;
        }

        // get input and output info
        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        estSessionPtr = NULL;
        modelname = "walk";
        command_.resize(3);
        isfirstCompAct_ = true;
        isfirstRecObs_ = NULL;
        count = 0;
        model_type = 0;
        onnxdatainit();
        ret = getmodelparam();
        initfinish = 1;

        printf("Load Onnx run model successfully !!!\n");
        return true;
}

int main() {
        // setenv("CYCLONEDDS_URI","file:///home/oem/test/dds-test/config/dds.xml",1);
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        ctrl = legged::LowController::Instance();
        ctrl->init();
        init();
        loadModel(path + "/policy/policy_walk.onnx");
        while (1) {
                auto start_time = std::chrono::steady_clock::now();
                process();
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time);
                std::this_thread::sleep_for(std::chrono::microseconds(2000) -
                                            elapsed);
        }
        return 0;
}
