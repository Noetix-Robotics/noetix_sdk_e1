#include "lowcontroller.h"

namespace legged {
// Controllerbase controllerbase;
DataBuffer<std::array<MotorCmd, 24>> motor_cmd_buffer_;
DataBuffer<std::array<MotorState, 24>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

bool LowController::init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
	printf("SDK_VERSION: %s \r\n", SDK_VERSION);
        printf("cur path is %s\n", path.c_str());

        RobotSetMode::SetMode cmode;

        cmode.mode(2);

        ddswrapper.publishModeData(cmode);
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 24> data;
                    joydata remote_data;
                    NingImuData imudata;
                    int i = 0;
                    for (const auto &state :
                         ddsdata.motorstatearray().motorstates()) {
                            data[i].pos = state.pos();
                            data[i].vel = state.vel();
                            data[i].tau = state.tau();
                            data[i].motor_id = state.motor_id();
                            data[i].error = state.error();
                            data[i].temperature = state.temperature();
                            i++;
                    }
                    for (int i = 0; i < 4; i++) {
                            imudata.ori[i] = ddsdata.imudata().ori()[i];
                    }
                    for (int i = 0; i < 3; i++) {
                            imudata.angular_vel[i] =
                                ddsdata.imudata().angular_vel()[i];
                            imudata.linear_acc[i] =
                                ddsdata.imudata().linear_acc()[i];
                    }
                    for (int i = 0; i < 9; i++) {
                            imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
                            imudata.angular_vel_cov[i] =
                                ddsdata.imudata().angular_vel_cov()[i];
                            imudata.linear_acc_cov[i] =
                                ddsdata.imudata().linear_acc_cov()[i];
                    }
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));

                    LowController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });
        send_thread_ = std::thread(&LowController::send_thread_func, this);
        sched_param ddssched{.sched_priority = 98};
        if (pthread_setschedparam(send_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        return true;
}

void LowController::set_robotstatusdata(std::array<MotorState, 24> data,
                                        NingImuData imudata, joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void LowController::send_thread_func() {
        while (1) {
                const std::shared_ptr<const std::array<MotorCmd, 24>> mc =
                    motor_cmd_buffer_.GetData();
                if (mc) {
                        RobotMotorCmd::MotorCmdArray cmdarray;
                        cmdarray.motorcmds().resize(24);
                        for (int i = 0; i < 24; i++) {
                                auto &cmd = cmdarray.motorcmds()[i];
                                cmd.pos() = mc->at(i).pos;
                                cmd.vel() = mc->at(i).vel;
                                cmd.tau() = mc->at(i).tau;
                                cmd.kp() = mc->at(i).kp;
                                cmd.kd() = mc->at(i).kd;
                                cmd.motor_id() = mc->at(i).motor_id;
                        }

                        auto now = Clock::now();
                        long long timestamp = std::chrono::duration_cast<
                                                  std::chrono::microseconds>(
                                                  now.time_since_epoch())
                                                  .count();
                        cmdarray.timestamp() = timestamp;
                        ddswrapper.publishMotorCmdData(cmdarray);
                }
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
}

void LowController::set_joint(std::array<MotorCmd, 24> motorcmd) {
        motor_cmd_buffer_.SetData(motorcmd);
}

const std::array<MotorState, 24> LowController::get_joint_state() {
        std::array<MotorState, 24> motorstate;
        const std::shared_ptr<const std::array<MotorState, 24>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 21; i++) {
                        motorstate[i].pos = ms->at(i).pos;
                        motorstate[i].vel = ms->at(i).vel;
                        motorstate[i].tau = ms->at(i).tau;
                        motorstate[i].motor_id = ms->at(i).motor_id;
                        motorstate[i].error = ms->at(i).error;
                        motorstate[i].temperature = ms->at(i).temperature;
                }
        }
        return motorstate;
}

NingImuData LowController::get_imu_data() {
        NingImuData imudata;
        const std::shared_ptr<const NingImuData> idata = imu_buffer_.GetData();
        if (idata) {
                for (int i = 0; i < 4; i++) {
                        imudata.ori[i] = (*idata).ori[i];
                }
                for (int i = 0; i < 3; i++) {
                        imudata.angular_vel[i] = (*idata).angular_vel[i];
                        imudata.linear_acc[i] = (*idata).linear_acc[i];
                }
                for (int i = 0; i < 9; i++) {
                        imudata.ori_cov[i] = (*idata).ori_cov[i];
                        imudata.angular_vel_cov[i] =
                            (*idata).angular_vel_cov[i];
                        imudata.linear_acc_cov[i] = (*idata).linear_acc_cov[i];
                }
        }
        return imudata;
}

joydata LowController::from_dds_get_joydata() {
        joydata jsdata;
        const std::shared_ptr<const joydata> jdata = joy_buffer_.GetData();
        if (jdata) {
                memcpy(jsdata.button, &(*jdata).button[0],
                       sizeof(jsdata.button));
                memcpy(jsdata.axes, &(*jdata).axes[0], sizeof(jsdata.axes));
        }
        return jsdata;
}

int LowController::getJointsIndex(std::string jointname) {
        int index = 0;
        if (jointname == "arm_l1_joint") {
                index = 0;
        } else if (jointname == "arm_l2_joint") {
                index = 1;
        } else if (jointname == "arm_l3_joint") {
                index = 2;
        } else if (jointname == "arm_l4_joint") {
                index = 3;
        } else if (jointname == "arm_l5_joint") {
                index = 4;
        } else if (jointname == "leg_l1_joint") {
                index = 5;
        } else if (jointname == "leg_l2_joint") {
                index = 6;
        } else if (jointname == "leg_l3_joint") {
                index = 7;
        } else if (jointname == "leg_l4_joint") {
                index = 8;
        } else if (jointname == "leg_l5_joint") {
                index = 9;
        } else if (jointname == "leg_l6_joint") {
                index = 10;
        } else if (jointname == "arm_r1_joint") {
                index = 11;
        } else if (jointname == "arm_r2_joint") {
                index = 12;
        } else if (jointname == "arm_r3_joint") {
                index = 13;
        } else if (jointname == "arm_r4_joint") {
                index = 14;
        } else if (jointname == "arm_r5_joint") {
                index = 15;
        } else if (jointname == "leg_r1_joint") {
                index = 16;
        } else if (jointname == "leg_r2_joint") {
                index = 17;
        } else if (jointname == "leg_r3_joint") {
                index = 18;
        } else if (jointname == "leg_r4_joint") {
                index = 19;
        } else if (jointname == "leg_r5_joint") {
                index = 20;
        } else if (jointname == "leg_r6_joint") {
                index = 21;
        } else if (jointname == "waist_1_joint") {
                index = 22;
        } else if (jointname == "waist_2_joint") {
                index = 23;
        }
        return index;
}

} // namespace legged
