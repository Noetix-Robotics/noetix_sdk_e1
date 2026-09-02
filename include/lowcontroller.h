#ifndef LowController_H
#define LowController_H

#include "common.h"

namespace noetix {

enum class WorkMode : uint8_t { STAND, LIE, USERMODE, DEFAULT };

class DDSWrapper;

class LowController {

      public:
        ~LowController();

        static LowController *Instance();

        bool init();

        void set_joint(std::array<MotorCmd, 24> motorcmd);

        int getJointsIndex(const std::string jointname);

        void
        subscribe_robot_hardware_status(RobotHardwareStatusCallback callback);

        void subscribe_robot_hand_status(RobotHandStatusCallback callback);

	void publish_hand_data(const std::vector<uint8_t> &data);

      protected:
        void send_thread_func();

      private:
        std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
