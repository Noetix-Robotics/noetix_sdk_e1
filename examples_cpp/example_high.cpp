#include "highcontroller.h"

using namespace legged;

int main(int argc, char *argv[]) {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        HighController *ctrl = HighController::Instance();
        ctrl->init();

        joydata remote_data;

        int key_updown[14], key_inuse[14];
        float x, yaw;
        int fileindex;
        ControlCmd action;
        Command cmd;

        while (1) {
                remote_data = ctrl->from_dds_get_joydata();
                for (int i = 0; i < 14; i++) {
                        if (remote_data.button[i] == 0) {
                                key_updown[i] = 0;
                                key_inuse[i] = 0;
                        } else if (remote_data.button[i] == 1) {
                                key_updown[i] = 1;
                        }
                }
                action = ControlCmd::DEFAULT;
                cmd.x = remote_data.axes[1];
                cmd.y = 0;
                cmd.yaw = remote_data.axes[0];
                if (key_updown[Key2] == 0 && key_updown[Key5] == 1 &&
                    (key_inuse[Key5] == 0)) {
                        action = ControlCmd::STARTTEACH;
                        key_inuse[Key5] = 1;
                        printf("STARTTEACH \n");
                } else if ((key_updown[Key6] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key6] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::SWING;
                        key_inuse[Key6] = 1;
                        printf("SWING \r\n");
                } else if ((key_updown[Key7] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key7] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::SHAKE;
                        key_inuse[Key7] = 1;
                        printf("SHAKE \r\n");
                } else if ((key_updown[Key8] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key8] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::CHEER;
                        key_inuse[Key8] = 1;
                        printf("CHEER \r\n");
                } else if ((key_updown[Key9] == 1) && (key_inuse[Key9] == 0)) {
                        key_inuse[Key9] = 1;
                        action = ControlCmd::START;
                        printf("START \r\n");
                } else if (key_updown[Key10] == 1 && (key_inuse[Key10] == 0)) {
                        action = ControlCmd::SWITCH;
                        key_inuse[Key10] = 1;
                        printf("SWITCH \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key5] == 1 &&
                           (key_inuse[Key5] == 0)) {
                        action = ControlCmd::WALK;
                        key_inuse[Key5] = 1;
                        printf("WALK \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key6] == 1 &&
                           (key_inuse[Key6] == 0)) {
                        action = ControlCmd::SAVETEACH;
                        key_inuse[Key6] = 1;
                        fileindex++;
                        printf("SAVETEACH \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key7] == 1 &&
                           (key_inuse[Key7] == 0)) {
                        action = ControlCmd::ENDTEACH;
                        key_inuse[Key7] = 1;
                        printf("ENDTEACH \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key8] == 1 &&
                           (key_inuse[Key8] == 0)) {
                        action = ControlCmd::PLAYTEACH;
                        key_inuse[Key8] = 1;
                        fileindex = 1;
                        printf("PLAYTEACH \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key6] == 1) {
                        action = ControlCmd::RUN;
                        printf("RUN \r\n");
                        key_inuse[Key6] = 1;
                }

                ctrl->publish_cmd(cmd.x, cmd.yaw, action, fileindex);
                sleep_ms(2);
        }
        return 0;
}
