#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include "dds/dds.h"
#include "TopicDiscovery.h" // Include the generated header file

#define DEVICE_PATH "/dev/input/event6" // Replace with the correct event device path

typedef struct {
    int steering;
    int throttle;
    int brake;
    int buttons[8];
} ControllerData;

dds_entity_t participant;
dds_entity_t topic;
dds_entity_t writer;
dds_return_t rc;
TopicDiscovery_Data msg; // Use the generated type

void publish_data(ControllerData *data) {
    msg.steering = data->steering;
    msg.throttle = data->throttle;
    msg.brake = data->brake;
    for (int i = 0; i < 8; i++) {
        msg.buttons[i] = data->buttons[i];
    }

    printf("=== [Publisher] Publishing Data: Steering=%d Throttle=%d Brake=%d Buttons=%d,%d,%d,%d,%d,%d,%d,%d\n",
           msg.steering, msg.throttle, msg.brake,
           msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3],
           msg.buttons[4], msg.buttons[5], msg.buttons[6], msg.buttons[7]);

    rc = dds_write(writer, &msg);
    if (rc != DDS_RETCODE_OK) {
        DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));
    }
}

void setup_dds() {
    uint32_t status = 0;
    bool subscribers_found = false;

    participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0) {
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    }

    topic = dds_create_topic(participant, &TopicDiscovery_Data_desc, "TopicDiscovery_Data", NULL, NULL);
    if (topic < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
    }

    writer = dds_create_writer(participant, topic, NULL, NULL);
    if (writer < 0) {
        DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));
    }

    printf("=== [Publisher] Waiting for a subscriber to request subscription...\n");
    fflush(stdout);

    while (!subscribers_found) {
        rc = dds_get_status_changes(writer, &status);
        if (rc != DDS_RETCODE_OK) {
            DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));
        }

        if (status & DDS_PUBLICATION_MATCHED_STATUS) {
            printf("=== [Publisher] Subscriber discovered. Starting to publish data.\n");
            subscribers_found = true;
        } else {
            usleep(100000); // Sleep for 100 milliseconds
        }
    }
}

int main() {
    int fd;
    struct input_event ev;
    ControllerData data = {128, 0, 0, {0}};
    ControllerData prev_data = data;

    setup_dds();

    fd = open(DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        perror("Opening device");
        return EXIT_FAILURE;
    }

    while (1) {
        read(fd, &ev, sizeof(struct input_event));
        
        if (ev.type == EV_ABS) {
            switch (ev.code) {
                case ABS_X:
                    data.steering = ev.value * 255 / 65535;
                    break;
                case ABS_Z:
                    data.throttle = 255 - ev.value;
                    break;
                case ABS_RZ:
                    data.brake = 255 - ev.value;
                    break;
            }
        } else if (ev.type == EV_KEY) {
            switch (ev.code) {
                case 292: data.buttons[0] = ev.value; break;
                case 293: data.buttons[1] = ev.value; break;
                case 288: data.buttons[2] = ev.value; break;
                case 290: data.buttons[3] = ev.value; break;
                case 291: data.buttons[4] = ev.value; break;
                case 289: data.buttons[5] = ev.value; break;
                case 707: data.buttons[6] = ev.value; break;
                case 708: data.buttons[7] = ev.value; break;
            }
        }

        if (memcmp(&data, &prev_data, sizeof(ControllerData)) != 0) {
            publish_data(&data);
            prev_data = data;
        }
        usleep(10000); // Minimize delay to ensure continuous data stream
    }

    close(fd);

    rc = dds_delete(participant);
    if (rc != DDS_RETCODE_OK) {
        DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));
    }

    return EXIT_SUCCESS;
}

