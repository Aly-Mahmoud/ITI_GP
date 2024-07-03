#include <stdio.h>
#include <stdlib.h>
#include "dds/dds.h"
#include "TopicDiscovery.h" // Include the generated header file

int main(int argc, char **argv) {
    dds_entity_t participant;
    dds_entity_t topic;
    dds_entity_t reader;
    dds_return_t rc;
    TopicDiscovery_Data *msg;
    void *samples[1];
    dds_sample_info_t infos[1];
    
    (void)argc;
    (void)argv;

    // Create a Participant
    participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0) {
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    }

    // Create a Topic
    topic = dds_create_topic(participant, &TopicDiscovery_Data_desc, "TopicDiscovery_Data", NULL, NULL);
    if (topic < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
    }

    // Create a Reader
    reader = dds_create_reader(participant, topic, NULL, NULL);
    if (reader < 0) {
        DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
    }

    printf("=== [Subscriber] Waiting for data...\n");

    // Initialize data holder
    samples[0] = TopicDiscovery_Data__alloc();

    while (1) {
        // Wait for data
        rc = dds_read(reader, samples, infos, 1, 1);
        if (rc < 0) {
            DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));
        }

        if ((rc > 0) && (infos[0].valid_data)) {
            msg = (TopicDiscovery_Data*)samples[0];
            printf("Received data: Steering=%d, Throttle=%d, Brake=%d, Buttons=%d,%d,%d,%d,%d,%d,%d,%d\n",
                   msg->steering, msg->throttle, msg->brake,
                   msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3],
                   msg->buttons[4], msg->buttons[5], msg->buttons[6], msg->buttons[7]);
            fflush(stdout);
        }
        usleep(100000); // Sleep for 100 milliseconds to prevent busy-waiting
    }

    // Cleanup
    dds_return_t ret = dds_delete(participant);
    if (ret != DDS_RETCODE_OK) {
        DDS_FATAL("dds_delete: %s\n", dds_strretcode(-ret));
    }

    return EXIT_SUCCESS;
}

