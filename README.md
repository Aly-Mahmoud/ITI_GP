# README: Communication Between Host Machine (Ubuntu x86) and Comecu (Raspberry Pi x64)

## Overview

This project demonstrates a basic implementation of a Distributed Data Service (DDS) based communication system using CycloneDDS between two nodes: a host machine running Ubuntu x86 and a Raspberry Pi (Comecu) running a 64-bit OS. The communication involves a publisher on the host machine that reads data from a Logitech steering wheel set and publishes it to a DDS topic. The subscriber, running on the Raspberry Pi, receives this data and prints it to the terminal and then pass it to the second ECU in system through specific communication approach.

## Table of Contents

- [Project Structure](#project-structure)
- [Setting Up the Environment](#setting-up-the-environment)
- [Configuring DDS](#configuring-dds)
- [Running the Publisher](#running-the-publisher)
- [Running the Subscriber](#running-the-subscriber)
- [Explanation of the Code](#explanation-of-the-code)
- [Importance of DDS](#importance-of-dds)

## Project Structure

```bash
.
├── CMakeLists.txt
├── TopicDiscovery.idl
├── publisher.c
├── subscriber.c
├── build/
└── README.md
```



## Setting Up the Environment

### Prerequisites

- Ubuntu 18.04+ on the host machine
- Raspberry Pi 3/4 with a 64-bit OS
- CycloneDDS installed on both machines
- Development tools and libraries for CycloneDDS

### Installation

1. **Install CycloneDDS:**

   On both the host machine and the Raspberry Pi:

   ```bash
   sudo apt update
   sudo apt install cmake build-essential
   git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
   cd cyclonedds
   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX=<install-location> -DBUILD_EXAMPLES=ON ..
   cmake --build . --parallel
   cmake --build . --target install
   ```

   

2. **Set Up Steering Wheel Input Device:**

   Ensure the steering wheel is connected and determine its event device path:

   ```bash
   ls /dev/input/
   ```

   Update the `DEVICE_PATH` in `publisher.c` to the correct event device path.

   

3. **Generate DDS Types and Build Project:**

   Create a `CMakeLists.txt` file in the project root directory with the following content:

   ```cmake
   cmake_minimum_required(VERSION 3.16)
   project(pub_sub LANGUAGES C)
   
   if(NOT TARGET CycloneDDS::ddsc)
     # Find the CycloneDDS package.
     find_package(CycloneDDS REQUIRED)
   endif()
   
   # Generate the source files from IDL
   idlc_generate(TARGET TopicDiscovery_lib FILES "TopicDiscovery.idl" WARNINGS no-implicit-extensibility)
   
   # Publisher executable
   add_executable(publisher publisher.c)
   # Subscriber executable
   add_executable(subscriber subscriber.c)
   
   # Both executables need to be linked to the idl data type library and
   # the ddsc API library.
   target_link_libraries(publisher TopicDiscovery_lib CycloneDDS::ddsc)
   target_link_libraries(subscriber TopicDiscovery_lib CycloneDDS::ddsc)
   ```

   Then, build the project:

   ```bash
   mkdir build
   cd build
   cmake -DCMAKE_PREFIX_PATH=<install-location> ..
   cmake --build .
   ```

## Configuring DDS

Ensure that both the host machine and the Raspberry Pi are on the same Local Area Network. Enable multicast and configure firewall rules to allow multicast packets.

On Ubuntu, disable the firewall to avoid blocking multicast packets:

```bash
sudo ufw disable
```

On the Raspberry Pi, you can similarly disable or configure the firewall.

## Running the Publisher

1. **Run the Publisher:**

   On the host machine:

   ```bash
   sudo ./build/publisher
   ```

## Running the Subscriber

1. **Run the Subscriber:**

   On the Raspberry Pi:

   ```bash
   ./build/subscriber
   ```

## Explanation of the Code

### TopicDiscovery.idl

The IDL file defines the data structure for the DDS topic:

```idl
module TopicDiscovery {
    struct Data {
        long steering;
        long throttle;
        long brake;
        long buttons[8];
    };
};
```

### publisher.c

This file contains the implementation of the publisher:

- Reads data from the steering wheel set.
- Publishes the data to the DDS topic `ControllerData`.

### subscriber.c

This file contains the implementation of the subscriber:

- Subscribes to the DDS topic `ControllerData`.
- Receives and prints the steering wheel data.

## Importance of DDS

DDS is a middleware protocol and API standard for data-centric connectivity from the Object Management Group (OMG). It enables scalable, real-time, reliable, and high-performance communication between nodes in a distributed system.

### Benefits:

- **Decoupling:** DDS decouples publishers and subscribers in time, space, and synchronization.
- **Scalability:** Designed for scalable, distributed systems.
- **Real-time:** Supports real-time data exchange, which is crucial for applications like autonomous vehicles and industrial automation.
- **Quality of Service (QoS):** Provides extensive QoS policies to fine-tune the communication behavior according to application requirements.

In this application, DDS ensures that the steering wheel data is reliably transmitted from the publisher to the subscriber with minimal latency, making it suitable for real-time control applications.

## Conclusion

By following this guide, you have set up a basic DDS communication system between an Ubuntu host machine and a Raspberry Pi. This setup can be expanded and adapted for more complex systems, demonstrating the power and flexibility of DDS in distributed applications.
