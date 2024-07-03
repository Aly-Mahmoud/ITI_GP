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

