###############################################################
# This file can be used as-is in the directory of any example,#
# however you might need to specify your own dependencies in  #
# target_link_libraries if your app depends on more than dart #
###############################################################

add_executable(gripper Main.cpp MyWindow.cpp MyWindow.hpp gripper.hpp gripper.cpp ${FCL2_COLLISION_SRC})
dart_add_example(gripper)
target_link_libraries(gripper dart dart-gui dart-utils-urdf)
set_target_properties(gripper PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
