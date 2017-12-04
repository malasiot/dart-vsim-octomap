###############################################################
# This file can be used as-is in the directory of any example,#
# however you might need to specify your own dependencies in  #
# target_link_libraries if your app depends on more than dart #
###############################################################

add_executable(rigid Main.cpp MyWindow.cpp MyWindow.hpp)
dart_add_example(rigid)
target_link_libraries(rigid dart dart-gui)
set_target_properties(rigid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
