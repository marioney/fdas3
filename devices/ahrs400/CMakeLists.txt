add_custom_command(
  OUTPUT generated/ahrs400_messages/mavlink.h
  COMMAND mavgen.py --lang=C --output=generated
            --wire-protocol 1.0
            ${CMAKE_CURRENT_SOURCE_DIR}/ahrs400_messages.xml
  MAIN_DEPENDENCY ahrs400_messages.xml)
add_custom_target(ahrs400-mavgen DEPENDS generated/ahrs400_messages/mavlink.h)

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_executable(ahrs400-read ahrs400-read.c ahrs400.c)
add_dependencies(ahrs400-read ahrs400-mavgen)
