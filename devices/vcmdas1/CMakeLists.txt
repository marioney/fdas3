add_custom_command(
  OUTPUT generated/vcmdas1_messages/mavlink.h
  COMMAND mavgen.py --lang=C --output=generated
            --wire-protocol 1.0
            ${CMAKE_CURRENT_SOURCE_DIR}/vcmdas1_messages.xml
  MAIN_DEPENDENCY vcmdas1_messages.xml)
add_custom_target(vcmdas1-mavgen DEPENDS generated/vcmdas1_messages/mavlink.h)

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_executable(vcmdas1-read vcmdas1-read.c)
add_dependencies(vcmdas1-read vcmdas1-mavgen)
target_link_libraries(vcmdas1-read rt)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  install(CODE "execute_process(COMMAND \
      \"setcap\" \"cap_sys_rawio=ep\" \
      \"${CMAKE_CURRENT_BINARY_DIR}/vcmdas1-read\")")
endif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
install(TARGETS vcmdas1-read DESTINATION bin)

