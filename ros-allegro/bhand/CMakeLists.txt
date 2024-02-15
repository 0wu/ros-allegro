cmake_minimum_required(VERSION 3.8)
project(bhand)

find_package(ament_cmake REQUIRED)


###########
## Build ##
###########

include_directories(
  include
)

# NOTE: You can switch to 32 bit if necessary by updating the symbolic link.
find_library(BHAND_LIB
    NAMES BHand
    PATHS lib
)

# Make a tricky BHand target that just copies the .so binary file into the
# output directory.
add_library(BHand SHARED ${BHAND_LIB})
add_custom_command(
  TARGET BHand
  POST_BUILD
  DEPENDS ${BHAND_LIB}
  COMMAND cp ${BHAND_LIB} $<TARGET_SONAME_FILE:BHand>
  COMMAND touch $<TARGET_SONAME_FILE:BHand>)
set_target_properties(BHand PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

## Mark header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS BHand
  EXPORT bhand_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(bhand_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
