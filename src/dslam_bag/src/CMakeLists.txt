##############################################################################
# Library
##############################################################################

#add_subdirectory(lib)

##############################################################################
# Ecto Modules
##############################################################################

# first argument : module name (becomes library name)
# second argument : where to install it (i.e. folder under lib/python2.7/dist-packages
ectomodule(dslam_bag_vision_utils DESTINATION ${PROJECT_NAME} INSTALL
    dslam_bag_vision_utils/module.cpp
    dslam_bag_vision_utils/stamp.cpp
)

target_link_libraries(dslam_bag_vision_utils_ectomodule ${catkin_LIBRARIES})
