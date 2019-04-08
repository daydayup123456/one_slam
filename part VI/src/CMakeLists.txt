

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
# CSparse
find_package( CSparse REQUIRED)
include_directories( ${CSPARSE_INCLUDE_DIR} )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )

ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )




ADD_EXECUTABLE( visualOdometry visualOdometry.cpp)
TARGET_LINK_LIBRARIES( visualOdometry
    slambase
    ${OpenCV_LIBS} 
    ${Sophus_LIBRARIES}
    ${PCL_LIBRARIES} 
     g2o_core g2o_stuff g2o_types_sba  
    g2o_types_slam3d 
    g2o_solver_csparse g2o_csparse_extension
    cholmod g2o_solver_cholmod)
