add_library( myslam SHARED
			base.cpp
			compute.cpp
			readfile.cpp
			pointcloud.cpp
			frame.cpp
			loop.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
