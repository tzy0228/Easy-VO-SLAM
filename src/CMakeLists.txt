add_library(MYSLAM STATIC
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(MYSLAM
        ${THIRD_PARTY_LIBS} fmt )
