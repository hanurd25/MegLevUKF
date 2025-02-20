set(publicHeaders
        ${PROJECT_SOURCE_DIR}/include
)


set(SOURCES
        maglevSystemDynamics.cpp
        integration.cpp
        maglevObject.cpp
        maglevSystemDynamics.cpp
        MagLevUKF.cpp
        serialReader.cpp
)

add_library(dynamicsLib ${SOURCES})
target_include_directories(dynamicsLib PUBLIC ${publicHeaders} ${RAPIDCSV_INCLUDE_DIRS})
target_link_libraries(dynamicsLib PUBLIC Eigen3::Eigen)