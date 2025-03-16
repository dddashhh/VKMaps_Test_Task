cmake_minimum_required(VERSION 3.15)
project(VKMaps)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Core Gui Widgets REQUIRED)
if (NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Core Gui Widgets REQUIRED)
endif()

set(SOURCE_FILES
        main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

if (Qt6_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE
            Qt6::Core
            Qt6::Gui
            Qt6::Widgets
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
            Qt5::Core
            Qt5::Gui
            Qt5::Widgets
    )
endif()

if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE TRUE
    )
endif()