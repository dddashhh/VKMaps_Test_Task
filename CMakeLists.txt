cmake_minimum_required(VERSION 3.15)
project(VKMaps)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)  # Автоматический запуск MOC
set(CMAKE_AUTORCC ON)  # Автоматическая обработка qrc-файлов
set(CMAKE_AUTOUIC ON)  # Автоматическая обработка ui-файлов

# Поиск пакетов Qt
find_package(Qt6 COMPONENTS Core Gui Widgets REQUIRED)
if (NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Core Gui Widgets REQUIRED)
endif()

# Исходные файлы проекта
set(SOURCE_FILES
        main.cpp
)

# Создание исполняемого файла
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Линковка с библиотеками Qt
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

# Для macOS: создание bundle
if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE TRUE
    )
endif()