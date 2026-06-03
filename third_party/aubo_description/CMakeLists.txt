cmake_minimum_required(VERSION 2.8.3)
project(aubo_description)

if (DEFINED ENV{ROS_DISTRO})
    find_package(ros_environment REQUIRED)
    set(ROS_VERSION $ENV{ROS_VERSION})
    message("ROS_VERSION" ${ROS_VERSION})
    if(${ROS_VERSION} EQUAL 1)
        
    	execute_process(COMMAND bash ros1_xacro.sh
    	                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    	find_package(catkin REQUIRED)

    	catkin_package()

    	install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    	install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    	install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    	install(PROGRAMS
    	  scripts/calibrate_urdf_dh.py
    	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    	)
    endif()
    if(${ROS_VERSION} EQUAL 2)
    	cmake_minimum_required(VERSION 3.8)
    	if(NOT CMAKE_CXX_STANDARD)
    	  set(CMAKE_CXX_STANDARD_REQUIRED ON)
    	  set(CMAKE_CXX_STANDARD 14)
    	endif()
    	execute_process(COMMAND bash xacro.sh
    	                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
          add_compile_options(-Wall -Wextra -Wpedantic)
        endif()
        find_package(ament_cmake REQUIRED)
        if(BUILD_TESTING)
          find_package(ament_lint_auto REQUIRED)
          # the following line skips the linter which checks for copyrights
          # comment the line when a copyright and license is added to all source files
          set(ament_cmake_copyright_FOUND TRUE)
          # the following line skips cpplint (only works in a git repo)
          # comment the line when this package is in a git repo and when
          # a copyright and license is added to all source files
          set(ament_cmake_cpplint_FOUND TRUE)
          ament_lint_auto_find_test_dependencies()
        endif()
        install(DIRECTORY
             launch urdf rviz
             DESTINATION share/${PROJECT_NAME}/
             )
        file(GLOB AUBO_I5_VISUAL_MESHES
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/visual/*.DAE"
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/visual/*.dae")
        file(GLOB AUBO_I5_COLLISION_MESHES
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/collision/*.STL"
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/collision/*.stl")
        file(GLOB AUBO_I5_FLAT_MESHES
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/*.DAE"
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/*.dae"
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/*.STL"
             "${PROJECT_SOURCE_DIR}/meshes/aubo_i5*/*.stl")
        foreach(mesh_file IN LISTS AUBO_I5_VISUAL_MESHES AUBO_I5_COLLISION_MESHES AUBO_I5_FLAT_MESHES)
          if(NOT IS_SYMLINK "${mesh_file}")
            file(RELATIVE_PATH mesh_rel "${PROJECT_SOURCE_DIR}/meshes" "${mesh_file}")
            get_filename_component(mesh_dir "${mesh_rel}" DIRECTORY)
            install(FILES "${mesh_file}" DESTINATION "share/${PROJECT_NAME}/meshes/${mesh_dir}")
          endif()
        endforeach()
        install(PROGRAMS
             scripts/calibrate_urdf_dh.py
             DESTINATION lib/${PROJECT_NAME}
             )
        ament_package()    	
    endif()
endif()
