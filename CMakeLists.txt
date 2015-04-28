include(${SOFA_CMAKE_DIR}/preProject.cmake)

if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES

	init.h

	odesolver/pouf_solver.h

	tool/mapping_graph.h
	tool/assemble.h
	tool/find_insert.h
	tool/graph_vector.h
	tool/data.h

	linearsolver/pgs.h
	linearsolver/jacobi.h

	mapping/RigidMultiMapping.h
	mapping/LocalFrameMapping.h
	
	thread/pool.h

	math/anderson.h
	math/nlnscg.h

	)

set(SOURCE_FILES

    init.cpp

	odesolver/pouf_solver.cpp

	tool/mapping_graph.cpp
	tool/assemble.cpp
	tool/graph_vector.cpp
	tool/propagate_position_velocity.cpp
	tool/data.cpp
	tool/disable_rigid_mouse_interaction.cpp

	linearsolver/pgs.cpp
	linearsolver/jacobi.cpp
	
	mapping/RigidMultiMapping.cpp
	mapping/LocalFrameMapping.cpp
	
	math/anderson.cpp
	math/nlnscg.cpp
   )


set(README_FILES
	README.md
	)

if(NOT MSVC)
  AddCompilerDefinitions("SOFA_HAVE_VARIADIC_TEMPLATES")
endif()

if(APPLE)


  AddCompilerDefinitions("SOFA_HAVE_STD_THREAD")
  list(APPEND HEADER_FILES thread/worker.h)
  list(APPEND SOURCE_FILES thread/worker.cpp)
endif()


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${README_FILES})


# if(APPLE)
#   target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
# endif()

AddCompilerDefinitions("SOFA_BUILD_POUF")
AddLinkerDependencies(SofaGuiQt Compliant)

include(${SOFA_CMAKE_DIR}/postProject.cmake)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
remove_definitions(-DNDEBUG )

