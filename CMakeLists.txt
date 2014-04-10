include(${SOFA_CMAKE_DIR}/preProject.cmake)

if(APPLE)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES

	init.h

	odesolver/pouf_solver.h

	tool/mapping_graph.h
	tool/assemble.h
	tool/find_insert.h
	tool/graph_vector.h

	collision/triangle_model.h
	
	)

set(SOURCE_FILES

    init.cpp

	odesolver/pouf_solver.cpp

	tool/mapping_graph.cpp
	tool/assemble.cpp
	tool/graph_vector.cpp

	collision/triangle_model.cpp
	)

set(README_FILES
	README.md
	)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${README_FILES})

AddCompilerDefinitions("SOFA_BUILD_POUF")
AddLinkerDependencies(SofaGuiQt Compliant)


include(${SOFA_CMAKE_DIR}/postProject.cmake)
