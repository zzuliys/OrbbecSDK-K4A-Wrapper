find_package(Doxygen)
message("Doxygen_FOUND: ${Doxygen_FOUND}")

if(Doxygen_FOUND)
    set(OB_K4A_API_DOC_OUTPUT_DIR ${CMAKE_BINARY_DIR}/doc/api/)
    file(MAKE_DIRECTORY  ${CMAKE_BINARY_DIR}/doc/api/)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    add_custom_target(ob_k4a_api_doc ALL
        COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating Orbbec SDK K4A Wrapper API Reference documentation with Doxygen"
        VERBATIM
    )
endif()