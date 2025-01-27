set(SOURCES
  mjbots/moteus/moteus_multiplex.h
  mjbots/moteus/moteus_optional.h
  mjbots/moteus/moteus_protocol.h
  mjbots/moteus/moteus_tokenizer.h
  mjbots/moteus/moteus_transport.h
)

add_library(moteus INTERFACE ${SOURCES})
add_library(moteus::cpp ALIAS moteus)
target_include_directories(
    moteus
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mjbots>
        $<INSTALL_INTERFACE:include>
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/moteus-config-version.cmake"
    VERSION ${CMAKE_PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS moteus
    EXPORT moteus-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES
    DESTINATION include
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mjbots/ DESTINATION include)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/moteus-config.cmake.in"
    "${PROJECT_BINARY_DIR}/moteus-config.cmake"
    INSTALL_DESTINATION lib/cmake/moteus
)

install(EXPORT moteus-targets DESTINATION lib/cmake/moteus)
install(FILES "${PROJECT_BINARY_DIR}/moteus-config-version.cmake"
              "${PROJECT_BINARY_DIR}/moteus-config.cmake"
        DESTINATION lib/cmake/moteus)
