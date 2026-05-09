set(_libxr_st_driver_sources)

if(LIBXR_ST_DRIVER_COMPONENTS)
  foreach(_libxr_st_component IN LISTS LIBXR_ST_DRIVER_COMPONENTS)
    string(TOLOWER "${_libxr_st_component}" _libxr_st_component)

    if(_libxr_st_component STREQUAL "atomic_shim")
      set(_libxr_st_source "${CMAKE_CURRENT_LIST_DIR}/stm32_atomic_shim.c")
    else()
      set(_libxr_st_source
          "${CMAKE_CURRENT_LIST_DIR}/stm32_${_libxr_st_component}.cpp")
    endif()

    if(NOT EXISTS "${_libxr_st_source}")
      message(FATAL_ERROR
              "Unknown LIBXR_ST_DRIVER_COMPONENTS entry: ${_libxr_st_component}")
    endif()

    list(APPEND _libxr_st_driver_sources "${_libxr_st_source}")
  endforeach()
else()
  file(GLOB _libxr_st_driver_sources
       "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
       "${CMAKE_CURRENT_LIST_DIR}/*.c")
endif()

list(REMOVE_DUPLICATES _libxr_st_driver_sources)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${_libxr_st_driver_sources})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
