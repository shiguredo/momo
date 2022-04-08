find_package(LIBVA REQUIRED)

find_path(MSDK_INCLUDE_DIR NAMES mfx/mfxenc.h PATHS "${MSDK_ROOT_DIR}/include" NO_DEFAULT_PATH)
find_library(MSDK_LIBRARY NAMES mfx PATHS "${MSDK_ROOT_DIR}/lib" NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MSDK DEFAULT_MSG MSDK_LIBRARY MSDK_INCLUDE_DIR)

mark_as_advanced(MSDK_INCLUDE_DIR MSDK_LIBRARY)

if(MSDK_FOUND)
  if(NOT TARGET MSDK::MSDK)
    add_library(MSDK::MSDK UNKNOWN IMPORTED)

    target_link_libraries(MSDK::MSDK INTERFACE LIBVA::LIBVA LIBVA::LIBVA_DRM)
    set_target_properties(MSDK::MSDK PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MSDK_INCLUDE_DIR}"
      IMPORTED_LOCATION "${MSDK_LIBRARY}")
  endif()
endif()