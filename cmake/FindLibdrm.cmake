find_package(PkgConfig)

pkg_check_modules(Libdrm libdrm)
if(Libdrm_FOUND)
  if(NOT TARGET Libdrm::drm)
    add_library(Libdrm::drm UNKNOWN IMPORTED)

    set_target_properties(Libdrm::drm PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Libdrm_INCLUDE_DIRS}"
      IMPORTED_LOCATION "${Libdrm_LINK_LIBRARIES}")
  endif()
endif()
