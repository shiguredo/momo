# FindBlend2D.cmake
# Find Blend2D library

if(Blend2D_ROOT)
  set(BLEND2D_ROOT_DIR ${Blend2D_ROOT})
endif()

find_path(BLEND2D_INCLUDE_DIR
  NAMES blend2d.h
  PATHS ${BLEND2D_ROOT_DIR}/include
  NO_DEFAULT_PATH
)

find_library(BLEND2D_LIBRARY
  NAMES blend2d
  PATHS ${BLEND2D_ROOT_DIR}/lib
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Blend2D DEFAULT_MSG
  BLEND2D_LIBRARY BLEND2D_INCLUDE_DIR
)

if(Blend2D_FOUND)
  if(NOT TARGET Blend2D::Blend2D)
    add_library(Blend2D::Blend2D STATIC IMPORTED)
    set_target_properties(Blend2D::Blend2D PROPERTIES
      IMPORTED_LOCATION ${BLEND2D_LIBRARY}
      INTERFACE_INCLUDE_DIRECTORIES ${BLEND2D_INCLUDE_DIR}
      INTERFACE_COMPILE_DEFINITIONS "BL_STATIC"
    )
  endif()
endif()

mark_as_advanced(BLEND2D_INCLUDE_DIR BLEND2D_LIBRARY)