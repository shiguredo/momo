# 頑張って探したりせず、単純に NVCODEC_ROOT_DIR を見る
find_path(NvCodec_INCLUDE_DIR NAMES nvEncodeAPI.h PATHS "${NVCODEC_ROOT_DIR}/include")
find_library(NvCodec_LIBRARY_nvencodeapi NAMES nvencodeapi nvidia-encode PATHS "${NVCODEC_ROOT_DIR}/Lib/x64" "${NVCODEC_ROOT_DIR}/Lib/linux/stubs/x86_64")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NvCodec DEFAULT_MSG NvCodec_LIBRARY_nvencodeapi NvCodec_INCLUDE_DIR)

mark_as_advanced(NvCodec_INCLUDE_DIR NvCodec_LIBRARY_nvencodeapi)

if (NvCodec_FOUND)
  if (NOT TARGET NvCodec::nvencodeapi)
    add_library(NvCodec::nvencodeapi UNKNOWN IMPORTED)
    set_target_properties(NvCodec::nvencodeapi PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${NvCodec_INCLUDE_DIR}"
      IMPORTED_LOCATION "${NvCodec_LIBRARY_nvencodeapi}")
  endif()
endif()