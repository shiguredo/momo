# 頑張って探したりせず、単純に JSON_ROOT_DIR を見る
if (NOT TARGET JSON::JSON)
  add_library(JSON::JSON INTERFACE IMPORTED)
  set_target_properties(JSON::JSON PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${JSON_ROOT_DIR}/include")
endif()