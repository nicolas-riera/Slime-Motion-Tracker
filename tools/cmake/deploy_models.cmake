# Copies the engine models folder SRC into DST/models if DST exists (the installed
# SteamVR driver folder), next to the hot-deployed engine DLL.
if(NOT IS_DIRECTORY "${DST}")
  return()
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory "${SRC}" "${DST}/models" RESULT_VARIABLE res)
if(NOT res EQUAL 0)
  message(WARNING "hot deploy: could not copy the models to ${DST}/models")
endif()
