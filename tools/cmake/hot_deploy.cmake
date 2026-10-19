# Copies SRC into DST if DST exists (the installed SteamVR driver folder).
# With RESTART_OVERLAY, stops the running overlay first and starts the new one.
# With STOP_FIRST, stops the running program first (the overlay restarts the
# standalone engine host by itself).
if(NOT IS_DIRECTORY "${DST}")
  return()
endif()
get_filename_component(name "${SRC}" NAME)
if(RESTART_OVERLAY)
  execute_process(COMMAND taskkill /IM "${name}" /F OUTPUT_QUIET ERROR_QUIET RESULT_VARIABLE killed)
  # taskkill returns before the process is gone: started too soon, the new overlay
  # finds the old one's single-instance mutex and quits
  if(killed EQUAL 0)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 2)
  endif()
endif()
if(STOP_FIRST)
  execute_process(COMMAND taskkill /IM "${name}" /F OUTPUT_QUIET ERROR_QUIET)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 1)
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${SRC}" "${DST}/${name}"
                RESULT_VARIABLE res)
if(NOT res EQUAL 0)
  message(WARNING "hot deploy: could not copy ${name} to ${DST}")
  return()
endif()
message(STATUS "hot deploy: ${name} -> ${DST}")
if(RESTART_OVERLAY AND killed EQUAL 0)
  # Only restart it if it was open
  # Start-Process (ShellExecute) does not hand our output pipes to the overlay,
  # which would keep the build waiting until the overlay exits
  execute_process(COMMAND powershell -NoProfile -Command "Start-Process -FilePath '${DST}/${name}'")
endif()
