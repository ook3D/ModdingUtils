if(NOT DEFINED SRC OR NOT DEFINED DST)
    message(FATAL_ERROR "StageDll.cmake: SRC and DST must be defined.")
endif()

get_filename_component(_dst_dir "${DST}" DIRECTORY)
file(MAKE_DIRECTORY "${_dst_dir}")

execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${SRC}" "${DST}"
    RESULT_VARIABLE _rc
    OUTPUT_QUIET
    ERROR_VARIABLE  _err
)

if(_rc EQUAL 0)
    message(STATUS "Staged DLL -> ${DST}")
else()
    message(WARNING
        "Could not stage DLL to:\n"
        "  ${DST}\n"
        "Build itself succeeded; the file is probably locked by a running\n"
        "Blender. Close Blender and rerun build.bat to refresh the staged DLL.\n"
        "Underlying error: ${_err}")
endif()
