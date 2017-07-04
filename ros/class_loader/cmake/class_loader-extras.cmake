# hides all symbols of a library
function(class_loader_hide_library_symbols target)
  set(version_script "${CMAKE_CURRENT_BINARY_DIR}/class_loader_hide_library_symbols__${target}.script")
  file(WRITE "${version_script}"
    "    {
      local:
        *;
    };"
  )
  # checks if the linker supports version script
  include(TestCXXAcceptsFlag)
  check_cxx_accepts_flag("-Wl,--version-script,\"${version_script}\"" LD_ACCEPTS_VERSION_SCRIPT)
  if(LD_ACCEPTS_VERSION_SCRIPT)
    set_target_properties(${target} PROPERTIES LINK_FLAGS "-Wl,-version-script=\"${version_script}\"")
  endif()
endfunction()
