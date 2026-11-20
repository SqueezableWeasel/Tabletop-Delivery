if(NOT DEFINED APP OR NOT DEFINED CONFIG OR NOT DEFINED RUN_ROOT) # Run the built app from a directory containing only a copied experiment config. This catches accidental dependencies on the source tree or launch location. Require the executable, fixture config, and independent working directory supplied by CTest.
  message(FATAL_ERROR "APP, CONFIG, and RUN_ROOT are required") # Mark this CTest invocation failed with captured startup diagnostics or the violated contract.
endif()
file(MAKE_DIRECTORY "${RUN_ROOT}") # Create a separate launch directory to expose accidental dependencies on source-tree location.
configure_file("${CONFIG}" "${RUN_ROOT}/delivery_config.json" COPYONLY) # Copy only the experiment JSON into the independent directory without performing variable substitution.

execute_process( # Run a bounded application check and capture its exit code plus both output streams.
  COMMAND "${APP}" --demo --headless --seconds 0.3 --output runs # Exercise startup with the default local configuration name and save outputs beneath this working directory.
  WORKING_DIRECTORY "${RUN_ROOT}" # Launch from the independent test directory rather than the project source root.
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors # Keep return status, normal output, and error output available for assertions and failure reports.
  TIMEOUT 20) # Bound the child process so a startup or shutdown regression cannot hang the test indefinitely.
if(NOT status EQUAL 0 OR NOT output MATCHES "Stopped. Logs and snapshot:") # Require both a successful exit and the application's normal final-snapshot completion message.
  message(FATAL_ERROR "Local configuration startup failed: ${status}\n${output}\n${errors}") # Mark this CTest invocation failed with captured startup diagnostics or the violated contract.
endif()
file(GLOB snapshots "${RUN_ROOT}/runs/*/snapshot_1/state.json") # Inspect the independent output directory for the final structured snapshot artifact.
if(NOT snapshots) # Successful startup must also produce the promised final state snapshot.
  message(FATAL_ERROR "Standalone run did not write a final snapshot") # Mark this CTest invocation failed with captured startup diagnostics or the violated contract.
endif()

execute_process( # An explicit missing path must fail, rather than silently select another config. Run a bounded application check and capture its exit code plus both output streams.
  COMMAND "${APP}" --demo --headless --seconds 0.1 --config absent-config.json # Explicitly request an absent config and require failure rather than an implicit fallback.
  WORKING_DIRECTORY "${RUN_ROOT}" # Launch from the independent test directory rather than the project source root.
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors # Keep return status, normal output, and error output available for assertions and failure reports.
  TIMEOUT 20) # Bound the child process so a startup or shutdown regression cannot hang the test indefinitely.
if(status EQUAL 0 OR NOT errors MATCHES "Cannot open config") # Missing explicit configuration must fail with the expected actionable file-open error.
  message(FATAL_ERROR "Missing explicit config was not rejected: ${status}\n${output}\n${errors}") # Mark this CTest invocation failed with captured startup diagnostics or the violated contract.
endif()
message(STATUS "Local resources, diagnostic snapshot, and missing-path rejection passed") # Report completion only after local-resource startup, snapshot creation, and rejection checks pass.
