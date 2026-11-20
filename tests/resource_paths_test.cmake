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
file(MAKE_DIRECTORY "${RUN_ROOT}/selection") # Isolate automatic selection fixtures from the first-run fallback checks above.
configure_file("${CONFIG}" "${RUN_ROOT}/selection/delivery_config.json" COPYONLY) # Keep a valid seed available while testing preference order.
configure_file("${CONFIG}" "${RUN_ROOT}/selection/calibration_16in.json" COPYONLY) # Supply a valid saved default calibration with no activation record in this directory.
file(WRITE "${RUN_ROOT}/selection/active_calibration.json" "{\"schema_version\":1,\"config_path\":\"custom profile.json\"}") # Select a custom calibration using a relative filename containing a space.
file(READ "${CONFIG}" saved_profile) # Start from the valid fixture before modeling a saved calibration without explicit pacing.
string(REGEX REPLACE "\"speed_scale\"[ \t\r\n]*:[ \t\r\n]*[0-9.]+[ \t\r\n]*," "" saved_profile "${saved_profile}") # Omit the optional member so actual executable startup must apply default pacing.
file(WRITE "${RUN_ROOT}/selection/custom profile.json" "${saved_profile}") # Keep the selected calibration's geometry and nominal speed unchanged.
execute_process(
  COMMAND "${APP}" --demo --headless --seconds 0.1 --output auto-runs # Omit --config to exercise the actual executable's automatic selection.
  WORKING_DIRECTORY "${RUN_ROOT}/selection"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 20)
if(NOT status EQUAL 0 OR NOT output MATCHES "Configuration:.*custom profile.json") # The saved active record must take precedence over both default-name files.
  message(FATAL_ERROR "Automatic calibration selection failed: ${status}\n${output}\n${errors}")
endif()
if(NOT output MATCHES "Robot speed: 2[.]24 cm/s [(]80% pace[)]") # Loading an active calibration without a pacing field must use the slower default automatically.
  message(FATAL_ERROR "Saved calibration did not receive default pacing: ${output}")
endif()
execute_process(
  COMMAND "${APP}" --demo --headless --seconds 0.1 --config delivery_config.json --output explicit-runs # An explicit seed request must bypass the active calibration.
  WORKING_DIRECTORY "${RUN_ROOT}/selection"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 20)
if(NOT status EQUAL 0 OR NOT output MATCHES "Configuration:.*delivery_config.json") # Confirm CLI parsing preserves the distinction between explicit and default paths.
  message(FATAL_ERROR "Explicit configuration did not override calibration: ${status}\n${output}\n${errors}")
endif()
file(WRITE "${RUN_ROOT}/selection/active_calibration.json" "{\"schema_version\":1,\"config_path\":\"absent.json\"}") # Point the preference at a missing calibration to test failure behavior.
execute_process(
  COMMAND "${APP}" --demo --headless --seconds 0.1 --output invalid-runs # A normal launch must report missing active data instead of silently switching alignment.
  WORKING_DIRECTORY "${RUN_ROOT}/selection"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 20)
if(status EQUAL 0 OR NOT errors MATCHES "Cannot open config") # Invalid active preferences must stop before opening devices or running with seed geometry.
  message(FATAL_ERROR "Missing active calibration was not rejected: ${status}\n${output}\n${errors}")
endif()
message(STATUS "Local resources, saved calibration selection, explicit override, and missing-path rejection passed") # Report success only after actual executable startup verifies the preference rules.
