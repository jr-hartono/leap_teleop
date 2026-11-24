# C++ driver linking the LeapSDK CMake package

The tracker is C++ (`rclcpp`) and links the `LeapSDK::LeapC` target exported by the
Ultraleap Hand Tracking Service, found with `find_package(LeapSDK)`. The Python route was
rejected because the only binding, `leapc_cffi`, comes prebuilt for CPython 3.8 and would
have to be rebuilt from source per machine and per Python version.

## Consequences

- No vendored SDK and no generated bindings; the driver compiles against the installed
  service, which is also the component that owns the USB device.
- The build depends on a Ubuntu-packaged CMake config. `find_package(LeapSDK QUIET)` keeps
  the rest of the workspace buildable where the service is absent.
- The service version is effectively pinned: Gemini is the last line supporting the
  original controller, so `libLeapC` and the daemon must stay together.
