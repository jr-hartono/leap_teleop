#pragma once

#include <linux/input.h>

#include <string>
#include <vector>

namespace leap_teleop {

std::vector<std::string> discover_keyboards();

int key_code_from_name(const std::string& name);

class KeyboardDeadman {
 public:
  KeyboardDeadman(std::string device_path, int key_code);
  ~KeyboardDeadman();

  KeyboardDeadman(const KeyboardDeadman&) = delete;
  KeyboardDeadman& operator=(const KeyboardDeadman&) = delete;

  void open_device();
  void poll();
  void close_device();

  [[nodiscard]] bool engaged() const;
  [[nodiscard]] bool healthy() const;
  [[nodiscard]] const std::string& device_path() const;

 private:
  std::string device_path_;
  int key_code_ = 0;
  int fd_ = -1;
  bool engaged_ = false;
  bool healthy_ = false;
};

}  // namespace leap_teleop
