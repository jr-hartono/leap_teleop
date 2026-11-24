#include "leap_teleop_core/keyboard_deadman.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace leap_teleop {

namespace {

bool ends_with(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool canonical_path(const std::string& path, std::string& result) {
  char resolved[PATH_MAX];
  if (realpath(path.c_str(), resolved) == nullptr) {
    return false;
  }
  result = resolved;
  return true;
}

bool is_keyboard_device(const std::string& path) {
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }
  unsigned char ev_bits[(EV_MAX + 7) / 8] = {};
  unsigned char key_bits[(KEY_MAX + 7) / 8] = {};
  const bool ok = ioctl(fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) >= 0 &&
                  (ev_bits[EV_KEY / 8] & (1 << (EV_KEY % 8))) != 0 &&
                  ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) >= 0 &&
                  (key_bits[KEY_SPACE / 8] & (1 << (KEY_SPACE % 8))) != 0;
  close(fd);
  return ok;
}

std::unordered_map<std::string, int> build_key_map() {
  std::unordered_map<std::string, int> keys = {
      {"space", KEY_SPACE},
      {"leftctrl", KEY_LEFTCTRL},
      {"rightctrl", KEY_RIGHTCTRL},
      {"leftshift", KEY_LEFTSHIFT},
      {"rightshift", KEY_RIGHTSHIFT},
      {"leftalt", KEY_LEFTALT},
      {"rightalt", KEY_RIGHTALT},
      {"leftmeta", KEY_LEFTMETA},
      {"rightmeta", KEY_RIGHTMETA},
      {"enter", KEY_ENTER},
      {"tab", KEY_TAB},
      {"escape", KEY_ESC},
      {"esc", KEY_ESC},
      {"capslock", KEY_CAPSLOCK},
      {"backspace", KEY_BACKSPACE},
      {"delete", KEY_DELETE},
      {"insert", KEY_INSERT},
      {"home", KEY_HOME},
      {"end", KEY_END},
      {"pageup", KEY_PAGEUP},
      {"pagedown", KEY_PAGEDOWN},
      {"up", KEY_UP},
      {"down", KEY_DOWN},
      {"left", KEY_LEFT},
      {"right", KEY_RIGHT},
  };
  for (int i = 0; i < 26; ++i) {
    keys[std::string(1, static_cast<char>('a' + i))] = KEY_A + i;
  }
  for (int i = 0; i < 10; ++i) {
    keys[std::to_string(i)] = (i == 0) ? KEY_0 : KEY_1 + i - 1;
  }
  for (int i = 0; i < 12; ++i) {
    keys["f" + std::to_string(i + 1)] = KEY_F1 + i;
  }
  return keys;
}

}  // namespace

std::vector<std::string> discover_keyboards() {
  std::vector<std::string> candidates;

  DIR* by_id = opendir("/dev/input/by-id");
  if (by_id != nullptr) {
    while (dirent* entry = readdir(by_id)) {
      const std::string name(entry->d_name);
      if (!ends_with(name, "-event-kbd")) {
        continue;
      }
      std::string resolved;
      if (canonical_path("/dev/input/by-id/" + name, resolved) &&
          std::find(candidates.begin(), candidates.end(), resolved) == candidates.end()) {
        candidates.push_back(resolved);
      }
    }
    closedir(by_id);
  }

  DIR* input = opendir("/dev/input");
  if (input != nullptr) {
    while (dirent* entry = readdir(input)) {
      const std::string name(entry->d_name);
      if (name.compare(0, 5, "event") != 0) {
        continue;
      }
      const std::string path = "/dev/input/" + name;
      if (is_keyboard_device(path) &&
          std::find(candidates.begin(), candidates.end(), path) == candidates.end()) {
        candidates.push_back(path);
      }
    }
    closedir(input);
  }

  std::sort(candidates.begin(), candidates.end());
  return candidates;
}

int key_code_from_name(const std::string& name) {
  static const std::unordered_map<std::string, int> keys = build_key_map();
  const auto found = keys.find(name);
  if (found == keys.end()) {
    throw std::invalid_argument("unknown key name '" + name + "' for parameter 'key'");
  }
  return found->second;
}

KeyboardDeadman::KeyboardDeadman(std::string device_path, int key_code)
    : device_path_(std::move(device_path)), key_code_(key_code) {}

KeyboardDeadman::~KeyboardDeadman() { close_device(); }

void KeyboardDeadman::open_device() {
  if (fd_ >= 0) {
    return;
  }
  fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open keyboard device '" + device_path_ +
                             "': " + std::strerror(errno));
  }
  healthy_ = true;
}

void KeyboardDeadman::poll() {
  if (fd_ < 0) {
    return;
  }
  input_event event;
  while (true) {
    const ssize_t count = read(fd_, &event, sizeof(event));
    if (count == static_cast<ssize_t>(sizeof(event))) {
      if (event.type == EV_KEY && event.code == key_code_) {
        engaged_ = (event.value == 1);
      }
      continue;
    }
    if (count >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      if (count < 0 && (errno == ENODEV || errno == EIO)) {
        healthy_ = false;
        engaged_ = false;
      }
    }
    return;
  }
}

void KeyboardDeadman::close_device() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  engaged_ = false;
  healthy_ = false;
}

bool KeyboardDeadman::engaged() const { return engaged_; }

bool KeyboardDeadman::healthy() const { return healthy_; }

const std::string& KeyboardDeadman::device_path() const { return device_path_; }

}  // namespace leap_teleop
