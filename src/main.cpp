#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "app.hpp"

namespace
{

void print_usage(const char * argv0)
{
  std::printf(
    "Usage: %s [BAG_A] [BAG_B] [--config FILE]\n"
    "\n"
    "Overlay the IMU streams of two ROS 2 bags, slide bag B in time until it\n"
    "lines up with bag A, and save the resulting offset.\n"
    "\n"
    "  BAG_A, BAG_B     bag directories or single storage files, loaded at startup\n"
    "  --config FILE    restore bag paths and offset from a previously saved file\n"
    "  -h, --help       show this message\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> bags;
  std::string config;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    }
    if (arg == "--config") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "--config needs a file path\n");
        return 1;
      }
      config = argv[++i];
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::fprintf(stderr, "unknown option '%s'\n", arg.c_str());
      return 1;
    }
    if (bags.size() >= 2) {
      std::fprintf(stderr, "at most two bag paths can be given\n");
      return 1;
    }
    bags.push_back(arg);
  }

  try {
    bms::App app;
    if (!config.empty()) {
      app.preload_config(config);
    }
    for (std::size_t i = 0; i < bags.size(); ++i) {
      app.preload(static_cast<int>(i), bags[i]);
    }
    return app.run();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    return 1;
  }
}
