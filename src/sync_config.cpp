#include "sync_config.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace bms
{
namespace
{

const char * source_key(TimestampSource src)
{
  return src == TimestampSource::BagReceive ? "bag_receive" : "header_stamp";
}

TimestampSource source_from_key(const std::string & key)
{
  if (key == "bag_receive") {
    return TimestampSource::BagReceive;
  }
  if (key == "header_stamp") {
    return TimestampSource::HeaderStamp;
  }
  throw std::runtime_error("unknown timestamp_source '" + key + "'");
}

}  // namespace

void save_sync_config(const std::string & path, const SyncConfig & cfg)
{
  YAML::Node root;
  root["bag_a"]["uri"] = cfg.bag_a_uri;
  root["bag_a"]["imu_topic"] = cfg.bag_a_topic;
  root["bag_b"]["uri"] = cfg.bag_b_uri;
  root["bag_b"]["imu_topic"] = cfg.bag_b_topic;
  root["timestamp_source"] = source_key(cfg.timestamp_source);
  root["time_offset_seconds"] = cfg.time_offset_seconds;
  // Integer nanoseconds for consumers that would rather not re-parse a double.
  root["time_offset_nanoseconds"] =
    static_cast<long long>(std::llround(cfg.time_offset_seconds * 1e9));

  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot open '" + path + "' for writing");
  }
  out << "# bag_manual_synchronizer result\n"
      << "# Bag B is aligned to bag A's clock by:\n"
      << "#   t_aligned = t_b + time_offset_seconds\n"
      << YAML::Dump(root) << "\n";
  if (!out) {
    throw std::runtime_error("failed while writing '" + path + "'");
  }
}

SyncConfig load_sync_config(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot parse '" + path + "': " + e.what());
  }

  SyncConfig cfg;
  if (root["bag_a"]) {
    cfg.bag_a_uri = root["bag_a"]["uri"].as<std::string>("");
    cfg.bag_a_topic = root["bag_a"]["imu_topic"].as<std::string>("");
  }
  if (root["bag_b"]) {
    cfg.bag_b_uri = root["bag_b"]["uri"].as<std::string>("");
    cfg.bag_b_topic = root["bag_b"]["imu_topic"].as<std::string>("");
  }
  if (root["timestamp_source"]) {
    cfg.timestamp_source = source_from_key(root["timestamp_source"].as<std::string>());
  }
  cfg.time_offset_seconds = root["time_offset_seconds"].as<double>(0.0);
  return cfg;
}

}  // namespace bms
