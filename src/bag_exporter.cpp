#include "bag_exporter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>

#include "rclcpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace bms
{
namespace
{

constexpr std::int64_t kNsPerSec = 1000000000LL;
/// A CDR stream opens with a 4-byte encapsulation header; the message body,
/// and so a leading Header's `stamp`, starts right after it.
constexpr std::size_t kCdrBodyOffset = 4;

namespace introspection = rosidl_typesupport_introspection_cpp;

/// True when the first field of `type_name` is a std_msgs/Header, which is
/// what makes the stamp's byte position in the CDR stream predictable.
///
/// The type support library has to stay loaded for as long as the handle is
/// used, so it is handed back to the caller to own.
bool type_starts_with_header(
  const std::string & type_name,
  std::vector<std::shared_ptr<rcpputils::SharedLibrary>> & keep_alive)
{
  try {
    auto library = rclcpp::get_typesupport_library(type_name, introspection::typesupport_identifier);
    if (library == nullptr) {
      return false;
    }
    const rosidl_message_type_support_t * ts = rclcpp::get_message_typesupport_handle(
      type_name, introspection::typesupport_identifier, *library);
    if (ts == nullptr || ts->data == nullptr) {
      return false;
    }
    keep_alive.push_back(library);

    const auto * members = static_cast<const introspection::MessageMembers *>(ts->data);
    if (members->member_count_ == 0 || members->members_ == nullptr) {
      return false;
    }
    const introspection::MessageMember & first = members->members_[0];
    if (first.is_array_ || first.type_id_ != introspection::ROS_TYPE_MESSAGE ||
      first.members_ == nullptr || first.members_->data == nullptr)
    {
      return false;
    }
    // The field has to sit at the very start of the struct as well as be the
    // first declared one; anything else would move the stamp's byte offset.
    if (first.offset_ != 0) {
      return false;
    }
    const auto * sub = static_cast<const introspection::MessageMembers *>(first.members_->data);
    return std::string(sub->message_name_) == "Header" &&
           std::string(sub->message_namespace_) == "std_msgs::msg";
  } catch (const std::exception &) {
    // A type whose support library is not installed simply gets no header
    // rewriting; the message itself is still copied through untouched.
    return false;
  }
}

std::uint32_t read_u32(const std::uint8_t * b, bool little_endian)
{
  if (little_endian) {
    return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
           (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
  }
  return static_cast<std::uint32_t>(b[3]) | (static_cast<std::uint32_t>(b[2]) << 8) |
         (static_cast<std::uint32_t>(b[1]) << 16) | (static_cast<std::uint32_t>(b[0]) << 24);
}

void write_u32(std::uint8_t * b, std::uint32_t v, bool little_endian)
{
  if (little_endian) {
    b[0] = static_cast<std::uint8_t>(v & 0xff);
    b[1] = static_cast<std::uint8_t>((v >> 8) & 0xff);
    b[2] = static_cast<std::uint8_t>((v >> 16) & 0xff);
    b[3] = static_cast<std::uint8_t>((v >> 24) & 0xff);
  } else {
    b[3] = static_cast<std::uint8_t>(v & 0xff);
    b[2] = static_cast<std::uint8_t>((v >> 8) & 0xff);
    b[1] = static_cast<std::uint8_t>((v >> 16) & 0xff);
    b[0] = static_cast<std::uint8_t>((v >> 24) & 0xff);
  }
}

enum class StampPatch
{
  Shifted,
  LeftZero,    ///< an unset stamp, deliberately not moved
  Clamped,     ///< would have gone negative
  NotPatched,  ///< buffer too short to hold a stamp
};

/// Rewrite the `stamp` of a leading std_msgs/Header directly in the CDR
/// buffer. Patching the four+four bytes in place keeps every other field of
/// the message bit-identical, which a deserialize/reserialize round-trip
/// through a generic type support would not guarantee.
StampPatch patch_header_stamp(rcutils_uint8_array_t & buf, std::int64_t delta_ns)
{
  if (buf.buffer == nullptr || buf.buffer_length < kCdrBodyOffset + 8) {
    return StampPatch::NotPatched;
  }
  // Byte 1 of the encapsulation carries the endianness of everything after it.
  const bool little_endian = (buf.buffer[1] & 0x01) != 0;
  std::uint8_t * stamp = buf.buffer + kCdrBodyOffset;

  const auto sec = static_cast<std::int32_t>(read_u32(stamp, little_endian));
  const std::uint32_t nsec = read_u32(stamp + 4, little_endian);

  // A zero stamp means "unset" in ROS, not "1970"; shifting it would invent a
  // timestamp that the recording never had.
  if (sec == 0 && nsec == 0) {
    return StampPatch::LeftZero;
  }

  const std::int64_t total = static_cast<std::int64_t>(sec) * kNsPerSec +
    static_cast<std::int64_t>(nsec);
  std::int64_t shifted = total + delta_ns;
  bool clamped = false;
  if (shifted < 0) {
    shifted = 0;
    clamped = true;
  }

  const std::int64_t new_sec = shifted / kNsPerSec;
  const std::int64_t new_nsec = shifted % kNsPerSec;
  if (new_sec > INT32_MAX) {
    return StampPatch::NotPatched;
  }

  write_u32(stamp, static_cast<std::uint32_t>(static_cast<std::int32_t>(new_sec)), little_endian);
  write_u32(stamp + 4, static_cast<std::uint32_t>(new_nsec), little_endian);
  return clamped ? StampPatch::Clamped : StampPatch::Shifted;
}

}  // namespace

ExportResult export_shifted_bag(
  const std::string & src_uri,
  const std::string & dst_uri,
  double offset_seconds,
  const ExportOptions & opts,
  ExportProgress * progress)
{
  ExportResult result;
  result.destination = dst_uri;

  const auto delta_ns = static_cast<std::int64_t>(std::llround(offset_seconds * 1e9));

  rosbag2_cpp::Reader reader;
  reader.open(src_uri);

  const auto & metadata = reader.get_metadata();
  if (progress != nullptr) {
    progress->total.store(metadata.message_count, std::memory_order_relaxed);
    progress->written.store(0, std::memory_order_relaxed);
  }

  // Message definitions travel with the bag so the output MCAP carries real
  // schemas; anything the source did not record is looked up locally, which
  // is what makes a sqlite3 -> mcap export readable by external tools.
  std::vector<rosbag2_storage::MessageDefinition> definitions;
  reader.get_all_message_definitions(definitions);
  std::map<std::string, rosbag2_storage::MessageDefinition> definition_by_type;
  for (const auto & def : definitions) {
    definition_by_type[def.topic_type] = def;
  }
  rosbag2_cpp::LocalMessageDefinitionSource local_definitions;

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = dst_uri;
  storage_options.storage_id = opts.storage_id;
  writer.open(storage_options, rosbag2_cpp::ConverterOptions{"", ""});

  std::vector<std::shared_ptr<rcpputils::SharedLibrary>> typesupport_libraries;
  std::map<std::string, bool> patch_topic;

  for (const auto & topic : reader.get_all_topics_and_types()) {
    auto definition = definition_by_type.find(topic.type);
    if (definition != definition_by_type.end() &&
      !definition->second.encoded_message_definition.empty())
    {
      writer.create_topic(topic, definition->second);
    } else {
      try {
        writer.create_topic(topic, local_definitions.get_full_text(topic.type));
      } catch (const std::exception &) {
        writer.create_topic(
          topic, rosbag2_storage::MessageDefinition::empty_message_definition_for(topic.type));
      }
    }

    // Only CDR has the byte layout the stamp patch relies on.
    const bool patchable = opts.shift_header_stamps &&
      topic.serialization_format == "cdr" &&
      type_starts_with_header(topic.type, typesupport_libraries);
    patch_topic[topic.name] = patchable;
    if (patchable) {
      result.header_topics.push_back(topic.name);
    } else {
      result.stampless_topics.push_back(topic.name);
    }
  }

  while (reader.has_next()) {
    if (progress != nullptr && progress->cancel.load(std::memory_order_relaxed)) {
      result.cancelled = true;
      break;
    }
    auto message = reader.read_next();

    message->recv_timestamp += delta_ns;
    message->send_timestamp += delta_ns;

    const auto patchable = patch_topic.find(message->topic_name);
    if (patchable != patch_topic.end() && patchable->second && message->serialized_data) {
      switch (patch_header_stamp(*message->serialized_data, delta_ns)) {
        case StampPatch::Shifted:
          ++result.header_stamps_shifted;
          break;
        case StampPatch::LeftZero:
          ++result.zero_stamps_kept;
          break;
        case StampPatch::Clamped:
          ++result.header_stamps_shifted;
          ++result.clamped_stamps;
          break;
        case StampPatch::NotPatched:
          break;
      }
    }

    writer.write(message);
    ++result.messages_written;
    if (progress != nullptr && (result.messages_written % 512) == 0) {
      progress->written.store(result.messages_written, std::memory_order_relaxed);
    }
  }

  writer.close();
  reader.close();
  if (progress != nullptr) {
    progress->written.store(result.messages_written, std::memory_order_relaxed);
  }
  return result;
}

}  // namespace bms
