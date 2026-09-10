#include "app.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_stdlib.h"
#include "implot.h"

namespace bms
{
namespace
{

constexpr int kWindowWidth = 1500;
constexpr int kWindowHeight = 950;

/// Bag A gets the cool colours, bag B the warm ones, so a glance at the plot
/// says which trace belongs to which bag regardless of the channel.
const ImVec4 kColorA[4] = {
  ImVec4(0.30f, 0.62f, 0.95f, 1.0f),   // x
  ImVec4(0.25f, 0.80f, 0.75f, 1.0f),   // y
  ImVec4(0.55f, 0.50f, 0.95f, 1.0f),   // z
  ImVec4(0.20f, 0.45f, 0.90f, 1.0f),   // magnitude
};
const ImVec4 kColorB[4] = {
  ImVec4(0.95f, 0.55f, 0.20f, 1.0f),
  ImVec4(0.90f, 0.75f, 0.25f, 1.0f),
  ImVec4(0.90f, 0.40f, 0.55f, 1.0f),
  ImVec4(0.92f, 0.35f, 0.15f, 1.0f),
};

/// File managers hand over whatever the user grabbed: the bag directory, or a
/// file inside it. rosbag2 opens a directory or a storage file directly, but
/// not the metadata, so redirect that one case to the containing directory.
std::string normalize_bag_path(const char * dropped)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path path(dropped);
  if (path.filename() == "metadata.yaml" && fs::is_regular_file(path, ec)) {
    return path.parent_path().string();
  }
  return path.string();
}

/// Sit the exported bag next to the source: /data/bag_b -> /data/bag_b_synced.
std::string default_export_path(const std::string & uri)
{
  namespace fs = std::filesystem;
  if (uri.empty()) {
    return {};
  }
  fs::path path = fs::path(uri).lexically_normal();
  if (path.filename().empty()) {
    path = path.parent_path();   // the URI ended in a separator
  }
  if (path.has_extension()) {
    path = path.parent_path() / path.stem();   // a single .mcap/.db3 file
  }
  return path.string() + "_synced";
}

void glfw_error_callback(int error, const char * description)
{
  std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

void status_text(const std::string & text, bool is_error)
{
  if (text.empty()) {
    return;
  }
  const ImVec4 color = is_error ?
    ImVec4(1.0f, 0.45f, 0.40f, 1.0f) : ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
  // Bag errors are long paths; wrap them instead of letting them run past the
  // edge of the panel they belong to.
  ImGui::PushStyleColor(ImGuiCol_Text, color);
  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextUnformatted(text.c_str());
  ImGui::PopTextWrapPos();
  ImGui::PopStyleColor();
}

void help_marker(const char * desc)
{
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

std::string format_seconds(double s)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(6) << s;
  return os.str();
}

/// Draw one series into the current plot, decimating by stride if asked.
void plot_series(
  const std::string & label,
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  const ImVec4 & color,
  bool decimate,
  int max_points)
{
  const int count = static_cast<int>(std::min(xs.size(), ys.size()));
  if (count == 0) {
    return;
  }
  int step = 1;
  if (decimate && max_points > 0 && count > max_points) {
    step = (count + max_points - 1) / max_points;
  }
  ImPlot::SetNextLineStyle(color, 1.4f);
  ImPlot::PlotLine(
    label.c_str(), xs.data(), ys.data(), count / step,
    ImPlotLineFlags_None, 0, static_cast<int>(sizeof(double)) * step);
}

}  // namespace

App::App()
{
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    throw std::runtime_error("failed to initialize GLFW");
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(
    kWindowWidth, kWindowHeight, "ROS 2 bag manual synchronizer", nullptr, nullptr);
  if (window_ == nullptr) {
    glfwTerminate();
    throw std::runtime_error("failed to create a GLFW window (is a display available?)");
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // The ImGui GLFW backend installs no drop callback and leaves the user
  // pointer alone, so both are ours to claim.
  glfwSetWindowUserPointer(window_, this);
  glfwSetDropCallback(window_, &App::drop_callback);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::StyleColorsDark();
  ImPlot::StyleColorsDark();
  ImGui::GetIO().IniFilename = nullptr;   // no imgui.ini litter next to the bags

  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330");
}

App::~App()
{
  // Detach any loader still running before its ImuSeries destination dies.
  if (a_.load_future.valid()) {a_.load_future.wait();}
  if (b_.load_future.valid()) {b_.load_future.wait();}
  if (align_future_.valid()) {align_future_.wait();}
  if (export_future_.valid()) {
    export_progress_.cancel.store(true, std::memory_order_relaxed);
    export_future_.wait();
  }

  if (window_ != nullptr) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
}

void App::preload(int slot, const std::string & uri)
{
  slot_ref(slot).uri_input = uri;
  pending_scan_[slot] = true;
}

void App::preload_config(const std::string & path)
{
  pending_config_ = path;
}

int App::run()
{
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != 0) {
      glfwWaitEventsTimeout(0.1);
      continue;
    }

    poll_loads();
    poll_align();
    poll_export();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    draw();

    ImGui::Render();
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.09f, 0.09f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
  }
  return 0;
}

void App::draw()
{
  // Startup actions run here so their status text lands in the normal UI.
  if (!pending_config_.empty()) {
    const std::string path = pending_config_;
    pending_config_.clear();
    try {
      const SyncConfig cfg = load_sync_config(path);
      source_ = cfg.timestamp_source;
      offset_ = cfg.time_offset_seconds;
      save_path_ = path;
      // A path given on the command line wins over the one in the config.
      if (!pending_scan_[0]) {a_.uri_input = cfg.bag_a_uri;}
      if (!pending_scan_[1]) {b_.uri_input = cfg.bag_b_uri;}
      if (!a_.uri_input.empty()) {pending_scan_[0] = true;}
      if (!b_.uri_input.empty()) {pending_scan_[1] = true;}
      save_status_ = "loaded " + path;
      save_status_is_error_ = false;
    } catch (const std::exception & e) {
      save_status_ = e.what();
      save_status_is_error_ = true;
    }
  }
  for (int i = 0; i < 2; ++i) {
    // start_load() refuses to run during an estimate; keep the request pending
    // instead of scanning now and silently never loading.
    if (pending_scan_[i] && !aligning_) {
      pending_scan_[i] = false;
      scan(i);
      if (slot_ref(i).topic_index >= 0) {
        start_load(i);
      }
    }
  }

  const ImGuiViewport * vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(vp->WorkSize);
  ImGui::Begin(
    "main", nullptr,
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled(
    "Drag bags onto the window: the next drop fills bag %s.",
    next_drop_slot_ == 0 ? "A" : "B");
  ImGui::SameLine();
  if (ImGui::SmallButton("drop A next")) {
    next_drop_slot_ = 0;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("drop B next")) {
    next_drop_slot_ = 1;
  }

  if (ImGui::BeginTable("bags", 2, ImGuiTableFlags_BordersInnerV)) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    draw_bag_panel(0);
    ImGui::TableSetColumnIndex(1);
    draw_bag_panel(1);
    ImGui::EndTable();
  }

  ImGui::Separator();
  draw_offset_controls();
  ImGui::Separator();
  draw_save_panel();
  ImGui::Separator();
  draw_export_panel();
  ImGui::Separator();

  handle_shortcuts();
  draw_plots();

  ImGui::End();
}

void App::draw_bag_panel(int slot)
{
  BagSlot & s = slot_ref(slot);
  const char * name = (slot == 0) ? "Bag A (reference)" : "Bag B (shifted)";
  ImGui::PushID(slot);

  const std::string header =
    (next_drop_slot_ == slot) ? std::string(name) + "   << next drop" : std::string(name);
  ImGui::SeparatorText(header.c_str());

  ImGui::SetNextItemWidth(-90.0f);
  const bool submitted =
    ImGui::InputTextWithHint(
    "##uri", "path to bag directory or .mcap/.db3 file", &s.uri_input,
    ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  if (ImGui::Button("Scan") || submitted) {
    scan(slot);
  }

  const auto & topics = s.data.imu_topics;
  ImGui::SetNextItemWidth(-90.0f);
  const char * preview = (s.topic_index >= 0 && s.topic_index < static_cast<int>(topics.size())) ?
    topics[s.topic_index].c_str() : "<no IMU topic>";
  if (ImGui::BeginCombo("##topic", preview)) {
    for (int i = 0; i < static_cast<int>(topics.size()); ++i) {
      const bool selected = (i == s.topic_index);
      if (ImGui::Selectable(topics[i].c_str(), selected)) {
        s.topic_index = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  // Reloading while an estimate is in flight would swap the series out from
  // under the worker thread that is reading it.
  ImGui::BeginDisabled(s.loading || aligning_ || s.topic_index < 0);
  if (ImGui::Button("Load")) {
    start_load(slot);
  }
  ImGui::EndDisabled();

  if (s.loading) {
    ImGui::Text("loading... %zu messages", s.load_progress.load(std::memory_order_relaxed));
  } else {
    status_text(s.status, s.status_is_error);
  }

  const ImuSeries & series = s.data.series;
  if (!series.empty()) {
    ImGui::Text(
      "%zu samples | %.3f s | %.1f Hz", series.size(), series.duration(), series.mean_rate());
    ImGui::Text("starts at %.6f (epoch s)", series.t_begin());
    if (series.invalid_stamps > 0) {
      ImGui::TextColored(
        ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%zu messages had an unset header stamp",
        series.invalid_stamps);
    }
    if (series.out_of_order > 0) {
      ImGui::TextColored(
        ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%zu out-of-order stamps were sorted",
        series.out_of_order);
    }
  }

  ImGui::PopID();
}

void App::draw_offset_controls()
{
  ImGui::SeparatorText("Time offset");

  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("t_aligned(B) = t(B) + offset");
  ImGui::SameLine();
  help_marker(
    "Bag A is the reference and never moves. The offset is added to bag B's "
    "timestamps; this is the value written to the output file.");

  ImGui::SetNextItemWidth(240.0f);
  ImGui::InputDouble("offset [s]", &offset_, nudge_fine_, nudge_coarse_, "%.6f");

  ImGui::SetNextItemWidth(-1.0f);
  const double slider_min = -slider_range_;
  const double slider_max = slider_range_;
  ImGui::SliderScalar(
    "##offset_slider", ImGuiDataType_Double, &offset_, &slider_min, &slider_max, "%.6f s");

  ImGui::SetNextItemWidth(140.0f);
  ImGui::InputDouble("slider range [s]", &slider_range_, 1.0, 10.0, "%.1f");
  slider_range_ = std::max(slider_range_, 1e-3);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(120.0f);
  ImGui::InputDouble("fine step", &nudge_fine_, 0.0, 0.0, "%.4f");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(120.0f);
  ImGui::InputDouble("coarse step", &nudge_coarse_, 0.0, 0.0, "%.4f");

  const double steps[] = {-1.0, -0.1, -0.01, -0.001, 0.001, 0.01, 0.1, 1.0};
  for (std::size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
    if (i != 0) {
      ImGui::SameLine();
    }
    char label[32];
    std::snprintf(label, sizeof(label), "%+g s", steps[i]);
    if (ImGui::Button(label)) {
      offset_ += steps[i];
    }
  }

  if (ImGui::Button("Reset to 0")) {
    offset_ = 0.0;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!both_loaded());
  if (ImGui::Button("Align bag starts")) {
    offset_ = a_.data.series.t_begin() - b_.data.series.t_begin();
  }
  ImGui::SameLine();
  help_marker(
    "Sets the offset so both bags begin at the same instant. Useful when the "
    "two recordings come from machines whose clocks are far apart.");
  ImGui::EndDisabled();

  ImGui::SeparatorText("Auto-align (normalized cross-correlation)");
  ImGui::BeginDisabled(!both_loaded() || aligning_ || a_.loading || b_.loading);
  if (ImGui::Button(aligning_ ? "Estimating..." : "Estimate offset")) {
    run_auto_align();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::SetNextItemWidth(140.0f);
  ImGui::InputDouble("search radius [s]", &align_opts_.search_radius_s, 0.5, 5.0, "%.2f");
  align_opts_.search_radius_s = std::clamp(align_opts_.search_radius_s, 1e-3, 3600.0);
  ImGui::SameLine();
  ImGui::Checkbox("use acceleration", &align_opts_.use_accel);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(120.0f);
  ImGui::Combo("channel", &align_opts_.channel, "x\0y\0z\0magnitude\0");
  ImGui::SameLine();
  help_marker(
    "Searches for the lag that maximizes correlation between the two signals, "
    "starting from the current offset. Check the reported peak correlation: a "
    "low value means the motion was too weak to align on.");
  status_text(align_status_, align_status_is_error_);
}

void App::draw_save_panel()
{
  ImGui::SeparatorText("Save");

  ImGui::SetNextItemWidth(-260.0f);
  ImGui::InputTextWithHint("##save_path", "output YAML path", &save_path_);
  ImGui::SameLine();
  if (ImGui::Button("Save offset")) {
    SyncConfig cfg;
    cfg.bag_a_uri = a_.data.uri.empty() ? a_.uri_input : a_.data.uri;
    cfg.bag_a_topic = a_.data.series.topic;
    cfg.bag_b_uri = b_.data.uri.empty() ? b_.uri_input : b_.data.uri;
    cfg.bag_b_topic = b_.data.series.topic;
    cfg.timestamp_source = source_;
    cfg.time_offset_seconds = offset_;
    try {
      save_sync_config(save_path_, cfg);
      save_status_ = "saved offset " + format_seconds(offset_) + " s to " + save_path_;
      save_status_is_error_ = false;
    } catch (const std::exception & e) {
      save_status_ = e.what();
      save_status_is_error_ = true;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Load config")) {
    preload_config(save_path_);
  }

  ImGui::SetNextItemWidth(200.0f);
  int source_index = (source_ == TimestampSource::BagReceive) ? 1 : 0;
  ImGui::BeginDisabled(aligning_);
  if (ImGui::Combo("timestamps", &source_index, "header stamp\0bag receive time\0")) {
    source_ = (source_index == 1) ? TimestampSource::BagReceive : TimestampSource::HeaderStamp;
    // The loaded series were built from the previous source, so they no
    // longer match what the UI claims; reload whatever is already selected.
    for (int i = 0; i < 2; ++i) {
      if (slot_ref(i).topic_index >= 0) {
        start_load(i);
      }
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  help_marker(
    "Which clock to take sample times from. Header stamps come from the "
    "sensor; bag receive times come from the recording machine and include "
    "transport delay. Changing this reloads both bags.");

  status_text(save_status_, save_status_is_error_);
}

void App::draw_export_panel()
{
  ImGui::SeparatorText("Export bag B with the offset applied");

  if (!export_path_edited_ && export_path_.empty()) {
    export_path_ = default_export_path(b_.data.uri.empty() ? b_.uri_input : b_.data.uri);
  }

  ImGui::SetNextItemWidth(-260.0f);
  if (ImGui::InputTextWithHint("##export_path", "output bag path", &export_path_)) {
    export_path_edited_ = true;
  }
  ImGui::SameLine();
  const bool can_export = !exporting_ && !a_.loading && !b_.loading && !aligning_ &&
    !b_.data.uri.empty() && !export_path_.empty();
  ImGui::BeginDisabled(!can_export);
  if (ImGui::Button("Export MCAP")) {
    start_export();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!exporting_);
  if (ImGui::Button("Cancel")) {
    export_progress_.cancel.store(true, std::memory_order_relaxed);
  }
  ImGui::EndDisabled();

  ImGui::Checkbox("shift header stamps too", &export_opts_.shift_header_stamps);
  ImGui::SameLine();
  help_marker(
    "Every message's bag timestamp is shifted. With this on, the stamp inside "
    "the message header is shifted as well, for any type that starts with a "
    "std_msgs/Header. Leave it on unless something downstream depends on the "
    "original header stamps: most tools read the header, so an export without "
    "it would still look unsynchronized.");

  if (exporting_) {
    const auto written = export_progress_.written.load(std::memory_order_relaxed);
    const auto total = export_progress_.total.load(std::memory_order_relaxed);
    const float fraction = total > 0 ?
      static_cast<float>(written) / static_cast<float>(total) : 0.0f;
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%zu / %zu messages", written, total);
    ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), overlay);
  }
  status_text(export_status_, export_status_is_error_);
}

void App::start_export()
{
  if (exporting_ || b_.data.uri.empty() || export_path_.empty()) {
    return;
  }
  // rosbag2 refuses to write into an existing bag, and quietly removing one
  // the user may still need is not this tool's call to make.
  std::error_code ec;
  if (std::filesystem::exists(export_path_, ec)) {
    export_status_ = "'" + export_path_ + "' already exists; choose another path";
    export_status_is_error_ = true;
    return;
  }

  export_progress_.cancel.store(false, std::memory_order_relaxed);
  export_progress_.written.store(0, std::memory_order_relaxed);
  export_progress_.total.store(0, std::memory_order_relaxed);
  exporting_ = true;
  export_status_ = "exporting...";
  export_status_is_error_ = false;

  const std::string src = b_.data.uri;
  const std::string dst = export_path_;
  const double offset = offset_;
  export_offset_ = offset_;   // the slider may move before the export finishes
  const ExportOptions opts = export_opts_;
  export_future_ = std::async(
    std::launch::async, [src, dst, offset, opts, progress = &export_progress_]() {
      return export_shifted_bag(src, dst, offset, opts, progress);
    });
}

void App::poll_export()
{
  if (!exporting_ || !export_future_.valid()) {
    return;
  }
  if (export_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }
  exporting_ = false;
  try {
    const ExportResult r = export_future_.get();
    std::ostringstream os;
    if (r.cancelled) {
      os << "cancelled after " << r.messages_written << " messages; '" << r.destination
         << "' is incomplete";
      export_status_is_error_ = true;
    } else {
      os << "wrote " << r.messages_written << " messages to " << r.destination
         << " (offset " << format_seconds(export_offset_) << " s)";
      if (export_opts_.shift_header_stamps) {
        os << "; " << r.header_stamps_shifted << " header stamps shifted";
        if (!r.stampless_topics.empty()) {
          os << ", " << r.stampless_topics.size() << " topic(s) have no header";
        }
        if (r.zero_stamps_kept > 0) {
          os << "; " << r.zero_stamps_kept << " unset stamps left at zero";
        }
        if (r.clamped_stamps > 0) {
          os << "; " << r.clamped_stamps << " stamps clamped to zero";
        }
      }
      export_status_is_error_ = false;
    }
    export_status_ = os.str();
  } catch (const std::exception & e) {
    export_status_ = e.what();
    export_status_is_error_ = true;
  }
}

void App::draw_plots()
{
  rebuild_x();

  ImGui::TextUnformatted("gyro axes:");
  for (int i = 0; i < 4; ++i) {
    ImGui::SameLine();
    ImGui::Checkbox(kAxisNames[i], &show_gyro_axis_[i]);
  }
  ImGui::SameLine();
  ImGui::Spacing();
  ImGui::SameLine();
  ImGui::TextUnformatted("| accel axes:");
  ImGui::PushID("accel");
  for (int i = 0; i < 4; ++i) {
    ImGui::SameLine();
    ImGui::Checkbox(kAxisNames[i], &show_accel_axis_[i]);
  }
  ImGui::PopID();
  ImGui::SameLine();
  ImGui::Spacing();
  ImGui::SameLine();
  ImGui::Checkbox("decimate", &decimate_);
  ImGui::SameLine();
  ImGui::Checkbox("auto-fit Y", &autofit_y_);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) {
    x_links_valid_ = false;   // recompute the x range from the data next frame
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(140.0f);
  ImGui::DragInt("max points/line", &max_plot_points_, 500.0f, 1000, 500000);
  ImGui::SameLine();
  help_marker(
    "Plots at most this many points per trace by subsampling. Turn it off to "
    "inspect every sample once you are zoomed in on the alignment feature.");

  if (a_.data.series.empty() && b_.data.series.empty()) {
    ImGui::TextDisabled("Load a bag to see its IMU data.");
    return;
  }

  const float avail = ImGui::GetContentRegionAvail().y;
  const ImVec2 plot_size(-1.0f, std::max(120.0f, (avail - 8.0f) * 0.5f));

  struct PlotSpec
  {
    const char * title;
    const char * y_label;
    const bool * show;
    const std::array<std::vector<double>, 4> ImuSeries::* member;
  };
  const PlotSpec specs[2] = {
    {"Angular velocity", "rad/s", show_gyro_axis_, &ImuSeries::gyro},
    {"Linear acceleration", "m/s^2", show_accel_axis_, &ImuSeries::accel},
  };

  for (int p = 0; p < 2; ++p) {
    const PlotSpec & spec = specs[p];
    if (!ImPlot::BeginPlot(spec.title, plot_size)) {
      continue;
    }
    // Y is fitted to whatever is in view by default: the two plots hold
    // quantities of very different magnitude (rad/s against ~9.81 m/s^2), so a
    // shared fixed range would push one of them off-screen.
    ImPlot::SetupAxes(
      "time since reference [s]", spec.y_label,
      ImPlotAxisFlags_None, autofit_y_ ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None);
    // Both plots share one x range so panning either keeps them comparable.
    ImPlot::SetupAxisLinks(ImAxis_X1, &x_link_min_, &x_link_max_);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Outside);

    for (int slot = 0; slot < 2; ++slot) {
      const BagSlot & s = (slot == 0) ? a_ : b_;
      if (s.data.series.empty()) {
        continue;
      }
      const auto & channels = s.data.series.*(spec.member);
      for (int axis = 0; axis < 4; ++axis) {
        if (!spec.show[axis]) {
          continue;
        }
        const std::string label =
          std::string(slot == 0 ? "A " : "B ") + kAxisNames[axis];
        plot_series(
          label, s.x, channels[axis], slot == 0 ? kColorA[axis] : kColorB[axis],
          decimate_, max_plot_points_);
      }
    }
    ImPlot::EndPlot();
  }
  x_links_valid_ = true;
}

void App::drop_callback(GLFWwindow * window, int count, const char ** paths)
{
  auto * self = static_cast<App *>(glfwGetWindowUserPointer(window));
  if (self != nullptr) {
    self->handle_drop(count, paths);
  }
}

void App::handle_drop(int count, const char ** paths)
{
  // Dropping several paths at once fills the slots in the order they arrive,
  // which is the same rule as dropping them one at a time.
  for (int i = 0; i < count; ++i) {
    const int slot = next_drop_slot_;
    slot_ref(slot).uri_input = normalize_bag_path(paths[i]);
    pending_scan_[slot] = true;
    next_drop_slot_ = (next_drop_slot_ + 1) % 2;
  }
}

void App::scan(int slot)
{
  BagSlot & s = slot_ref(slot);
  s.data.imu_topics.clear();
  s.topic_index = -1;

  if (s.uri_input.empty()) {
    s.status = "no bag path given";
    s.status_is_error = true;
    return;
  }
  try {
    s.data.imu_topics = find_imu_topics(s.uri_input);
  } catch (const std::exception & e) {
    s.status = e.what();
    s.status_is_error = true;
    return;
  }
  if (s.data.imu_topics.empty()) {
    s.status = "no sensor_msgs/msg/Imu topic in this bag";
    s.status_is_error = true;
    return;
  }
  s.topic_index = 0;
  s.status = "found " + std::to_string(s.data.imu_topics.size()) + " IMU topic(s)";
  s.status_is_error = false;
}

void App::start_load(int slot)
{
  BagSlot & s = slot_ref(slot);
  if (s.loading || aligning_ || s.topic_index < 0 ||
    s.topic_index >= static_cast<int>(s.data.imu_topics.size()))
  {
    return;
  }
  const std::string uri = s.uri_input;
  const std::string topic = s.data.imu_topics[s.topic_index];
  const TimestampSource source = source_;

  s.load_progress.store(0, std::memory_order_relaxed);
  s.loading = true;
  s.status.clear();
  s.load_future = std::async(
    std::launch::async,
    [uri, topic, source, progress = &s.load_progress]() {
      return load_imu_series(uri, topic, source, progress);
    });
}

void App::poll_loads()
{
  for (int i = 0; i < 2; ++i) {
    BagSlot & s = slot_ref(i);
    if (!s.loading || !s.load_future.valid()) {
      continue;
    }
    if (s.load_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      continue;
    }
    s.loading = false;
    try {
      s.data.series = s.load_future.get();
      s.data.uri = s.uri_input;
      if (s.data.series.empty()) {
        s.status = "topic has no usable samples";
        s.status_is_error = true;
      } else {
        s.status = "loaded";
        s.status_is_error = false;
      }
      if (i == 1 && !export_path_edited_) {
        export_path_ = default_export_path(s.data.uri);
      }
      have_t_ref_ = false;   // force the display frame to be recomputed
      x_links_valid_ = false;
    } catch (const std::exception & e) {
      s.data.series = ImuSeries{};
      s.status = e.what();
      s.status_is_error = true;
    }
  }
}

void App::poll_align()
{
  if (!aligning_ || !align_future_.valid()) {
    return;
  }
  if (align_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }
  aligning_ = false;
  try {
    const AlignResult r = align_future_.get();
    align_status_ = r.message;
    align_status_is_error_ = !r.ok;
    if (r.ok) {
      offset_ = r.offset_seconds;
      align_status_ = "offset " + format_seconds(r.offset_seconds) +
        " s, peak correlation " + format_seconds(r.correlation) +
        ", grid " + format_seconds(r.grid_dt) + " s";
    }
  } catch (const std::exception & e) {
    align_status_ = e.what();
    align_status_is_error_ = true;
  }
}

void App::run_auto_align()
{
  if (aligning_ || a_.loading || b_.loading || !both_loaded()) {
    return;
  }
  aligning_ = true;
  align_status_ = "estimating...";
  align_status_is_error_ = false;
  // The series are not touched while a load is not running, so passing
  // references into the worker is safe for the lifetime of the estimate.
  const ImuSeries & sa = a_.data.series;
  const ImuSeries & sb = b_.data.series;
  const double init = offset_;
  const AlignOptions opts = align_opts_;
  align_future_ = std::async(
    std::launch::async, [&sa, &sb, init, opts]() {
      return estimate_offset(sa, sb, init, opts);
    });
}

void App::rebuild_x()
{
  if (!have_t_ref_) {
    if (!a_.data.series.empty()) {
      t_ref_ = a_.data.series.t_begin();
      have_t_ref_ = true;
    } else if (!b_.data.series.empty()) {
      t_ref_ = b_.data.series.t_begin() + offset_;
      have_t_ref_ = true;
    }
    // A new reference invalidates both cached x vectors.
    a_.x.clear();
    b_.x.clear();
  }

  const bool offset_changed = (offset_ != last_offset_);
  last_offset_ = offset_;

  const auto & ta = a_.data.series.t;
  if (a_.x.size() != ta.size()) {
    a_.x.resize(ta.size());
    for (std::size_t i = 0; i < ta.size(); ++i) {
      a_.x[i] = ta[i] - t_ref_;
    }
  }

  const auto & tb = b_.data.series.t;
  if (offset_changed || b_.x.size() != tb.size()) {
    b_.x.resize(tb.size());
    const double shift = offset_ - t_ref_;
    for (std::size_t i = 0; i < tb.size(); ++i) {
      b_.x[i] = tb[i] + shift;
    }
  }

  if (!x_links_valid_) {
    double lo = 0.0;
    double hi = 1.0;
    if (!a_.x.empty() && !b_.x.empty()) {
      lo = std::min(a_.x.front(), b_.x.front());
      hi = std::max(a_.x.back(), b_.x.back());
    } else if (!a_.x.empty()) {
      lo = a_.x.front();
      hi = a_.x.back();
    } else if (!b_.x.empty()) {
      lo = b_.x.front();
      hi = b_.x.back();
    }
    if (hi <= lo) {
      hi = lo + 1.0;
    }
    const double pad = 0.02 * (hi - lo);
    x_link_min_ = lo - pad;
    x_link_max_ = hi + pad;
  }
}

void App::handle_shortcuts()
{
  ImGuiIO & io = ImGui::GetIO();
  if (io.WantTextInput) {
    return;
  }
  const double step = io.KeyShift ? nudge_coarse_ : nudge_fine_;
  if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true)) {
    offset_ -= step;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) {
    offset_ += step;
  }
}

}  // namespace bms
