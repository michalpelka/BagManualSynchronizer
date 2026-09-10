#pragma once

#include <atomic>
#include <cstddef>
#include <future>
#include <string>
#include <vector>

#include "align.hpp"
#include "bag_exporter.hpp"
#include "bag_loader.hpp"
#include "imu_series.hpp"
#include "sync_config.hpp"

struct GLFWwindow;

namespace bms
{

/// One side of the comparison: the bag, its UI state, and its in-flight load.
struct BagSlot
{
  BagData data;
  std::string uri_input;
  int topic_index = -1;
  std::string status;
  bool status_is_error = false;

  std::future<ImuSeries> load_future;
  std::atomic<std::size_t> load_progress{0};
  bool loading = false;

  /// Plot x values: series time shifted into the display frame. Rebuilt
  /// whenever the offset or the time reference changes.
  std::vector<double> x;
};

class App
{
public:
  App();
  ~App();

  App(const App &) = delete;
  App & operator=(const App &) = delete;

  /// Queue a bag to be scanned and loaded as soon as the UI comes up.
  void preload(int slot, const std::string & uri);
  /// Restore paths, topics and offset from a previously saved config file.
  void preload_config(const std::string & path);

  /// Run the event loop until the window is closed. Returns the exit code.
  int run();

private:
  void draw();
  void draw_bag_panel(int slot);
  void draw_offset_controls();
  void draw_save_panel();
  void draw_export_panel();
  void draw_plots();

  /// GLFW hands dropped paths here; each drop fills the next bag slot.
  static void drop_callback(GLFWwindow * window, int count, const char ** paths);
  void handle_drop(int count, const char ** paths);

  void scan(int slot);
  void start_load(int slot);
  void poll_loads();
  void rebuild_x();
  void handle_shortcuts();
  void run_auto_align();
  void poll_align();
  void poll_export();
  void start_export();

  BagSlot & slot_ref(int slot) {return slot == 0 ? a_ : b_;}
  bool both_loaded() const {return !a_.data.series.empty() && !b_.data.series.empty();}

  GLFWwindow * window_ = nullptr;

  BagSlot a_;
  BagSlot b_;

  TimestampSource source_ = TimestampSource::HeaderStamp;
  double offset_ = 0.0;
  double last_offset_ = 0.0;
  double t_ref_ = 0.0;
  bool have_t_ref_ = false;

  // Offset editing
  double slider_range_ = 10.0;
  double nudge_fine_ = 0.001;
  double nudge_coarse_ = 0.1;

  // Auto-alignment
  AlignOptions align_opts_;
  std::future<AlignResult> align_future_;
  bool aligning_ = false;
  std::string align_status_;
  bool align_status_is_error_ = false;

  // Plot options
  bool show_gyro_axis_[4] = {false, false, false, true};
  bool show_accel_axis_[4] = {false, false, false, true};
  int max_plot_points_ = 20000;
  bool decimate_ = true;
  bool autofit_y_ = true;
  double x_link_min_ = 0.0;
  double x_link_max_ = 1.0;
  bool x_links_valid_ = false;

  // Exporting bag B with the offset baked in
  ExportOptions export_opts_;
  ExportProgress export_progress_;
  std::future<ExportResult> export_future_;
  bool exporting_ = false;
  /// Cleared whenever bag B changes so the default path follows the new bag.
  bool export_path_edited_ = false;
  std::string export_path_;
  /// The offset actually baked into the running export, for the status line.
  double export_offset_ = 0.0;
  std::string export_status_;
  bool export_status_is_error_ = false;

  // Saving
  std::string save_path_ = "sync_offset.yaml";
  std::string save_status_;
  bool save_status_is_error_ = false;

  // Deferred startup actions
  std::string pending_config_;
  bool pending_scan_[2] = {false, false};

  /// Which bag the next dropped path fills: 0 = A, 1 = B, then back to A.
  int next_drop_slot_ = 0;
};

}  // namespace bms
