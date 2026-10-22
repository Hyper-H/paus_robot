#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace uscapture {

struct Rect {
    int x = 0, y = 0, width = 0, height = 0;
};

// A display-ruler measurement, not a probe-to-robot spatial calibration.
struct RulerScale {
    int pixel_a = 0, pixel_b = 0;
    double distance_mm = 0;
    // Optional other-axis positions restore the inspector; -1 means not recorded.
    int orthogonal_a = -1, orthogonal_b = -1;
    double mm_per_pixel() const;
};

struct Profile {
    std::string executable;
    std::string working_directory;
    std::string process_name;
    std::string title_contains;
    std::string host = "127.0.0.1";
    int port = 55001;
    Rect roi;
    int reference_width = 0, reference_height = 0;
    unsigned reference_dpi = 0;
    bool roi_confirmed = false;
    bool auto_connect = false;
    bool launch_vendor = true;
    bool minimize_to_tray = true;
    std::optional<RulerScale> horizontal_scale, vertical_scale;
    std::string imaging_context; // Operator's depth/zoom/preset identification.
    std::string revision;
};

struct PreviewFrame {
    int width = 0, height = 0;
    unsigned dpi = 0;
    std::uint64_t sequence = 0;
    double capture_time_s = 0;
    double wgc_surface_time_s = 0;
    std::vector<std::uint8_t> bgra; // Packed, top-down rows; no UI overlay added here.
};

struct Status {
    std::string state = "STOPPED";
    std::string message;
    bool transmitting = false;
    bool connected = false;
    bool roi_matches = false;
    int width = 0, height = 0;
    unsigned dpi = 0;
    std::uint64_t captured = 0, acknowledged = 0, dropped = 0;
    double frame_age_s = -1, last_ack_ms = 0;
};

bool valid_rect(const Rect& roi, int width, int height);
bool matches(const Profile& profile, const PreviewFrame& frame);
void validate_profile(const Profile& profile, bool require_roi);
std::vector<std::uint8_t> crop_mono8(const PreviewFrame& frame, const Rect& roi);
std::filesystem::path default_profile_path();
Profile load_profile(const std::filesystem::path& path);
void save_profile(const std::filesystem::path& path, const Profile& profile);

class CaptureController {
public:
    CaptureController();
    ~CaptureController();
    CaptureController(const CaptureController&) = delete;
    CaptureController& operator=(const CaptureController&) = delete;
    // Mutating methods are serialized by the desktop command worker.
    // preview() and status() may be polled concurrently by the UI thread.
    // Starts/reuses the vendor process only when requested, then watches its window.
    // Capturing a preview does not transmit; the caller explicitly enables sending.
    void start(const Profile& profile, bool launch_vendor);
    void apply_profile(const Profile& profile);
    void set_transmitting(bool enabled);
    void stop();
    std::shared_ptr<const PreviewFrame> preview() const;
    Status status() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uscapture
