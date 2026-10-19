#include "capture_app.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uscapture {
namespace {

void check_text(const std::string& value, const char* field) {
    if (value.size() > 32768 || value.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string(field) + " contains invalid text");
}

bool ipv4_address(const std::string& host) {
    std::size_t begin = 0;
    for (int part = 0; part < 4; ++part) {
        const auto separator = host.find('.', begin);
        if ((part < 3) != (separator != std::string::npos)) return false;
        const auto end = separator == std::string::npos ? host.size() : separator;
        const auto length = end - begin;
        if (length == 0 || length > 3 || (length > 1 && host[begin] == '0')) return false;
        unsigned value = 0;
        const auto parsed = std::from_chars(host.data() + begin, host.data() + end, value);
        if (parsed.ec != std::errc{} || parsed.ptr != host.data() + end || value > 255) return false;
        begin = end + 1;
    }
    return true;
}

std::size_t packed_size(int width, int height, std::size_t channels) {
    if (width <= 0 || height <= 0)
        throw std::invalid_argument("Frame dimensions must be positive");
    const auto w = static_cast<std::size_t>(width);
    const auto h = static_cast<std::size_t>(height);
    if (w > std::numeric_limits<std::size_t>::max() / channels ||
        h > std::numeric_limits<std::size_t>::max() / (w * channels))
        throw std::invalid_argument("Frame byte count overflows");
    return w * h * channels;
}

void check_scale(const std::optional<RulerScale>& scale, int axis_size, int orthogonal_size) {
    if (!scale) return;
    scale->mm_per_pixel();
    if (axis_size <= 0 || scale->pixel_a >= axis_size || scale->pixel_b >= axis_size)
        throw std::invalid_argument("Ruler endpoints must be inside the reference image");
    const bool no_location = scale->orthogonal_a == -1 && scale->orthogonal_b == -1;
    if (!no_location && (scale->orthogonal_a < 0 || scale->orthogonal_b < 0 ||
                        scale->orthogonal_a >= orthogonal_size || scale->orthogonal_b >= orthogonal_size))
        throw std::invalid_argument("Both ruler inspection positions must be inside the reference image");
}

} // namespace

double RulerScale::mm_per_pixel() const {
    if (pixel_a < 0 || pixel_b < 0 || pixel_a == pixel_b ||
        !std::isfinite(distance_mm) || distance_mm <= 0)
        throw std::invalid_argument("Ruler needs distinct nonnegative pixels and a positive finite distance");
    const auto delta = static_cast<std::int64_t>(pixel_b) - pixel_a;
    const double value = distance_mm / static_cast<double>(delta < 0 ? -delta : delta);
    if (!std::isfinite(value) || value <= 0 || value > 1000)
        throw std::invalid_argument("Ruler scale is outside the supported numeric range");
    return value;
}

bool valid_rect(const Rect& roi, int width, int height) {
    return width > 0 && height > 0 && roi.x >= 0 && roi.y >= 0 &&
           roi.width > 0 && roi.height > 0 && roi.x < width && roi.y < height &&
           roi.width <= width - roi.x && roi.height <= height - roi.y;
}

bool matches(const Profile& profile, const PreviewFrame& frame) {
    return profile.roi_confirmed && profile.reference_dpi > 0 &&
           profile.reference_width == frame.width && profile.reference_height == frame.height &&
           profile.reference_dpi == frame.dpi && valid_rect(profile.roi, frame.width, frame.height);
}

void validate_profile(const Profile& profile, bool require_roi) {
    check_text(profile.executable, "executable");
    check_text(profile.working_directory, "working_directory");
    check_text(profile.process_name, "process_name");
    check_text(profile.title_contains, "title_contains");
    check_text(profile.host, "host");
    check_text(profile.imaging_context, "imaging_context");
    check_text(profile.revision, "revision");
    if (profile.imaging_context.size() > 256 || profile.revision.size() > 256)
        throw std::invalid_argument("Imaging context and revision must each fit in 256 UTF-8 bytes");
    if (profile.host != "localhost" && !ipv4_address(profile.host))
        throw std::invalid_argument("Receiver host must be localhost or an IPv4 address without leading zeroes");
    if (profile.port <= 0 || profile.port > 65535)
        throw std::invalid_argument("Receiver port must be between 1 and 65535");
    if (require_roi && profile.process_name.empty() && profile.title_contains.empty())
        throw std::invalid_argument("Select a vendor process or window title before connecting");
    if (profile.auto_connect && !profile.roi_confirmed)
        throw std::invalid_argument("Automatic connection requires a confirmed ROI");
    if (require_roi && !profile.roi_confirmed)
        throw std::invalid_argument("Confirm the ROI before connecting");

    const bool no_reference = profile.reference_width == 0 && profile.reference_height == 0 &&
                              profile.reference_dpi == 0;
    const bool no_roi = profile.roi.x == 0 && profile.roi.y == 0 &&
                        profile.roi.width == 0 && profile.roi.height == 0;
    if (no_reference) {
        if (profile.roi_confirmed || !no_roi || profile.horizontal_scale || profile.vertical_scale)
            throw std::invalid_argument("ROI or ruler data requires reference dimensions and DPI");
    } else {
        packed_size(profile.reference_width, profile.reference_height, 4);
        if (profile.reference_dpi == 0)
            throw std::invalid_argument("Reference DPI must be positive");
        if ((profile.roi_confirmed || !no_roi) &&
            !valid_rect(profile.roi, profile.reference_width, profile.reference_height))
            throw std::invalid_argument("ROI must fit inside the reference image");
    }
    check_scale(profile.horizontal_scale, profile.reference_width, profile.reference_height);
    check_scale(profile.vertical_scale, profile.reference_height, profile.reference_width);
}

std::vector<std::uint8_t> crop_mono8(const PreviewFrame& frame, const Rect& roi) {
    const auto expected = packed_size(frame.width, frame.height, 4);
    if (frame.bgra.size() != expected)
        throw std::invalid_argument("Expected packed BGRA rows with exactly width * height * 4 bytes");
    if (!valid_rect(roi, frame.width, frame.height))
        throw std::invalid_argument("Crop ROI lies outside the image");
    std::vector<std::uint8_t> output(packed_size(roi.width, roi.height, 1));
    for (int y = 0; y < roi.height; ++y) {
        const auto source = (static_cast<std::size_t>(roi.y + y) * frame.width + roi.x) * 4;
        const auto destination = static_cast<std::size_t>(y) * roi.width;
        for (int x = 0; x < roi.width; ++x) {
            const auto index = source + static_cast<std::size_t>(x) * 4;
            // Integer BT.709 weights sum to one, preserving every neutral gray exactly.
            const unsigned value = 722u * frame.bgra[index] + 7152u * frame.bgra[index + 1] +
                                   2126u * frame.bgra[index + 2];
            output[destination + x] = static_cast<std::uint8_t>(value / 10000u);
        }
    }
    return output;
}

} // namespace uscapture
