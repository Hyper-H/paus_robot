#include "capture_app.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winrt/base.h>
#include <fstream>
#include <iterator>
#endif

namespace {
void check(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}
template <typename Function> void rejects(Function function, const char* description) {
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error(description);
}

uscapture::Profile confirmed_profile() {
    uscapture::Profile p;
    p.process_name = "Ultra-DAQ.exe";
    p.reference_width = 1420;
    p.reference_height = 810;
    p.reference_dpi = 96;
    p.roi = {611, 97, 526, 510};
    p.roi_confirmed = true;
    return p;
}

#ifdef _WIN32
void check_windows_profiles() {
    using namespace uscapture;
    const auto directory = std::filesystem::temp_directory_path() /
        (L"uscapture-profile-test-" + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(GetTickCount64()));
    check(std::filesystem::create_directory(directory), "Cannot reserve test profile directory");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    } cleanup{directory};
    const auto path = directory / L"\u8d85\u58f0\u914d\u7f6e.json";
    auto p = confirmed_profile();
    p.executable = u8"C:\\Program Files\\\u8d85\u58f0\\Ultra-DAQ.exe";
    p.working_directory = u8"C:\\Program Files\\\u8d85\u58f0";
    p.title_contains = u8"\u8d85\u58f0";
    p.vertical_scale = RulerScale{100, 300, 10, 1148, 1148};
    p.imaging_context = "depth=40mm; zoom=1";
    p.revision = "first";
    p.auto_connect = true;
    p.minimize_to_tray = false;
    save_profile(path, p);
    auto loaded = load_profile(path);
    check(loaded.executable == p.executable && loaded.working_directory == p.working_directory &&
          loaded.title_contains == p.title_contains, "Unicode profile path or text failed to roundtrip");
    check(loaded.roi.x == p.roi.x && loaded.roi.width == p.roi.width && loaded.reference_dpi == 96 &&
          loaded.roi_confirmed && loaded.auto_connect, "ROI confirmation did not roundtrip");
    check(!loaded.minimize_to_tray, "Close-to-tray preference did not roundtrip");
    check(loaded.vertical_scale && loaded.vertical_scale->pixel_a == 100 &&
          loaded.vertical_scale->pixel_b == 300 && loaded.vertical_scale->distance_mm == 10 &&
          !loaded.horizontal_scale, "Ruler evidence or independent axes did not roundtrip");
    check(loaded.vertical_scale->orthogonal_a == 1148 && loaded.vertical_scale->orthogonal_b == 1148,
          "Ruler inspection locations did not roundtrip");
    p.revision = "second";
    save_profile(path, p);
    check(load_profile(path).revision == "second", "Atomic profile replacement failed");
    save_profile(directory / "draft.json", Profile{});
    check(!load_profile(directory / "draft.json").roi_confirmed, "Initial draft did not roundtrip");

    const auto write_text = [&](const std::string& text) {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << text;
        check(static_cast<bool>(output), "Cannot write profile test fixture");
    };
    const auto read_text = [&] {
        std::ifstream input(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    };
    const auto rejects_any = [](auto function, const char* description) {
        try { function(); } catch (...) { return; }
        throw std::runtime_error(description);
    };
    write_text(R"({"schema_version":1})");
    check(load_profile(path).minimize_to_tray, "Legacy profiles must preserve close-to-tray behavior");
    write_text(R"({"schema_version":1,"reference_width":1420,"reference_height":810,"reference_dpi":96,"vertical_scale":{"pixel_a":100,"pixel_b":300,"distance_mm":10}})");
    loaded = load_profile(path);
    check(loaded.vertical_scale && loaded.vertical_scale->orthogonal_a == -1 &&
          loaded.vertical_scale->orthogonal_b == -1, "Legacy ruler positions must remain unknown");
    for (const auto& text : {
        std::string("{broken"),
        std::string(R"({"schema_version":2})"),
        std::string(R"({"schema_version":1,"port":55001.5})"),
        std::string(R"({"schema_version":1,"reference_dpi":-1})"),
        std::string(R"({"schema_version":1,"auto_connect":true})"),
        std::string(R"({"schema_version":1,"minimize_to_tray":"false"})"),
        std::string(R"({"schema_version":1,"port":2147483648})"),
        std::string(R"({"schema_version":1,"host":"localhost)" "\xff" R"("})")}) {
        write_text(text);
        rejects_any([&] { load_profile(path); }, "Invalid persisted profile accepted");
        rejects_any([&] { save_profile(path, p); }, "Invalid persisted profile was silently overwritten");
        check(read_text() == text, "Rejected save changed the existing file");
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory))
        check(entry.path().filename().wstring().find(L".tmp.") == std::wstring::npos,
              "A profile transaction leaked a temporary file");
    std::cout << "profile_test: Unicode, draft, atomic replacement and corruption checks passed\n";
}
#endif
}

int main() {
    try {
#ifdef _WIN32
        winrt::init_apartment(winrt::apartment_type::multi_threaded);
        check_windows_profiles();
#endif
        using namespace uscapture;
        check(valid_rect({0, 0, 8, 4}, 8, 4), "Full frame must be a valid ROI");
        check(!valid_rect({7, 0, 2, 4}, 8, 4), "Overflowing ROI accepted");
        check(!valid_rect({0, 0, 0, 4}, 8, 4), "Empty ROI accepted");
        check(!valid_rect({-1, 0, 2, 4}, 8, 4), "Negative ROI accepted");
        const int maximum = std::numeric_limits<int>::max();
        check(!valid_rect({maximum - 2, 0, 10, 1}, maximum, 1), "Integer overflow bypassed ROI bounds");
        check(valid_rect({maximum - 2, 0, 2, 1}, maximum, 1), "Large valid ROI rejected");

        check(std::abs(RulerScale{100, 300, 10}.mm_per_pixel() - 0.05) < 1e-12, "Incorrect ruler scale");
        check(RulerScale{300, 100, 10}.mm_per_pixel() == RulerScale{100, 300, 10}.mm_per_pixel(),
              "Reversed ruler must yield the same positive scale");
        rejects([] { RulerScale{1, 1, 10}.mm_per_pixel(); }, "Coincident ruler endpoints accepted");
        rejects([] { RulerScale{0, 1, 0}.mm_per_pixel(); }, "Zero ruler distance accepted");
        rejects([] { RulerScale{-1, 1, 10}.mm_per_pixel(); }, "Negative ruler endpoint accepted");
        rejects([] { RulerScale{0, 1, std::numeric_limits<double>::infinity()}.mm_per_pixel(); }, "Infinite scale accepted");
        rejects([] { RulerScale{0, 1, std::numeric_limits<double>::quiet_NaN()}.mm_per_pixel(); }, "NaN scale accepted");
        check(RulerScale{0, 1, 1000}.mm_per_pixel() == 1000, "Protocol maximum ruler scale rejected");
        rejects([] { RulerScale{0, 1, 1000.01}.mm_per_pixel(); }, "Ruler scale exceeds receiver protocol limit");

        Profile draft;
        validate_profile(draft, false);
        rejects([&] { validate_profile(draft, true); }, "Unconfirmed draft connected");
        draft.auto_connect = true;
        rejects([&] { validate_profile(draft, false); }, "Unconfirmed draft auto-connect accepted");
        auto p = confirmed_profile();
        validate_profile(p, true);
        PreviewFrame frame;
        frame.width = 1420; frame.height = 810; frame.dpi = 96;
        check(matches(p, frame), "Confirmed reference frame did not match");
        frame.width = 1421;
        check(!matches(p, frame), "Resized frame retained confirmation");
        frame.width = 1420; frame.dpi = 120;
        check(!matches(p, frame), "DPI change retained confirmation");
        frame.dpi = 96; p.roi_confirmed = false;
        check(!matches(p, frame), "Unconfirmed ROI matched");
        p = confirmed_profile();
        p.vertical_scale = RulerScale{100, 300, 10};
        validate_profile(p, true);
        check(!p.horizontal_scale, "Vertical ruler unexpectedly set horizontal scale");
        p.vertical_scale = RulerScale{100, 300, 10, 1419, 1419};
        validate_profile(p, true);
        p.vertical_scale->orthogonal_a = 1420;
        rejects([&] { validate_profile(p, false); }, "Out-of-frame orthogonal ruler position accepted");
        p.vertical_scale = RulerScale{100, 300, 10, 200, -1};
        rejects([&] { validate_profile(p, false); }, "Incomplete ruler inspection location accepted");
        p.vertical_scale = RulerScale{100, 300, 10, -2, -2};
        rejects([&] { validate_profile(p, false); }, "Invalid unknown ruler inspection sentinel accepted");
        p.vertical_scale.reset();
        p.horizontal_scale = RulerScale{100, 300, 10, 809, 809};
        validate_profile(p, true);
        p.horizontal_scale->orthogonal_b = 810;
        rejects([&] { validate_profile(p, false); }, "Horizontal inspection position used the wrong frame axis");
        p.horizontal_scale.reset();
        p.vertical_scale = RulerScale{0, 810, 10};
        rejects([&] { validate_profile(p, false); }, "Out-of-frame ruler accepted");
        p = confirmed_profile(); p.port = 65536;
        rejects([&] { validate_profile(p, false); }, "Invalid port accepted");
        p.port = 55001; p.reference_height = -1;
        rejects([&] { validate_profile(p, false); }, "Negative reference dimension accepted");
        p = confirmed_profile(); p.reference_dpi = 0;
        rejects([&] { validate_profile(p, false); }, "Missing DPI accepted");
        p = confirmed_profile(); p.roi.width = maximum;
        rejects([&] { validate_profile(p, false); }, "Overflowing profile ROI accepted");
        p = confirmed_profile(); p.host = "http://127.0.0.1";
        rejects([&] { validate_profile(p, false); }, "URL accepted as receiver hostname");
        for (const auto* host : {"localhost", "127.0.0.1", "172.16.33.132", "0.0.0.0", "255.255.255.255"}) {
            p = confirmed_profile(); p.host = host;
            validate_profile(p, false);
        }
        for (const auto* host : {"", "lab-host", "::1", "1.2.3", "1.2.3.4.5", "1.2.3.",
                                 "1..3.4", "01.2.3.4", "1.02.3.4", "1.2.3.004", "256.1.1.1",
                                 "-1.2.3.4", "+1.2.3.4", "1.2.3.4 ", " 1.2.3.4", "1.2.3.x"}) {
            p = confirmed_profile(); p.host = host;
            rejects([&] { validate_profile(p, false); }, "Invalid or unsupported receiver address accepted");
        }
        p = confirmed_profile(); p.imaging_context = std::string(256, 'x'); p.revision = std::string(256, 'r');
        validate_profile(p, false);
        p.imaging_context.push_back('x');
        rejects([&] { validate_profile(p, false); }, "Oversized imaging context accepted");
        p = confirmed_profile(); p.revision = std::string(257, 'r');
        rejects([&] { validate_profile(p, false); }, "Oversized revision accepted");
        p = confirmed_profile();
        for (int i = 0; i < 128; ++i) p.imaging_context += "\xc2\xb5";
        validate_profile(p, false);
        p.imaging_context += "\xc2\xb5";
        rejects([&] { validate_profile(p, false); }, "UTF-8 byte limit was treated as a character count");
        p = confirmed_profile(); p.executable = std::string(1024, 'x'); p.working_directory = std::string(1024, 'd');
        validate_profile(p, false);

        PreviewFrame pixels;
        pixels.width = 3; pixels.height = 2;
        pixels.bgra = {0, 0, 0, 255, 255, 255, 255, 0, 80, 80, 80, 31,
                       255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255};
        check(crop_mono8(pixels, {0, 0, 3, 2}) == std::vector<std::uint8_t>({0, 255, 80, 18, 182, 54}),
              "BGRA conversion, channel order, or alpha handling incorrect");
        check(crop_mono8(pixels, {1, 0, 2, 2}) == std::vector<std::uint8_t>({255, 80, 182, 54}),
              "Crop row offset incorrect");
        pixels.bgra.push_back(0);
        rejects([&] { crop_mono8(pixels, {0, 0, 3, 2}); }, "Padded or extra bytes accepted as packed input");
        pixels.bgra.resize(23);
        rejects([&] { crop_mono8(pixels, {0, 0, 3, 2}); }, "Truncated BGRA accepted");
        pixels.bgra.resize(24);
        rejects([&] { crop_mono8(pixels, {2, 0, 2, 2}); }, "Out-of-bounds crop accepted");
        pixels.width = maximum; pixels.height = maximum;
        rejects([&] { crop_mono8(pixels, {0, 0, 1, 1}); }, "Huge mismatched frame accepted");

        PreviewFrame ramp;
        ramp.width = 256; ramp.height = 1;
        for (unsigned gray = 0; gray < 256; ++gray)
            ramp.bgra.insert(ramp.bgra.end(), {static_cast<std::uint8_t>(gray), static_cast<std::uint8_t>(gray),
                                             static_cast<std::uint8_t>(gray), 0});
        const auto grayscale = crop_mono8(ramp, {0, 0, 256, 1});
        for (unsigned gray = 0; gray < 256; ++gray)
            check(grayscale[gray] == gray, "Neutral grayscale was changed");
        std::cout << "core_test: ROI, lifecycle, profile, ruler and BGRA checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "core_test: " << error.what() << '\n';
        return 1;
    }
}
