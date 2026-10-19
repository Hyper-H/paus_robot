#include "capture_app.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shlobj.h>
#include <winrt/Windows.Data.Json.h>
#include <winrt/Windows.Foundation.Collections.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace uscapture {
namespace {
using namespace winrt::Windows::Data::Json;
constexpr int kSchemaVersion = 1;
constexpr std::uintmax_t kMaximumProfileBytes = 1024 * 1024;

winrt::hstring from_utf8(const std::string& text) {
    if (text.empty()) return {};
    if (text.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("UTF-8 text is too long");
    const auto bytes = static_cast<int>(text.size());
    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), bytes, nullptr, 0);
    if (!size) throw std::invalid_argument("Profile contains invalid UTF-8");
    std::wstring output(size, L'\0');
    if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), bytes, output.data(), size) != size)
        throw std::invalid_argument("Cannot decode UTF-8 profile");
    return winrt::hstring(output);
}

std::string to_utf8(const winrt::hstring& text) {
    if (text.empty()) return {};
    const int size = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(),
        static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    if (!size) throw std::invalid_argument("Profile contains invalid Unicode");
    std::string output(size, '\0');
    if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
        output.data(), size, nullptr, nullptr) != size)
        throw std::invalid_argument("Cannot encode Unicode profile");
    return output;
}

double number(const JsonObject& object, const wchar_t* name) {
    const double value = object.GetNamedNumber(name);
    if (!std::isfinite(value)) throw std::invalid_argument("Profile numbers must be finite");
    return value;
}

template <typename T> T integer(const JsonObject& object, const wchar_t* name) {
    const double value = number(object, name);
    if (std::floor(value) != value || value < static_cast<double>(std::numeric_limits<T>::lowest()) ||
        value > static_cast<double>(std::numeric_limits<T>::max()))
        throw std::invalid_argument("Profile integer is fractional or outside its allowed range");
    return static_cast<T>(value);
}

std::string string_or(const JsonObject& object, const wchar_t* name, const std::string& fallback = {}) {
    return object.HasKey(name) ? to_utf8(object.GetNamedString(name)) : fallback;
}

bool bool_or(const JsonObject& object, const wchar_t* name, bool fallback) {
    return object.HasKey(name) ? object.GetNamedBoolean(name) : fallback;
}

std::optional<RulerScale> read_scale(const JsonObject& object, const wchar_t* name) {
    if (!object.HasKey(name) || object.GetNamedValue(name).ValueType() == JsonValueType::Null)
        return std::nullopt;
    const auto scale = object.GetNamedObject(name);
    RulerScale result{integer<int>(scale, L"pixel_a"), integer<int>(scale, L"pixel_b"),
                      number(scale, L"distance_mm")};
    if (scale.HasKey(L"orthogonal_a")) result.orthogonal_a = integer<int>(scale, L"orthogonal_a");
    if (scale.HasKey(L"orthogonal_b")) result.orthogonal_b = integer<int>(scale, L"orthogonal_b");
    return result;
}

void put_string(const JsonObject& object, const wchar_t* name, const std::string& text) {
    object.Insert(name, JsonValue::CreateStringValue(from_utf8(text)));
}

void put_number(const JsonObject& object, const wchar_t* name, double value) {
    object.Insert(name, JsonValue::CreateNumberValue(value));
}

void put_bool(const JsonObject& object, const wchar_t* name, bool value) {
    object.Insert(name, JsonValue::CreateBooleanValue(value));
}

void put_scale(const JsonObject& object, const wchar_t* name, const std::optional<RulerScale>& scale) {
    if (!scale) return;
    JsonObject value;
    put_number(value, L"pixel_a", scale->pixel_a);
    put_number(value, L"pixel_b", scale->pixel_b);
    put_number(value, L"distance_mm", scale->distance_mm);
    if (scale->orthogonal_a >= 0 && scale->orthogonal_b >= 0) {
        put_number(value, L"orthogonal_a", scale->orthogonal_a);
        put_number(value, L"orthogonal_b", scale->orthogonal_b);
    }
    object.Insert(name, value);
}

struct FileHandle {
    HANDLE value = INVALID_HANDLE_VALUE;
    ~FileHandle() { if (value != INVALID_HANDLE_VALUE) CloseHandle(value); }
};

void write_atomically(const std::filesystem::path& path, const std::string& contents) {
    if (path.empty() || path.filename().empty()) throw std::invalid_argument("Profile path is empty");
    const auto parent = path.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::filesystem::path temporary;
    FileHandle file;
    for (unsigned attempt = 0; attempt < 100; ++attempt) {
        temporary = path;
        temporary += L".tmp." + std::to_wstring(GetCurrentProcessId()) + L"." +
                     std::to_wstring(GetTickCount64()) + L"." + std::to_wstring(attempt);
        file.value = CreateFileW(temporary.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                                 FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file.value != INVALID_HANDLE_VALUE) break;
        if (GetLastError() != ERROR_FILE_EXISTS)
            throw std::runtime_error("Cannot create temporary profile file");
    }
    if (file.value == INVALID_HANDLE_VALUE) throw std::runtime_error("Cannot reserve temporary profile file");
    try {
        DWORD written = 0;
        if (!WriteFile(file.value, contents.data(), static_cast<DWORD>(contents.size()), &written, nullptr) ||
            written != contents.size() || !FlushFileBuffers(file.value))
            throw std::runtime_error("Cannot write and flush profile file");
        if (!CloseHandle(file.value)) throw std::runtime_error("Cannot close profile file");
        file.value = INVALID_HANDLE_VALUE;
        // Same-directory replacement leaves either the old or the complete new profile visible.
        if (!MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            throw std::runtime_error("Cannot replace profile file");
    } catch (...) {
        if (file.value != INVALID_HANDLE_VALUE) {
            CloseHandle(file.value);
            file.value = INVALID_HANDLE_VALUE;
        }
        DeleteFileW(temporary.c_str());
        throw;
    }
}

} // namespace

std::filesystem::path default_profile_path() try {
    PWSTR directory = nullptr;
    winrt::check_hresult(SHGetKnownFolderPath(FOLDERID_LocalAppData, KF_FLAG_DEFAULT, nullptr, &directory));
    std::filesystem::path result;
    try { result = std::filesystem::path(directory) / L"PausUltrasound" / L"profile.json"; }
    catch (...) { CoTaskMemFree(directory); throw; }
    CoTaskMemFree(directory);
    return result;
} catch (const winrt::hresult_error& error) {
    throw std::runtime_error("Cannot locate profile directory: " + winrt::to_string(error.message()));
}

Profile load_profile(const std::filesystem::path& path) try {
    const auto size = std::filesystem::file_size(path);
    if (size == 0 || size > kMaximumProfileBytes) throw std::invalid_argument("Profile file has an invalid size");
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open profile file");
    std::string contents(static_cast<std::size_t>(size), '\0');
    if (!input.read(contents.data(), static_cast<std::streamsize>(contents.size())) ||
        input.peek() != std::ifstream::traits_type::eof())
        throw std::runtime_error("Profile file changed or could not be read completely");
    if (contents.compare(0, 3, "\xEF\xBB\xBF") == 0) contents.erase(0, 3);
    const auto object = JsonObject::Parse(from_utf8(contents));
    if (integer<int>(object, L"schema_version") != kSchemaVersion)
        throw std::invalid_argument("Unsupported profile schema version");
    Profile profile;
    profile.executable = string_or(object, L"executable");
    profile.working_directory = string_or(object, L"working_directory");
    profile.process_name = string_or(object, L"process_name");
    profile.title_contains = string_or(object, L"title_contains");
    profile.host = string_or(object, L"host", profile.host);
    if (object.HasKey(L"port")) profile.port = integer<int>(object, L"port");
    if (object.HasKey(L"roi")) {
        const auto roi = object.GetNamedObject(L"roi");
        profile.roi = {integer<int>(roi, L"x"), integer<int>(roi, L"y"),
                       integer<int>(roi, L"width"), integer<int>(roi, L"height")};
    }
    if (object.HasKey(L"reference_width")) profile.reference_width = integer<int>(object, L"reference_width");
    if (object.HasKey(L"reference_height")) profile.reference_height = integer<int>(object, L"reference_height");
    if (object.HasKey(L"reference_dpi")) profile.reference_dpi = integer<unsigned>(object, L"reference_dpi");
    profile.roi_confirmed = bool_or(object, L"roi_confirmed", false);
    profile.auto_connect = bool_or(object, L"auto_connect", false);
    profile.launch_vendor = bool_or(object, L"launch_vendor", true);
    profile.minimize_to_tray = bool_or(object, L"minimize_to_tray", true);
    profile.horizontal_scale = read_scale(object, L"horizontal_scale");
    profile.vertical_scale = read_scale(object, L"vertical_scale");
    profile.imaging_context = string_or(object, L"imaging_context");
    profile.revision = string_or(object, L"revision");
    validate_profile(profile, false);
    return profile;
} catch (const winrt::hresult_error& error) {
    throw std::runtime_error("Cannot parse profile JSON: " + winrt::to_string(error.message()));
}

void save_profile(const std::filesystem::path& path, const Profile& profile) try {
    validate_profile(profile, false);
    // Never silently replace a corrupt or future-version configuration.
    if (std::filesystem::exists(path)) load_profile(path);
    JsonObject object;
    put_number(object, L"schema_version", kSchemaVersion);
    put_string(object, L"executable", profile.executable);
    put_string(object, L"working_directory", profile.working_directory);
    put_string(object, L"process_name", profile.process_name);
    put_string(object, L"title_contains", profile.title_contains);
    put_string(object, L"host", profile.host);
    put_number(object, L"port", profile.port);
    JsonObject roi;
    put_number(roi, L"x", profile.roi.x);
    put_number(roi, L"y", profile.roi.y);
    put_number(roi, L"width", profile.roi.width);
    put_number(roi, L"height", profile.roi.height);
    object.Insert(L"roi", roi);
    put_number(object, L"reference_width", profile.reference_width);
    put_number(object, L"reference_height", profile.reference_height);
    put_number(object, L"reference_dpi", profile.reference_dpi);
    put_bool(object, L"roi_confirmed", profile.roi_confirmed);
    put_bool(object, L"auto_connect", profile.auto_connect);
    put_bool(object, L"launch_vendor", profile.launch_vendor);
    put_bool(object, L"minimize_to_tray", profile.minimize_to_tray);
    put_scale(object, L"horizontal_scale", profile.horizontal_scale);
    put_scale(object, L"vertical_scale", profile.vertical_scale);
    put_string(object, L"imaging_context", profile.imaging_context);
    put_string(object, L"revision", profile.revision);
    const std::string contents = to_utf8(object.Stringify()) + "\n";
    if (contents.size() > kMaximumProfileBytes) throw std::invalid_argument("Profile file is too large");
    write_atomically(path, contents);
} catch (const winrt::hresult_error& error) {
    throw std::runtime_error("Cannot serialize profile JSON: " + winrt::to_string(error.message()));
}

} // namespace uscapture
