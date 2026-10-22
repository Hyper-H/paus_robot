#include "capture_app.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#include <shellapi.h>
#include <d3d11.h>
#include <windows.graphics.capture.interop.h>
#include <windows.graphics.directx.direct3d11.interop.h>
#include <winrt/Windows.Data.Json.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Graphics.Capture.h>
#include <winrt/Windows.Graphics.DirectX.Direct3D11.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace uscapture {
namespace {
using namespace winrt;
using namespace winrt::Windows::Graphics::Capture;
using namespace winrt::Windows::Data::Json;
using winrt::Windows::Graphics::DirectX::DirectXPixelFormat;
using winrt::Windows::Graphics::DirectX::Direct3D11::IDirect3DDevice;
using Clock = std::chrono::steady_clock;

double qpc_seconds() {
    LARGE_INTEGER t{}, f{};
    QueryPerformanceCounter(&t); QueryPerformanceFrequency(&f);
    return static_cast<double>(t.QuadPart) / static_cast<double>(f.QuadPart);
}
std::string error_text() {
    try { throw; }
    catch (const hresult_error& e) { return to_string(e.message()); }
    catch (const std::exception& e) { return e.what(); }
    catch (...) { return "Unknown Windows capture error"; }
}
std::wstring wide(const std::string& s) { return std::wstring(to_hstring(s)); }
bool iequal(const std::wstring& a, const std::wstring& b) {
    return CompareStringOrdinal(a.c_str(), -1, b.c_str(), -1, TRUE) == CSTR_EQUAL;
}
std::wstring basename(const std::wstring& path) {
    auto n = path.find_last_of(L"/\\"); return n == std::wstring::npos ? path : path.substr(n + 1);
}
std::string new_session() {
    GUID guid{}; check_hresult(CoCreateGuid(&guid));
    wchar_t text[40]{}; StringFromGUID2(guid, text, 40);
    return to_string(text);
}
bool process_running(const Profile& p) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) throw std::runtime_error("Cannot enumerate vendor processes");
    PROCESSENTRY32W e{}; e.dwSize = sizeof(e); bool found = false;
    const auto name = wide(p.process_name);
    if (Process32FirstW(snap, &e)) do { if (iequal(e.szExeFile, name)) { found = true; break; } } while (Process32NextW(snap, &e));
    CloseHandle(snap); return found;
}
HWND discover(const Profile& p);
void launch_once(const Profile& p) {
    if (p.process_name.empty() ? discover(p) != nullptr : process_running(p)) return;
    if (p.executable.empty()) throw std::runtime_error("Choose the vendor executable before starting it");
    const auto exe = wide(p.executable);
    auto cwd = p.working_directory.empty() ? std::filesystem::path(exe).parent_path().wstring() : wide(p.working_directory);
    SHELLEXECUTEINFOW info{}; info.cbSize = sizeof(info);
    info.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_FLAG_NO_UI;
    info.lpVerb = L"open"; info.lpFile = exe.c_str();
    info.lpDirectory = cwd.empty() ? nullptr : cwd.c_str(); info.nShow = SW_SHOWNORMAL;
    if (!ShellExecuteExW(&info)) throw std::runtime_error("Cannot start vendor software (Windows error " + std::to_string(GetLastError()) + ")");
    if (info.hProcess) CloseHandle(info.hProcess);
}

struct WindowsFound { const Profile* profile; std::vector<HWND> windows; };
BOOL CALLBACK enum_window(HWND hwnd, LPARAM data) noexcept {
    try {
        auto& found = *reinterpret_cast<WindowsFound*>(data);
        if (!IsWindowVisible(hwnd) || GetWindow(hwnd, GW_OWNER)) return TRUE;
        DWORD pid = 0; GetWindowThreadProcessId(hwnd, &pid);
        HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
        if (!process) return TRUE;
        std::wstring path(32768, L'\0'); DWORD length = static_cast<DWORD>(path.size());
        bool read = QueryFullProcessImageNameW(process, 0, path.data(), &length) != 0;
        CloseHandle(process); if (!read) return TRUE; path.resize(length);
        if (!found.profile->process_name.empty() && !iequal(basename(path), wide(found.profile->process_name))) return TRUE;
        int size = GetWindowTextLengthW(hwnd);
        std::wstring title(static_cast<std::size_t>(size) + 1, L'\0');
        title.resize(GetWindowTextW(hwnd, title.data(), size + 1));
        if (!found.profile->title_contains.empty() && title.find(wide(found.profile->title_contains)) == std::wstring::npos) return TRUE;
        RECT rect{}; if (!GetClientRect(hwnd, &rect) || rect.right < 32 || rect.bottom < 32) return TRUE;
        found.windows.push_back(hwnd);
    } catch (...) { /* Do not unwind through a Win32 callback. */ }
    return TRUE;
}
HWND discover(const Profile& p) {
    WindowsFound found{&p, {}}; EnumWindows(enum_window, reinterpret_cast<LPARAM>(&found));
    if (found.windows.size() > 1) throw std::runtime_error("Multiple vendor windows match; refine the window title");
    return found.windows.empty() ? nullptr : found.windows.front();
}

class WindowCapture : public std::enable_shared_from_this<WindowCapture> {
public:
    std::function<void(std::shared_ptr<PreviewFrame>)> on_image;
    std::function<void(std::string)> on_error;
    void open(HWND hwnd) {
        hwnd_ = hwnd;
        if (!GraphicsCaptureSession::IsSupported()) throw std::runtime_error("WGC is unavailable in this desktop session");
        check_hresult(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
            D3D11_CREATE_DEVICE_BGRA_SUPPORT, nullptr, 0, D3D11_SDK_VERSION,
            device_.put(), nullptr, context_.put()));
        auto dxgi = device_.as<IDXGIDevice>(); com_ptr<::IInspectable> inspectable;
        check_hresult(CreateDirect3D11DeviceFromDXGIDevice(dxgi.get(), inspectable.put()));
        direct_ = inspectable.as<IDirect3DDevice>();
        auto factory = get_activation_factory<GraphicsCaptureItem, IGraphicsCaptureItemInterop>();
        check_hresult(factory->CreateForWindow(hwnd, guid_of<GraphicsCaptureItem>(), put_abi(item_)));
        size_ = item_.Size();
        pool_ = Direct3D11CaptureFramePool::CreateFreeThreaded(direct_, DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, size_);
        auto weak = weak_from_this();
        token_ = pool_.FrameArrived([weak](auto const& pool, auto const&) noexcept {
            if (auto self = weak.lock()) self->arrived(pool);
        });
        closed_token_ = item_.Closed([weak](auto const&, auto const&) noexcept {
            if (auto self = weak.lock()) {
                std::lock_guard<std::mutex> guard(self->frame_lock_);
                if (!self->closing_.load()) self->on_error("Vendor capture window closed");
            }
        });
        session_ = pool_.CreateCaptureSession(item_);
        // Windows 10 2004+ supports omitting the system cursor from image data.
        if (session_.try_as<IGraphicsCaptureSession2>()) session_.IsCursorCaptureEnabled(false);
        else throw std::runtime_error("Cursor-free capture requires Windows 10 2004 or newer");
        session_.StartCapture();
    }
    void close() noexcept {
        // Removing events after pool.Close() can fail-fast with RO_E_CLOSED.
        if (closing_.exchange(true)) return;
        try { if (pool_) pool_.FrameArrived(token_); } catch (...) {}
        try { if (item_) item_.Closed(closed_token_); } catch (...) {}
        try { if (session_) session_.Close(); } catch (...) {}
        try { if (pool_) pool_.Close(); } catch (...) {}
        // Drain any callback that could still call the controller's handlers.
        std::lock_guard<std::mutex> drain(frame_lock_);
    }
    ~WindowCapture() { close(); }
private:
    void arrived(Direct3D11CaptureFramePool const& pool) noexcept {
        std::lock_guard<std::mutex> guard(frame_lock_);
        if (closing_.load()) return;
        try {
            const double callback_time_s = qpc_seconds();
            auto frame = pool.TryGetNextFrame(); if (!frame) return;
            for (;;) { auto next = pool.TryGetNextFrame(); if (!next) break; frame.Close(); frame = std::move(next); }
            auto content = frame.ContentSize();
            if (content.Width <= 0 || content.Height <= 0 || IsIconic(hwnd_)) { frame.Close(); return; }
            if (content.Width > 4096 || content.Height > 4096) throw std::runtime_error("Window exceeds capture limit (4096 pixels)");
            if (content.Width != size_.Width || content.Height != size_.Height) {
                frame.Close(); staging_ = nullptr; size_ = content;
                pool.Recreate(direct_, DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, size_); return;
            }
            auto surface = frame.Surface().as<::Windows::Graphics::DirectX::Direct3D11::IDirect3DDxgiInterfaceAccess>();
            com_ptr<ID3D11Texture2D> texture;
            check_hresult(surface->GetInterface(IID_PPV_ARGS(texture.put())));
            D3D11_TEXTURE2D_DESC desc{}; texture->GetDesc(&desc);
            if (desc.Width < static_cast<unsigned>(content.Width) || desc.Height < static_cast<unsigned>(content.Height)) { frame.Close(); return; }
            if (!staging_ || desc.Width != staging_width_ || desc.Height != staging_height_) {
                staging_ = nullptr; staging_width_ = desc.Width; staging_height_ = desc.Height;
                desc.Usage = D3D11_USAGE_STAGING; desc.BindFlags = 0;
                desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ; desc.MiscFlags = 0;
                check_hresult(device_->CreateTexture2D(&desc, nullptr, staging_.put()));
            }
            auto output = std::make_shared<PreviewFrame>();
            output->width = content.Width; output->height = content.Height; output->dpi = GetDpiForWindow(hwnd_);
            // Keep compositor timestamps separate: they can be slightly ahead
            // of callback QPC on some drivers and are not acoustic timestamps.
            output->capture_time_s = callback_time_s;
            output->wgc_surface_time_s = static_cast<double>(frame.SystemRelativeTime().count()) / 10000000.0;
            output->bgra.resize(static_cast<std::size_t>(output->width) * output->height * 4);
            context_->CopyResource(staging_.get(), texture.get());
            D3D11_MAPPED_SUBRESOURCE map{};
            check_hresult(context_->Map(staging_.get(), 0, D3D11_MAP_READ, 0, &map));
            const auto stride = static_cast<std::size_t>(output->width) * 4;
            if (map.RowPitch < stride) { context_->Unmap(staging_.get(), 0); throw std::runtime_error("Unexpected texture stride"); }
            for (int y = 0; y < output->height; ++y)
                std::memcpy(output->bgra.data() + y * stride, static_cast<const std::uint8_t*>(map.pData) + static_cast<std::size_t>(y) * map.RowPitch, stride);
            context_->Unmap(staging_.get(), 0); frame.Close();
            if (!closing_.load()) on_image(std::move(output));
        } catch (...) { if (!closing_.load()) on_error(error_text()); }
    }
    HWND hwnd_ = nullptr;
    std::atomic<bool> closing_{false}; std::mutex frame_lock_;
    com_ptr<ID3D11Device> device_; com_ptr<ID3D11DeviceContext> context_; com_ptr<ID3D11Texture2D> staging_;
    unsigned staging_width_ = 0, staging_height_ = 0;
    IDirect3DDevice direct_{nullptr}; GraphicsCaptureItem item_{nullptr};
    Direct3D11CaptureFramePool pool_{nullptr}; GraphicsCaptureSession session_{nullptr};
    winrt::Windows::Graphics::SizeInt32 size_{}; event_token token_{}, closed_token_{};
};

class Connection {
public:
    std::function<bool()> cancelled;
    ~Connection() { close(); }
    bool open() const { return socket_ != INVALID_SOCKET; }
    void close() { if (open()) closesocket(socket_); socket_ = INVALID_SOCKET; }
    void connect_to(const Profile& p) {
        sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(static_cast<u_short>(p.port));
        const auto host = p.host == "localhost" ? std::string("127.0.0.1") : p.host;
        if (InetPtonA(AF_INET, host.c_str(), &address.sin_addr) != 1) throw std::runtime_error("Receiver address must be an IPv4 address");
        socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (!open()) throw std::runtime_error("Cannot create TCP socket");
        u_long nonblocking = 1;
        if (ioctlsocket(socket_, FIONBIO, &nonblocking)) throw std::runtime_error("Cannot configure TCP socket");
        int enabled = 1; setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&enabled), sizeof(enabled));
        if (::connect(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR) {
            if (WSAGetLastError() != WSAEWOULDBLOCK) throw std::runtime_error("Receiver is unavailable");
            wait(true, Clock::now() + std::chrono::milliseconds(1500));
            int error = 0, length = sizeof(error);
            if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &length) || error) throw std::runtime_error("Cannot connect to receiver");
        }
    }
    void transfer(bool writing, void* bytes, std::size_t length, Clock::time_point deadline) {
        auto* ptr = static_cast<char*>(bytes);
        while (length) {
            wait(writing, deadline);
            int amount = static_cast<int>(std::min<std::size_t>(length, 65536));
            int n = writing ? ::send(socket_, ptr, amount, 0) : ::recv(socket_, ptr, amount, 0);
            if (n == SOCKET_ERROR && WSAGetLastError() == WSAEWOULDBLOCK) continue;
            if (n <= 0) throw std::runtime_error("Receiver disconnected");
            ptr += n; length -= static_cast<std::size_t>(n);
        }
    }
private:
    void wait(bool writing, Clock::time_point deadline) {
        for (;;) {
            if (cancelled && cancelled()) throw std::runtime_error("Transmission stopped or configuration changed");
            if (Clock::now() >= deadline) throw std::runtime_error("TCP send/ACK deadline exceeded");
            fd_set fds, errors; FD_ZERO(&fds); FD_ZERO(&errors); FD_SET(socket_, &fds); FD_SET(socket_, &errors);
            timeval timeout{0, 20000};
            int count = select(0, writing ? nullptr : &fds, writing ? &fds : nullptr, &errors, &timeout);
            if (count < 0 || FD_ISSET(socket_, &errors)) throw std::runtime_error("TCP connection error");
            if (count > 0) return;
        }
    }
    SOCKET socket_ = INVALID_SOCKET;
};

void number(JsonObject& o, const wchar_t* key, double n) { o.SetNamedValue(key, JsonValue::CreateNumberValue(n)); }
void text(JsonObject& o, const wchar_t* key, const std::string& s) { o.SetNamedValue(key, JsonValue::CreateStringValue(to_hstring(s))); }
JsonArray numbers(std::initializer_list<double> ns) { JsonArray a; for (double n : ns) a.Append(JsonValue::CreateNumberValue(n)); return a; }
JsonObject header(const Profile& p, const PreviewFrame& f, const std::string& session, std::uint64_t dropped) {
    JsonObject o; number(o,L"version",2); text(o,L"type","frame"); text(o,L"session_id",session); number(o,L"sequence_id",static_cast<double>(f.sequence));
    number(o,L"width",p.roi.width); number(o,L"height",p.roi.height);
    text(o,L"encoding","mono8");text(o,L"storage_order","row_major");text(o,L"byte_order","little");text(o,L"modality","bmode");text(o,L"source","windows_window_capture");
    text(o,L"timestamp_kind","display_callback");text(o,L"capture_clock","windows_qpc");text(o,L"source_time_definition","wgc_callback_qpc");
    number(o,L"wgc_surface_time_s",f.wgc_surface_time_s);
    number(o,L"source_time_s",f.capture_time_s);number(o,L"send_time_s",qpc_seconds());
    number(o,L"sender_queue_age_s",std::max(0.0,qpc_seconds()-f.capture_time_s));number(o,L"sender_dropped",static_cast<double>(dropped));
    text(o,L"intensity_mapping","bt709_fixed_uint8");o.SetNamedValue(L"geometry_valid",JsonValue::CreateBooleanValue(false));text(o,L"geometry_version","");
    o.SetNamedValue(L"pixel_spacing_m",numbers({0,0}));o.SetNamedValue(L"origin_m",numbers({0,0,0}));o.SetNamedValue(L"column_direction",numbers({1,0,0}));o.SetNamedValue(L"row_direction",numbers({0,1,0}));
    o.SetNamedValue(L"roi_confirmed",JsonValue::CreateBooleanValue(p.roi_confirmed));
    o.SetNamedValue(L"capture_size",numbers({static_cast<double>(f.width),static_cast<double>(f.height)}));
    o.SetNamedValue(L"roi_pixels",numbers({static_cast<double>(p.roi.x),static_cast<double>(p.roi.y),static_cast<double>(p.roi.width),static_cast<double>(p.roi.height)}));
    text(o,L"profile_revision",p.revision);text(o,L"imaging_context",p.imaging_context);
    JsonObject scales;
    scales.SetNamedValue(L"horizontal",p.horizontal_scale ? JsonValue::CreateNumberValue(p.horizontal_scale->mm_per_pixel()) : JsonValue::Parse(L"null"));
    scales.SetNamedValue(L"vertical",p.vertical_scale ? JsonValue::CreateNumberValue(p.vertical_scale->mm_per_pixel()) : JsonValue::Parse(L"null"));
    o.SetNamedValue(L"display_scale_mm_per_pixel",scales);return o;
}
void little(std::uint8_t* p, std::uint64_t n, int bytes) { for(int i=0;i<bytes;++i) p[i]=static_cast<std::uint8_t>(n>>(8*i)); }
std::uint64_t read_little(const std::uint8_t* p,int bytes) { std::uint64_t n=0;for(int i=0;i<bytes;++i)n|=static_cast<std::uint64_t>(p[i])<<(8*i);return n; }
void send_frame(Connection& c,const Profile& p,const PreviewFrame& f,const std::string& session,std::uint64_t dropped) {
    auto pixels=crop_mono8(f,p.roi);auto json=to_string(header(p,f,session,dropped).Stringify());
    if(json.size()>16384||pixels.size()>16*1024*1024)throw std::runtime_error("Frame exceeds USFR limits");
    std::array<std::uint8_t,16> prefix{};std::memcpy(prefix.data(),"USFR",4);little(prefix.data()+4,json.size(),4);little(prefix.data()+8,pixels.size(),8);
    auto deadline=Clock::now()+std::chrono::milliseconds(1000);
    c.transfer(true,prefix.data(),prefix.size(),deadline);c.transfer(true,json.data(),json.size(),deadline);c.transfer(true,pixels.data(),pixels.size(),deadline);
    c.transfer(false,prefix.data(),prefix.size(),deadline);
    auto length=read_little(prefix.data()+4,4);
    if(std::memcmp(prefix.data(),"USFR",4)||!length||length>16384||read_little(prefix.data()+8,8))throw std::runtime_error("Invalid ACK prefix");
    std::string ack(static_cast<std::size_t>(length),'\0');c.transfer(false,ack.data(),ack.size(),deadline);
    auto obj=JsonObject::Parse(to_hstring(ack));
    if(obj.GetNamedNumber(L"version")!=2||obj.GetNamedString(L"type")!=L"ack"||obj.GetNamedString(L"status")!=L"accepted"||to_string(obj.GetNamedString(L"session_id"))!=session||obj.GetNamedNumber(L"sequence_id")!=static_cast<double>(f.sequence))throw std::runtime_error("ACK does not match this frame");
}
} // namespace

struct CaptureController::Impl {
    mutable std::mutex lock;
    std::condition_variable wake;
    Profile profile;
    Status state;
    std::shared_ptr<const PreviewFrame> latest;
    std::thread worker;
    std::atomic<bool> stopped{true}, transmit{false};
    std::atomic<bool> capture_failed{false};
    std::atomic<std::uint64_t> epoch{0};
    void report(const char* name,const std::string& message={}) {
        std::lock_guard<std::mutex> guard(lock);state.state=name;state.message=message;
    }
    void loop(bool launch) noexcept {
        std::shared_ptr<WindowCapture> capture;
        WSADATA data{};bool winsock=false;
        try {
            init_apartment(apartment_type::multi_threaded);
            if(WSAStartup(MAKEWORD(2,2),&data))throw std::runtime_error("Cannot initialize networking");winsock=true;
            Profile source;{std::lock_guard<std::mutex> g(lock);source=profile;}
            if(launch)launch_once(source);
            Connection connection;std::string session;HWND window=nullptr;
            std::uint64_t connection_epoch=0,last_attempt=0;int backoff_ms=250;auto retry_at=Clock::now();
            while(!stopped.load()) {
                Profile current;std::uint64_t current_epoch;
                {std::lock_guard<std::mutex> g(lock);current=profile;current_epoch=epoch.load();}
                if(capture&&(capture_failed.load()||!IsWindow(window)||!IsWindowVisible(window))) {
                    capture->close();capture.reset();connection.close();window=nullptr;
                    std::lock_guard<std::mutex> g(lock);latest.reset();state.connected=false;
                }
                if(!capture) {
                    try {window=discover(current);}catch(...){report("ERROR",error_text());pause(500);continue;}
                    if(!window){report("WAIT_WINDOW","Waiting for the configured vendor window");pause(250);continue;}
                    capture=std::make_shared<WindowCapture>();
                    capture->on_image=[this](std::shared_ptr<PreviewFrame> image){
                        std::lock_guard<std::mutex> g(lock);image->sequence=++state.captured;
                        state.width=image->width;state.height=image->height;state.dpi=image->dpi;latest=std::move(image);wake.notify_all();
                    };
                    capture_failed.store(false);
                    capture->on_error=[this](std::string message){capture_failed.store(true);report("CAPTURE_ERROR",message);wake.notify_all();};
                    try{capture->open(window);}catch(...){capture->close();capture.reset();report("CAPTURE_ERROR",error_text());pause(1000);continue;}
                    connection.close();last_attempt=0;
                }
                std::shared_ptr<const PreviewFrame> image;{std::lock_guard<std::mutex> g(lock);image=latest;}
                if(!transmit.load()) {
                    connection.close();{std::lock_guard<std::mutex> g(lock);state.connected=false;}
                    if(image&&current.roi_confirmed&&!matches(current,*image))report("NEEDS_ROI","Window size/DPI changed; reconfirm the ROI");
                    else report(image?"PREVIEW":"WAIT_FRAME");pause(30);continue;
                }
                if(!image||IsIconic(window)||qpc_seconds()-image->capture_time_s>1.0) {
                    connection.close();{std::lock_guard<std::mutex> g(lock);state.connected=false;}
                    report("STALE","No recent captured frame; transmission suspended");pause(50);continue;
                }
                if(!matches(current,*image)) {
                    transmit.store(false);epoch.fetch_add(1);connection.close();
                    {std::lock_guard<std::mutex> g(lock);state.connected=false;}
                    report("NEEDS_ROI","Window size/DPI differs from the confirmed ROI; reconfirm before connecting");pause(100);continue;
                }
                if(connection.open()&&connection_epoch!=current_epoch)connection.close();
                connection.cancelled=[this,current_epoch]{return stopped.load()||!transmit.load()||epoch.load()!=current_epoch;};
                if(!connection.open()) {
                    if(Clock::now()<retry_at){pause(30);continue;}
                    report("CONNECTING","Waiting for the Linux receiver");
                    try{
                        connection.connect_to(current);connection_epoch=current_epoch;session=new_session();
                        last_attempt=0;backoff_ms=250;{std::lock_guard<std::mutex> g(lock);state.connected=true;}
                    }catch(...){connection.close();{std::lock_guard<std::mutex> g(lock);state.connected=false;}report("CONNECTING",error_text());retry_at=Clock::now()+std::chrono::milliseconds(backoff_ms);backoff_ms=std::min(4000,backoff_ms*2);}
                    continue; // Retrieve the latest frame after connection establishment.
                }
                if(image->sequence==last_attempt){pause(5);continue;}
                std::uint64_t dropped=0;
                {std::lock_guard<std::mutex> g(lock);if(last_attempt&&image->sequence>last_attempt+1)state.dropped+=image->sequence-last_attempt-1;dropped=state.dropped;}
                last_attempt=image->sequence;
                if(qpc_seconds()-image->capture_time_s>0.25){std::lock_guard<std::mutex> g(lock);++state.dropped;continue;}
                const auto before=Clock::now();
                try {
                    send_frame(connection,current,*image,session,dropped);
                    std::lock_guard<std::mutex> g(lock);++state.acknowledged;state.last_ack_ms=std::chrono::duration<double,std::milli>(Clock::now()-before).count();state.state="STREAMING";state.message.clear();
                }catch(...){connection.close();{std::lock_guard<std::mutex> g(lock);state.connected=false;++state.dropped;}report("CONNECTING",error_text());retry_at=Clock::now()+std::chrono::milliseconds(backoff_ms);}
            }
        } catch(...) {report("ERROR",error_text());}
        if(capture)capture->close();
        capture.reset();
        if(winsock)WSACleanup();
        std::lock_guard<std::mutex> g(lock);state.connected=false;transmit.store(false);
    }
    void pause(int ms){std::unique_lock<std::mutex> g(lock);wake.wait_for(g,std::chrono::milliseconds(ms));}
};

CaptureController::CaptureController():impl_(std::make_unique<Impl>()){}
CaptureController::~CaptureController(){stop();}
void CaptureController::start(const Profile& p,bool launch){
    validate_profile(p,false);stop();
    {std::lock_guard<std::mutex> g(impl_->lock);impl_->profile=p;impl_->state=Status{};impl_->latest.reset();}
    impl_->stopped.store(false);impl_->epoch.fetch_add(1);impl_->worker=std::thread([this,launch]{impl_->loop(launch);});
}
void CaptureController::apply_profile(const Profile& p){
    validate_profile(p,false);
    std::lock_guard<std::mutex> g(impl_->lock);
    if(impl_->worker.joinable()&&(p.process_name!=impl_->profile.process_name||p.title_contains!=impl_->profile.title_contains))throw std::runtime_error("Restart the source preview after changing window selection");
    impl_->profile=p;impl_->epoch.fetch_add(1);impl_->wake.notify_all();
}
void CaptureController::set_transmitting(bool enabled){
    std::lock_guard<std::mutex> g(impl_->lock);
    if(enabled){
        validate_profile(impl_->profile,true);
        if(impl_->stopped.load()||!impl_->latest||!matches(impl_->profile,*impl_->latest))throw std::runtime_error("Capture and confirm the current window ROI first");
        if(qpc_seconds()-impl_->latest->capture_time_s>1.0)throw std::runtime_error("The current preview is stale");
    }
    impl_->transmit.store(enabled);impl_->epoch.fetch_add(1);impl_->wake.notify_all();
}
void CaptureController::stop(){
    impl_->stopped.store(true);impl_->transmit.store(false);impl_->epoch.fetch_add(1);impl_->wake.notify_all();
    if(impl_->worker.joinable())impl_->worker.join();
    std::lock_guard<std::mutex> g(impl_->lock);impl_->latest.reset();impl_->state.state="STOPPED";impl_->state.connected=false;
}
std::shared_ptr<const PreviewFrame> CaptureController::preview()const{std::lock_guard<std::mutex> g(impl_->lock);return impl_->latest;}
Status CaptureController::status()const{
    std::lock_guard<std::mutex> g(impl_->lock);auto s=impl_->state;s.transmitting=impl_->transmit.load();
    if(impl_->latest){s.frame_age_s=std::max(0.0,qpc_seconds()-impl_->latest->capture_time_s);s.roi_matches=matches(impl_->profile,*impl_->latest);}
    if(s.frame_age_s>1.0&&s.state=="STREAMING")s.state="STALE";
    return s;
}
} // namespace uscapture
