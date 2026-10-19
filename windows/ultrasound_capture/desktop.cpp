#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <commdlg.h>
#include <shellapi.h>
#include <shlobj.h>
#include <roapi.h>

#include "capture_app.hpp"
#include "editor_core.hpp"
#include "icons_win.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "runtimeobject.lib")
#pragma comment(lib, "msimg32.lib")
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

namespace {
using namespace uscapture;
constexpr wchar_t kMainClass[] = L"PausUltrasoundCaptureDesktop";
constexpr wchar_t kBodyClass[] = L"PausUltrasoundCaptureBody";
constexpr wchar_t kImageClass[] = L"PausUltrasoundCaptureImage";
constexpr UINT kTray = WM_APP + 1, kTaskDone = WM_APP + 2;
constexpr UINT_PTR kRefresh = 1;
constexpr COLORREF kPaper = RGB(255,255,255), kBand = RGB(245,247,248);
constexpr COLORREF kInk = RGB(36,49,56), kMuted = RGB(102,117,124), kLine = RGB(220,227,231);
constexpr COLORREF kAccent = RGB(7,109,106), kSelected = RGB(228,243,241);
constexpr COLORREF kSelection = RGB(38,223,192), kWarning = RGB(139,87,21), kError = RGB(179,45,61);

enum Id {
    Executable = 100, BrowseExe, WorkingDir, BrowseDir, ProcessName, Title,
    Host, Port, LaunchVendor, AutoConnect, Context, StartPreview, Transmit,
    Pause, SaveDraft, FullView, CropView, Draw, Move, Undo, FullWindow,
    RoiX, RoiY, RoiWidth, RoiHeight, Confirm, RulerAxis, PickRuler,
    RulerDistance, RecordRuler, ClearRuler, StatusText, SourceText, RulerText,
    RoiText, Receipt, MainImage, OpenRoi, OpenSettings, Back, Cancel,
    MainSource, MainMeta, MainNotice, Brand, PageTitle, DirtyState,
    TabSource, TabReceiver, TabStartup, Advanced, MinimizeToTray, ConfigPath,
    ReloadConfig, RestoreRoi, RefreshReference, InspectorKind, ActiveEdge,
    ActiveTick, Zoom, MagnifierView, MagnifierPosition, NudgeLeft, NudgeUp,
    NudgeDown, NudgeRight, ScaleExpanded, RulerA, RulerB, RulerUnit,
    UnitConfirmed, RoiError, SettingsError, MagnifierTitle, FooterHint,
    CropHeading, ScaleHeading, RulerNote, MainCaption,
    TrayShow = 300, TrayPause, TrayExit
};
enum class Page { Main, Settings, Roi };
enum class SettingTab { Source, Receiver, Startup };
enum class EditMode { Move, Draw, Ruler };
enum class Navigation { None, Main, Settings, Roi, Close, Exit };
enum class Action { Preview, Transmit, Pause, EnterRoi, SaveSettings, SaveRoi, Shutdown };

std::wstring wide(const std::string& value) {
    if(value.empty()) return {};
    const int n=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),nullptr,0);
    if(!n) throw std::runtime_error("Invalid UTF-8 text");
    std::wstring result(n,L'\0');
    MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),result.data(),n);
    return result;
}
std::string utf8(const std::wstring& value) {
    if(value.empty()) return {};
    const int n=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),nullptr,0,nullptr,nullptr);
    if(!n) throw std::runtime_error("Invalid Unicode text");
    std::string result(n,'\0');
    WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),result.data(),n,nullptr,nullptr);
    return result;
}
std::wstring window_text(HWND control) {
    std::wstring result(static_cast<std::size_t>(GetWindowTextLengthW(control))+1,L'\0');
    result.resize(GetWindowTextW(control,result.data(),static_cast<int>(result.size())));
    return result;
}
bool same_rect(const Rect& a,const Rect& b) {
    return a.x==b.x&&a.y==b.y&&a.width==b.width&&a.height==b.height;
}
bool same_scale(const std::optional<RulerScale>& a,const std::optional<RulerScale>& b) {
    if(bool(a)!=bool(b)) return false;
    return !a||(a->pixel_a==b->pixel_a&&a->pixel_b==b->pixel_b&&a->distance_mm==b->distance_mm&&
        a->orthogonal_a==b->orthogonal_a&&a->orthogonal_b==b->orthogonal_b);
}
bool same_profile(const Profile& a,const Profile& b) {
    return a.executable==b.executable&&a.working_directory==b.working_directory&&a.process_name==b.process_name&&
        a.title_contains==b.title_contains&&a.host==b.host&&a.port==b.port&&same_rect(a.roi,b.roi)&&
        a.reference_width==b.reference_width&&a.reference_height==b.reference_height&&a.reference_dpi==b.reference_dpi&&
        a.roi_confirmed==b.roi_confirmed&&a.auto_connect==b.auto_connect&&a.launch_vendor==b.launch_vendor&&
        a.minimize_to_tray==b.minimize_to_tray&&a.imaging_context==b.imaging_context&&
        same_scale(a.horizontal_scale,b.horizontal_scale)&&same_scale(a.vertical_scale,b.vertical_scale);
}
std::string source_key(const Profile& p) {
    return p.executable+'\n'+p.working_directory+'\n'+p.process_name+'\n'+p.title_contains;
}
bool same_shape(const PreviewFrame& a,const PreviewFrame& b) {
    return a.width==b.width&&a.height==b.height&&a.dpi==b.dpi;
}
std::wstring number(double value,int digits=4) {
    std::wostringstream out;out<<std::fixed<<std::setprecision(digits)<<value;return out.str();
}
int parse_integer(const std::wstring& value) {
    std::size_t end=0;const auto parsed=std::stoll(value,&end);
    if(end!=value.size()||parsed<0||parsed>std::numeric_limits<int>::max())throw std::invalid_argument("\u8bf7\u8f93\u5165\u975e\u8d1f\u6574\u6570");
    return static_cast<int>(parsed);
}
std::wstring target_name(RoiTarget target) {
    switch(target) {
    case RoiTarget::NW:return L"\u5de6\u4e0a\u89d2";case RoiTarget::N:return L"\u4e0a\u8fb9";
    case RoiTarget::NE:return L"\u53f3\u4e0a\u89d2";case RoiTarget::E:return L"\u53f3\u8fb9";
    case RoiTarget::SE:return L"\u53f3\u4e0b\u89d2";case RoiTarget::S:return L"\u4e0b\u8fb9";
    case RoiTarget::SW:return L"\u5de6\u4e0b\u89d2";case RoiTarget::W:return L"\u5de6\u8fb9";
    case RoiTarget::Move:return L"\u6574\u4e2a\u533a\u57df";
    }
    return L"";
}
std::wstring state_name(const Status& status) {
    if(status.state=="STREAMING")return L"\u6b63\u5728\u4f20\u8f93";
    if(status.state=="PREVIEW")return L"\u5df2\u6682\u505c";
    if(status.state=="CONNECTING")return L"\u6b63\u5728\u8fde\u63a5\u63a5\u6536\u7aef";
    if(status.state=="WAIT_WINDOW")return L"\u7b49\u5f85\u8d85\u58f0\u7a97\u53e3";
    if(status.state=="WAIT_FRAME")return L"\u7b49\u5f85\u753b\u9762";
    if(status.state=="STALE")return L"\u753b\u9762\u66f4\u65b0\u8d85\u65f6";
    if(status.state=="NEEDS_ROI")return L"\u56fe\u50cf\u533a\u57df\u5f85\u590d\u6838";
    if(status.state=="STOPPED")return L"\u672a\u5f00\u59cb\u9884\u89c8";
    return L"\u9884\u89c8\u5f02\u5e38";
}

struct Button {
    std::wstring label;
    std::optional<UiIcon> icon;
    bool text=true,primary=false,selected=false,tab=false;
};
struct TaskResult {
    Action action=Action::Pause;
    Navigation next=Navigation::None;
    bool failed=false;
    std::wstring message;
    std::optional<Profile> persisted;
    std::optional<Profile> source_started;
};
struct Job {
    Action action;
    Navigation next;
    std::function<void(TaskResult&)> run;
};
struct RulerDraft {
    std::array<std::optional<PixelPoint>,2> points;
    std::array<std::wstring,2> coordinates;
    std::wstring distance;
    int unit=0;
    bool checked=false,pending=false,invalid=false;
};

class Desktop {
public:
    explicit Desktop(HINSTANCE instance):instance_(instance){}
    ~Desktop() {
        {std::lock_guard lock(task_mutex_);quitting_=true;tasks_.clear();}
        task_changed_.notify_one();if(worker_.joinable())worker_.join();
        MSG stale{};while(PeekMessageW(&stale,hwnd_,kTaskDone,kTaskDone,PM_REMOVE))delete reinterpret_cast<TaskResult*>(stale.lParam);
        remove_tray();clear_icons();
        for(auto font:{font_,heading_font_,small_font_})if(font)DeleteObject(font);
        if(paper_brush_)DeleteObject(paper_brush_);
        if(band_brush_)DeleteObject(band_brush_);
    }
    int run(int show) {
        INITCOMMONCONTROLSEX common{sizeof(common),ICC_STANDARD_CLASSES|ICC_UPDOWN_CLASS};InitCommonControlsEx(&common);
        WNDCLASSEXW cls{};cls.cbSize=sizeof(cls);cls.hInstance=instance_;cls.hCursor=LoadCursorW(nullptr,IDC_ARROW);
        cls.hIcon=LoadIconW(nullptr,IDI_APPLICATION);cls.lpfnWndProc=main_proc;cls.lpszClassName=kMainClass;
        RegisterClassExW(&cls);cls.lpfnWndProc=body_proc;cls.lpszClassName=kBodyClass;RegisterClassExW(&cls);
        cls.lpfnWndProc=image_proc;cls.lpszClassName=kImageClass;RegisterClassExW(&cls);
        RECT work_area{};SystemParametersInfoW(SPI_GETWORKAREA,0,&work_area,0);
        const int width=std::min(1160L,work_area.right-work_area.left),height=std::min(900L,work_area.bottom-work_area.top);
        hwnd_=CreateWindowExW(0,kMainClass,L"\u8d85\u58f0\u56fe\u50cf\u63a5\u5165",WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,
            work_area.left+(work_area.right-work_area.left-width)/2,work_area.top+(work_area.bottom-work_area.top-height)/2,width,height,
            nullptr,nullptr,instance_,this);
        if(!hwnd_)throw std::runtime_error("Cannot create capture desktop");
        worker_=std::thread([this]{work();});
        load_saved();ShowWindow(hwnd_,show);UpdateWindow(hwnd_);SetTimer(hwnd_,kRefresh,100,nullptr);
        MSG message{};while(GetMessageW(&message,nullptr,0,0)>0){
            if(!IsDialogMessageW(hwnd_,&message)){TranslateMessage(&message);DispatchMessageW(&message);}
        }
        return static_cast<int>(message.wParam);
    }

private:
    HWND c(int id)const {auto it=controls_.find(id);return it==controls_.end()?nullptr:it->second;}
    int px(int value)const{return MulDiv(value,dpi_,96);}
    void text(int id,const std::wstring& value){if(window_text(c(id))!=value)SetWindowTextW(c(id),value.c_str());}
    bool checked(int id)const{return SendMessageW(c(id),BM_GETCHECK,0,0)==BST_CHECKED;}
    void check(int id,bool value){SendMessageW(c(id),BM_SETCHECK,value?BST_CHECKED:BST_UNCHECKED,0);}
    int selection(int id)const{return static_cast<int>(SendMessageW(c(id),CB_GETCURSEL,0,0));}
    void select(int id,int value){SendMessageW(c(id),CB_SETCURSEL,value,0);}
    HWND add(int id,const wchar_t* kind,const wchar_t* label,DWORD style=0,bool fixed_control=false) {
        const bool edit=std::wstring(kind)==L"EDIT";
        HWND child=CreateWindowExW(edit?WS_EX_CLIENTEDGE:0,kind,label,WS_CHILD|style,0,0,1,1,
            fixed_control?hwnd_:body_,reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),instance_,this);
        if(!child)throw std::runtime_error("Cannot create desktop control");
        controls_[id]=child;SendMessageW(child,WM_SETFONT,reinterpret_cast<WPARAM>(font_),TRUE);return child;
    }
    void label(int id,const wchar_t* value,bool fixed_control=false,bool small_text=false) {
        add(id,L"STATIC",value,SS_LEFT,fixed_control);
        if(small_text){small_labels_.push_back(id);SendMessageW(c(id),WM_SETFONT,reinterpret_cast<WPARAM>(small_font_),TRUE);}
    }
    void button(int id,const wchar_t* value,std::optional<UiIcon> icon={},bool with_text=true,bool primary=false,bool fixed_control=false,bool tab=false) {
        buttons_[id]={value,icon,with_text,primary,false,tab};
        HWND child=add(id,L"BUTTON",value,WS_TABSTOP|BS_OWNERDRAW,fixed_control);
        SetWindowSubclass(child,button_proc,1,reinterpret_cast<DWORD_PTR>(this));
        TOOLINFOW tool{};tool.cbSize=sizeof(tool);tool.uFlags=TTF_IDISHWND|TTF_SUBCLASS;tool.hwnd=hwnd_;
        tool.uId=reinterpret_cast<UINT_PTR>(child);tool.lpszText=const_cast<wchar_t*>(buttons_[id].label.c_str());
        SendMessageW(tooltip_,TTM_ADDTOOLW,0,reinterpret_cast<LPARAM>(&tool));
    }
    void button_label(int id,const std::wstring& value,std::optional<UiIcon> icon={}) {
        auto& b=buttons_.at(id);if(b.label==value&&b.icon==icon)return;b.label=value;b.icon=icon;
        SetWindowTextW(c(id),value.c_str());TOOLINFOW tool{};tool.cbSize=sizeof(tool);tool.hwnd=hwnd_;
        tool.uId=reinterpret_cast<UINT_PTR>(c(id));tool.lpszText=const_cast<wchar_t*>(b.label.c_str());
        SendMessageW(tooltip_,TTM_UPDATETIPTEXTW,0,reinterpret_cast<LPARAM>(&tool));InvalidateRect(c(id),nullptr,TRUE);
    }
    void combo(int id,std::initializer_list<const wchar_t*> values) {
        add(id,L"COMBOBOX",L"",WS_TABSTOP|CBS_DROPDOWNLIST);
        for(const auto* value:values)SendMessageW(c(id),CB_ADDSTRING,0,reinterpret_cast<LPARAM>(value));
        select(id,0);
    }
    void make_fonts() {
        auto make=[&](int point,int weight){return CreateFontW(-MulDiv(point,dpi_,72),0,0,0,weight,FALSE,FALSE,FALSE,
            DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");};
        HFONT old=font_,old_heading=heading_font_,old_small=small_font_;
        font_=make(10,FW_NORMAL);heading_font_=make(12,FW_MEDIUM);small_font_=make(9,FW_NORMAL);
        for(const auto& item:controls_)SendMessageW(item.second,WM_SETFONT,reinterpret_cast<WPARAM>(font_),TRUE);
        for(int id:small_labels_)SendMessageW(c(id),WM_SETFONT,reinterpret_cast<WPARAM>(small_font_),TRUE);
        for(int id:{Brand,PageTitle,MainSource})if(c(id))SendMessageW(c(id),WM_SETFONT,reinterpret_cast<WPARAM>(heading_font_),TRUE);
        for(auto font:{old,old_heading,old_small})if(font)DeleteObject(font);
        clear_icons();
    }
    void create_controls() {
        dpi_=GetDpiForWindow(hwnd_);make_fonts();paper_brush_=CreateSolidBrush(kPaper);band_brush_=CreateSolidBrush(kBand);
        body_=CreateWindowExW(WS_EX_CONTROLPARENT,kBodyClass,L"",WS_CHILD|WS_VISIBLE|WS_CLIPCHILDREN|WS_VSCROLL,0,0,1,1,hwnd_,nullptr,instance_,this);
        tooltip_=CreateWindowExW(WS_EX_TOPMOST,TOOLTIPS_CLASSW,nullptr,WS_POPUP|TTS_ALWAYSTIP|TTS_NOPREFIX,
            CW_USEDEFAULT,CW_USEDEFAULT,CW_USEDEFAULT,CW_USEDEFAULT,hwnd_,nullptr,instance_,nullptr);
        SendMessageW(tooltip_,TTM_SETMAXTIPWIDTH,0,px(320));
        label(Brand,L"\u8d85\u58f0\u56fe\u50cf\u63a5\u5165",true);label(PageTitle,L"",true);label(DirtyState,L"",true,true);
        label(StatusText,L"\u672a\u5f00\u59cb\u9884\u89c8",true,true);label(Receipt,L"",true,true);label(FooterHint,L"",true,true);
        label(MainSource,L"\u672a\u914d\u7f6e\u8d85\u58f0\u8f6f\u4ef6",true);label(MainCaption,L"\u8f93\u51fa\u9884\u89c8",true,true);
        label(MainMeta,L"",false,true);label(MainNotice,L"",false,true);
        button(OpenRoi,L"\u8c03\u6574\u56fe\u50cf\u533a\u57df",UiIcon::Crop,true,false,true);button(OpenSettings,L"\u8bbe\u5907\u8bbe\u7f6e",UiIcon::Settings,false,false,true);
        button(Back,L"\u8fd4\u56de",UiIcon::Back,false,false,true);button(Transmit,L"\u5f00\u59cb\u4f20\u8f93",UiIcon::Play,true,true,true);
        button(StartPreview,L"\u6253\u5f00\u8d85\u58f0\u8f6f\u4ef6",UiIcon::Refresh,true);
        button(Cancel,L"\u53d6\u6d88",{},true,false,true);button(SaveDraft,L"\u4fdd\u5b58",UiIcon::Save,true,true,true);
        add(MainImage,kImageClass,L"",0);
        button(TabSource,L"\u6765\u6e90",{},true,false,true,true);button(TabReceiver,L"\u63a5\u6536\u7aef",{},true,false,true,true);button(TabStartup,L"\u542f\u52a8\u884c\u4e3a",{},true,false,true,true);
        for(int id:{Executable,WorkingDir,ProcessName,Title,Host,Port,Context,RoiX,RoiY,RoiWidth,RoiHeight,RulerA,RulerB,RulerDistance})add(id,L"EDIT",L"",WS_TABSTOP|ES_AUTOHSCROLL);
        button(BrowseExe,L"\u9009\u62e9\u8d85\u58f0\u8f6f\u4ef6",UiIcon::FolderOpen,false);button(BrowseDir,L"\u9009\u62e9\u5de5\u4f5c\u76ee\u5f55",UiIcon::FolderOpen,false);
        button(Advanced,L"\u9ad8\u7ea7\u8bbe\u7f6e",{},true);button(ReloadConfig,L"\u91cd\u65b0\u8bfb\u53d6\u914d\u7f6e",UiIcon::Refresh,true);
        for(const auto& item:std::array<std::pair<int,const wchar_t*>,4>{{
            {LaunchVendor,L"\u542f\u52a8\u65f6\u6253\u5f00\u8d85\u58f0\u8f6f\u4ef6"},{AutoConnect,L"\u542f\u52a8\u65f6\u81ea\u52a8\u4f20\u8f93"},
            {MinimizeToTray,L"\u5173\u95ed\u7a97\u53e3\u65f6\u7559\u5728\u6258\u76d8"},{Confirm,L"\u9009\u533a\u5b8c\u6574\uff0c\u672a\u5305\u542b\u6587\u5b57\u548c\u63a7\u4ef6"}}})
            add(item.first,L"BUTTON",item.second,WS_TABSTOP|BS_AUTOCHECKBOX|BS_MULTILINE);
        label(ConfigPath,L"",false,true);label(SettingsError,L"",false,true);
        label(1000,L"\u8d85\u58f0\u8f6f\u4ef6");label(1001,L"\u5de5\u4f5c\u76ee\u5f55");label(1002,L"\u8fdb\u7a0b\u540d");label(1003,L"\u7a97\u53e3\u6807\u9898\u5305\u542b");
        label(1004,L"\u63a5\u6536\u7535\u8111\u5730\u5740");label(1005,L"\u7aef\u53e3");label(1006,L"\u6210\u50cf\u8bbe\u7f6e\u5907\u6ce8");label(1007,L"\u914d\u7f6e\u6587\u4ef6",false,true);
        button(Draw,L"\u91cd\u65b0\u6846\u9009",UiIcon::Draw,false,false,true);button(Move,L"\u79fb\u52a8\u6216\u8c03\u6574\u8fb9\u754c",UiIcon::Move,false,false,true);
        button(Undo,L"\u64a4\u9500\u9009\u533a\u4fee\u6539",UiIcon::Undo,false,false,true);button(RestoreRoi,L"\u6062\u590d\u5df2\u4fdd\u5b58\u9009\u533a",UiIcon::Restore,false,false,true);
        button(FullWindow,L"\u9009\u6574\u4e2a\u7a97\u53e3",UiIcon::FullWindow,false,false,true);button(RefreshReference,L"\u66f4\u65b0\u53c2\u8003\u5e27",UiIcon::Refresh,false,false,true);
        label(SourceText,L"",true,true);add(FullView,kImageClass,L"",WS_TABSTOP);add(CropView,kImageClass,L"",0);
        label(CropHeading,L"\u88c1\u526a\u9884\u89c8");label(RoiText,L"",false,true);
        label(1010,L"\u5de6 / px",false,true);label(1011,L"\u4e0a / px",false,true);label(1012,L"\u5bbd / px",false,true);label(1013,L"\u9ad8 / px",false,true);
        label(MagnifierTitle,L"\u8fb9\u754c\u5c40\u90e8\u653e\u5927");label(MagnifierPosition,L"",false,true);
        combo(InspectorKind,{L"\u9009\u533a\u8fb9\u754c",L"\u6807\u5c3a\u523b\u5ea6"});combo(ActiveEdge,{L"\u5de6\u4e0a\u89d2",L"\u4e0a\u8fb9",L"\u53f3\u4e0a\u89d2",L"\u53f3\u8fb9",L"\u53f3\u4e0b\u89d2",L"\u4e0b\u8fb9",L"\u5de6\u4e0b\u89d2",L"\u5de6\u8fb9",L"\u6574\u4e2a\u533a\u57df"});
        combo(ActiveTick,{L"\u523b\u5ea6 A",L"\u523b\u5ea6 B"});combo(Zoom,{L"4x",L"8x"});
        add(MagnifierView,kImageClass,L"",WS_TABSTOP);
        button(NudgeLeft,L"\u5411\u5de6 1 px",UiIcon::Left,false);button(NudgeUp,L"\u5411\u4e0a 1 px",UiIcon::Up,false);
        button(NudgeDown,L"\u5411\u4e0b 1 px",UiIcon::Down,false);button(NudgeRight,L"\u5411\u53f3 1 px",UiIcon::Right,false);
        button(ScaleExpanded,L"\u5c3a\u5bf8\u6bd4\u4f8b\uff08\u53ef\u9009\uff09",UiIcon::Ruler,true);label(ScaleHeading,L"\u5c3a\u5bf8\u6bd4\u4f8b");label(RulerNote,L"",false,true);
        combo(RulerAxis,{L"\u7eb5\u5411",L"\u6a2a\u5411"});button(PickRuler,L"\u9009\u4e24\u523b\u5ea6",UiIcon::Crosshair,true);
        label(1020,L"\u523b\u5ea6 A / px",false,true);label(1021,L"\u523b\u5ea6 B / px",false,true);label(1022,L"\u4e24\u523b\u5ea6\u8ddd\u79bb",false,true);
        combo(RulerUnit,{L"mm",L"cm"});add(UnitConfirmed,L"BUTTON",L"\u5df2\u6838\u5bf9\u523b\u5ea6\u8ddd\u79bb\u4e0e\u5355\u4f4d",WS_TABSTOP|BS_AUTOCHECKBOX);
        button(RecordRuler,L"\u8bb0\u5f55\u6bd4\u4f8b",UiIcon::Check,true);button(ClearRuler,L"\u6e05\u9664\u672c\u8f74",UiIcon::Trash,true);
        label(RulerText,L"",false,true);label(RoiError,L"",false,true);
        for(int i=0;i<4;++i){
            HWND spin=add(500+i,UPDOWN_CLASSW,L"",UDS_ALIGNRIGHT|UDS_SETBUDDYINT|UDS_ARROWKEYS|UDS_NOTHOUSANDS);
            SendMessageW(spin,UDM_SETBUDDY,reinterpret_cast<WPARAM>(c(RoiX+i)),0);SendMessageW(spin,UDM_SETRANGE32,0,32768);
        }
        make_fonts();add_tray();taskbar_message_=RegisterWindowMessageW(L"TaskbarCreated");layout();refresh_controls();
    }
    void show(int id,bool visible){ShowWindow(c(id),visible?SW_SHOW:SW_HIDE);}
    void fixed(int id,int x,int y,int w,int h){show(id,true);MoveWindow(c(id),px(x),px(y),px(std::max(1,w)),px(h),TRUE);}
    void place(int id,int x,int y,int w,int h){fixed(id,x,y-scroll_,w,h);}
    void layout() {
        if(laying_out_||!body_)return;
        laying_out_=true;
        for(const auto& item:controls_)ShowWindow(item.second,SW_HIDE);
        RECT client{};GetClientRect(hwnd_,&client);const int w=MulDiv(client.right,96,dpi_),h=MulDiv(client.bottom,96,dpi_);
        const int footer=76,margin=20;
        fixed(Brand,20,18,w-310,26);fixed(StatusText,w-280,21,260,22);
        int body_top=118;
        if(page_==Page::Main){
            fixed(MainSource,margin,70,w-350,25);fixed(MainCaption,margin,96,w-350,18);
            fixed(OpenRoi,w-220,70,150,36);fixed(OpenSettings,w-60,70,40,36);
            fixed(Transmit,w-154,h-57,134,38);
        }else{
            fixed(Back,margin,70,36,36);fixed(PageTitle,68,76,180,25);fixed(DirtyState,250,79,100,20);
            fixed(Cancel,w-260,h-57,112,38);fixed(SaveDraft,w-138,h-57,118,38);
            if(page_==Page::Settings){
                fixed(TabSource,20,116,76,34);fixed(TabReceiver,112,116,76,34);fixed(TabStartup,204,116,92,34);body_top=160;
            }else{
                int x=20;for(int id:{Draw,Move,Undo,RestoreRoi,FullWindow}){fixed(id,x,119,36,36);x+=42;}
                fixed(RefreshReference,w-58,119,36,36);fixed(SourceText,245,127,w-313,22);body_top=168;
            }
        }
        fixed(FooterHint,20,h-59,std::max(100,w-330),20);fixed(Receipt,20,h-36,std::max(100,w-330),24);
        MoveWindow(body_,0,px(body_top),client.right,std::max(px(80),static_cast<int>(client.bottom)-px(body_top+footer)),TRUE);
        ShowScrollBar(body_,SB_VERT,page_!=Page::Main);
        RECT area{};GetClientRect(body_,&area);const int bw=MulDiv(area.right,96,dpi_),bh=MulDiv(area.bottom,96,dpi_);
        content_height_=bh;
        if(page_==Page::Main)layout_main(bw,bh);else if(page_==Page::Settings)layout_settings(bw);else layout_roi(bw);
        const int max_scroll=std::max(0,content_height_-bh);
        if(scroll_>max_scroll){scroll_=max_scroll;laying_out_=false;layout();return;}
        SCROLLINFO si{};si.cbSize=sizeof(si);si.fMask=SIF_RANGE|SIF_PAGE|SIF_POS|SIF_DISABLENOSCROLL;
        si.nMin=0;si.nMax=std::max(0,content_height_-1);si.nPage=bh;si.nPos=scroll_;SetScrollInfo(body_,SB_VERT,&si,TRUE);
        if(page_==Page::Main)ShowScrollBar(body_,SB_VERT,FALSE);
        laying_out_=false;InvalidateRect(hwnd_,nullptr,TRUE);InvalidateRect(body_,nullptr,TRUE);
    }
    void layout_main(int w,int h) {
        const bool retry=!frame_||!current_source()||status_.state=="ERROR"||status_.state=="CAPTURE_ERROR";
        const bool notice=profile_read_error_||runtime_review_required_||!saved_.roi_confirmed||retry;
        const int top=notice?34:0;
        if(notice)place(MainNotice,20,2,w-(retry?224:40),28);
        if(retry)place(StartPreview,w-190,0,170,30);
        place(MainImage,20,top,w-40,std::max(170,h-top-58));
        place(MainMeta,20,std::max(170,h-44),w-40,34);content_height_=h;
    }
    void layout_settings(int w) {
        const int width=std::min(760,w-40),x=20;int y=12;
        if(setting_tab_==SettingTab::Source){
            place(1000,x,y,width,24);y+=29;place(Executable,x,y,width-46,34);place(BrowseExe,x+width-38,y,38,34);y+=50;
            place(Advanced,x,y,138,34);y+=48;
            if(advanced_){
                place(1001,x,y,width,23);y+=28;place(WorkingDir,x,y,width-46,34);place(BrowseDir,x+width-38,y,38,34);y+=50;
                const int half=(width-16)/2;
                place(1002,x,y,half,23);place(1003,x+half+16,y,half,23);y+=28;
                place(ProcessName,x,y,half,34);place(Title,x+half+16,y,half,34);y+=50;
                place(1006,x,y,width,23);y+=28;place(Context,x,y,width,34);y+=50;
                place(1007,x,y,width,20);y+=25;place(ConfigPath,x,y,width,48);y+=56;
                if(profile_read_error_){place(ReloadConfig,x,y,160,34);y+=44;}
            }
        }else if(setting_tab_==SettingTab::Receiver){
            place(1004,x,y,width-140,23);place(1005,x+width-120,y,120,23);y+=28;
            place(Host,x,y,width-140,36);place(Port,x+width-120,y,120,36);y+=54;
        }else{
            place(LaunchVendor,x,y,width,34);y+=48;place(AutoConnect,x,y,width,34);y+=48;
            place(MinimizeToTray,x,y,width,34);y+=48;
        }
        place(SettingsError,x,y,width,58);content_height_=y+78;
    }
    void layout_roi(int w) {
        const int margin=20,gap=20,aside=260;
        const bool narrow=w<900;
        const int left_width=narrow?w-40:w-40-gap-aside;
        const int ref_w=reference_?reference_->width:1420,ref_h=reference_?reference_->height:810;
        const int view_height=std::clamp(left_width*ref_h/std::max(1,ref_w),180,520);
        place(FullView,margin,8,left_width,view_height);
        const int col=(left_width-30)/4,y=view_height+23;
        for(int i=0;i<4;++i){
            const int x=margin+i*(col+10);place(1010+i,x,y,col,21);place(RoiX+i,x,y+27,col,34);
            place(500+i,x+col-18,y+27,18,34);
        }
        const int ax=narrow?margin:margin+left_width+gap;
        const int aw=narrow?w-40:aside;
        const int ay=narrow?y+88:8;
        place(CropHeading,ax,ay,aw,23);place(CropView,ax,ay+30,aw,140);place(RoiText,ax,ay+180,aw,22);
        place(MagnifierTitle,ax,ay+218,aw,23);
        place(InspectorKind,ax,ay+250,aw-80,150);place(Zoom,ax+aw-72,ay+250,72,100);
        place(ActiveTick,ax,ay+290,aw,100);place(ActiveEdge,ax,ay+290,aw,290);
        show(ActiveTick,inspect_ruler_);show(ActiveEdge,!inspect_ruler_);
        const int size=192;place(MagnifierView,ax+(aw-size)/2,ay+334,size,size);
        place(MagnifierPosition,ax,ay+536,aw,30);
        int nx=ax+(aw-168)/2;for(int id:{NudgeLeft,NudgeUp,NudgeDown,NudgeRight}){place(id,nx,ay+572,36,36);nx+=44;}
        place(Confirm,ax,ay+620,aw,43);
        int end=std::max(y+82,ay+680);
        place(ScaleExpanded,margin,end,w-40,36);end+=46;
        if(scale_expanded_){
            const int form_width=std::min(w-40,760);
            place(RulerAxis,margin,end,130,120);place(PickRuler,margin+142,end,134,34);place(ClearRuler,margin+288,end,132,34);end+=48;
            place(1020,margin,end,145,21);place(1021,margin+160,end,145,21);place(1022,margin+320,end,180,21);end+=26;
            place(RulerA,margin,end,145,34);place(RulerB,margin+160,end,145,34);place(RulerDistance,margin+320,end,120,34);place(RulerUnit,margin+450,end,80,100);end+=46;
            place(UnitConfirmed,margin,end,std::max(200,form_width-150),32);place(RecordRuler,margin+form_width-140,end,140,34);end+=46;
            place(RulerText,margin,end,form_width,40);end+=48;place(RulerNote,margin,end,form_width,36);end+=44;
        }
        place(RoiError,margin,end,w-40,50);content_height_=end+62;
    }

    Profile source_profile(Profile p)const {
        if(p.process_name.empty()&&p.title_contains.empty()&&!p.executable.empty())
            p.process_name=utf8(std::filesystem::path(wide(p.executable)).filename().wstring());
        return p;
    }
    bool configured(const Profile& p)const {
        const auto source=source_profile(p);return !source.process_name.empty()||!source.title_contains.empty();
    }
    bool current_source()const {
        return source_started_&&source_key(source_profile(saved_))==active_source_;
    }
    bool usable_reference()const {
        return reference_&&frame_&&current_source()&&!reference_changed_&&same_shape(*reference_,*frame_);
    }
    bool ready_to_transmit()const {
        return !profile_read_error_&&!runtime_review_required_&&frame_&&current_source()&&matches(saved_,*frame_)&&status_.frame_age_s>=0&&status_.frame_age_s<=1.0;
    }
    void fail(const std::exception& e) {
        error_=L"\u672a\u5b8c\u6210\uff1a"+wide(e.what());refresh_controls();
    }
    void work() {
        const HRESULT apartment=RoInitialize(RO_INIT_MULTITHREADED);
        for(;;){
            Job job;
            {std::unique_lock lock(task_mutex_);task_changed_.wait(lock,[&]{return quitting_||!tasks_.empty();});
                if(quitting_)break;
                job=std::move(tasks_.front());tasks_.pop_front();}
            auto result=std::make_unique<TaskResult>();result->action=job.action;result->next=job.next;
            try{job.run(*result);}catch(const std::exception& e){result->failed=true;result->message=wide(e.what());}
            catch(...){result->failed=true;result->message=L"Windows \u64cd\u4f5c\u5931\u8d25";}
            if(PostMessageW(hwnd_,kTaskDone,0,reinterpret_cast<LPARAM>(result.get())))result.release();
        }
        controller_.stop();if(SUCCEEDED(apartment))RoUninitialize();
    }
    void queue(Action action,std::function<void(TaskResult&)> job,Navigation next=Navigation::None) {
        {std::lock_guard lock(task_mutex_);tasks_.push_back({action,next,std::move(job)});}
        ++pending_;error_.clear();refresh_controls();task_changed_.notify_one();
    }
    void load_saved() {
        try{
            const auto path=default_profile_path();Profile loaded;
            if(std::filesystem::exists(path))loaded=load_profile(path);
            saved_=loaded;draft_=saved_;profile_read_error_=false;error_.clear();
            notice_=std::filesystem::exists(path)?L"\u914d\u7f6e\u5df2\u8bfb\u53d6":L"\u8bf7\u8bbe\u7f6e\u8d85\u58f0\u8f6f\u4ef6";
            runtime_review_required_=false;
            if(configured(saved_))start_preview(saved_,true);
        }catch(const std::exception& e){
            profile_read_error_=true;pending_auto_=false;
            error_=L"\u914d\u7f6e\u8bfb\u53d6\u5931\u8d25\uff0c\u539f\u6587\u4ef6\u5df2\u4fdd\u7559\uff1a"+wide(e.what());
        }
        update_main();layout();refresh_controls();
    }
    void start_preview(const Profile& value,bool startup=false,Navigation after=Navigation::None) {
        if(pending_||profile_read_error_)return;
        if(after!=Navigation::Roi){enter_roi_when_ready_=false;deferred_roi_source_.clear();}
        const auto p=source_profile(value);
        try{
            validate_profile(p,false);if(!configured(p))throw std::invalid_argument("\u8bf7\u5148\u9009\u62e9\u8d85\u58f0\u8f6f\u4ef6\u6216\u7a97\u53e3");
            pending_auto_=startup&&p.auto_connect&&p.roi_confirmed;
            send_requested_=false;
            queue(Action::Preview,[this,p](TaskResult& r){
                controller_.set_transmitting(false);controller_.start(p,p.launch_vendor);
                r.source_started=p;r.message=L"\u6b63\u5728\u83b7\u53d6\u7a97\u53e3\u753b\u9762";
            },after);
        }catch(const std::exception& e){fail(e);}
    }
    void pause(Navigation after=Navigation::None) {
        pending_auto_=false;send_requested_=false;
        queue(after==Navigation::Roi?Action::EnterRoi:Action::Pause,[this](TaskResult& r){
            controller_.set_transmitting(false);r.message=L"\u4f20\u8f93\u5df2\u6682\u505c";
        },after);
    }
    void toggle_transmission() {
        if(pending_)return;
        if(status_.transmitting||send_requested_){pause();return;}
        try{
            if(!ready_to_transmit())throw std::invalid_argument("\u8bf7\u5148\u590d\u6838\u5e76\u4fdd\u5b58\u5f53\u524d\u56fe\u50cf\u533a\u57df");
            const auto p=source_profile(saved_);validate_profile(p,true);send_requested_=true;pending_auto_=false;
            queue(Action::Transmit,[this,p](TaskResult& r){controller_.apply_profile(p);controller_.set_transmitting(true);r.message=L"\u6b63\u5728\u8fde\u63a5\u63a5\u6536\u7aef";});
        }catch(const std::exception& e){send_requested_=false;fail(e);}
    }
    Profile settings_form()const {
        Profile p=draft_;
        p.executable=utf8(window_text(c(Executable)));p.working_directory=utf8(window_text(c(WorkingDir)));
        p.process_name=utf8(window_text(c(ProcessName)));p.title_contains=utf8(window_text(c(Title)));
        p.host=utf8(window_text(c(Host)));p.port=parse_integer(window_text(c(Port)));
        p.imaging_context=utf8(window_text(c(Context)));p.launch_vendor=checked(LaunchVendor);
        p.auto_connect=checked(AutoConnect);p.minimize_to_tray=checked(MinimizeToTray);
        return p;
    }
    Profile prepared_settings()const {
        auto p=source_profile(settings_form());
        if(source_key(p)!=source_key(source_profile(saved_))||p.imaging_context!=saved_.imaging_context||runtime_review_required_){
            p.roi_confirmed=false;p.auto_connect=false;p.horizontal_scale.reset();p.vertical_scale.reset();
        }
        return p;
    }
    void populate_settings() {
        syncing_=true;
        text(Executable,wide(draft_.executable));text(WorkingDir,wide(draft_.working_directory));
        text(ProcessName,wide(draft_.process_name));text(Title,wide(draft_.title_contains));
        text(Host,wide(draft_.host));text(Port,std::to_wstring(draft_.port));text(Context,wide(draft_.imaging_context));
        check(LaunchVendor,draft_.launch_vendor);check(AutoConnect,draft_.auto_connect);check(MinimizeToTray,draft_.minimize_to_tray);
        text(ConfigPath,default_profile_path().wstring());syncing_=false;
    }
    void settings_changed(int id) {
        if(page_!=Page::Settings||syncing_)return;
        if(id==Executable){
            const std::filesystem::path old_path(wide(draft_.executable)),next_path(window_text(c(Executable)));
            syncing_=true;
            const auto old_parent=old_path.parent_path().wstring(),old_name=old_path.filename().wstring();
            if(window_text(c(WorkingDir)).empty()||window_text(c(WorkingDir))==old_parent)text(WorkingDir,next_path.parent_path().wstring());
            if(window_text(c(ProcessName)).empty()||window_text(c(ProcessName))==old_name)text(ProcessName,next_path.filename().wstring());
            syncing_=false;
            draft_.executable=utf8(window_text(c(Executable)));
        }
        try{draft_=settings_form();invalid_settings_=false;validate_profile(prepared_settings(),false);text(SettingsError,L"");}
        catch(const std::exception& e){invalid_settings_=true;text(SettingsError,wide(e.what()));}
        update_dirty();refresh_controls();
    }
    void update_dirty() {
        dirty_=page_!=Page::Main&&(!same_profile(draft_,saved_)||(page_==Page::Settings&&invalid_settings_)||
            (page_==Page::Roi&&(invalid_roi_||rulers_[0].pending||rulers_[1].pending||review_touched_||runtime_review_required_)));
        text(DirtyState,dirty_?L"\u672a\u4fdd\u5b58":L"\u5df2\u4fdd\u5b58");
    }
    bool can_save()const {
        if(profile_read_error_||pending_||!dirty_)return false;
        if(page_==Page::Settings)return !invalid_settings_;
        return page_==Page::Roi&&usable_reference()&&!invalid_roi_&&!rulers_[0].pending&&!rulers_[1].pending&&
            !rulers_[0].invalid&&!rulers_[1].invalid&&draft_.roi_confirmed&&valid_rect(draft_.roi,reference_->width,reference_->height);
    }
    void save_draft(Navigation after=Navigation::Main) {
        if(!can_save()){error_=L"\u8bf7\u68c0\u67e5\u672a\u5b8c\u6210\u7684\u8bbe\u7f6e\u6216\u56fe\u50cf\u533a\u57df\u786e\u8ba4";refresh_controls();return;}
        try{
            Profile p=page_==Page::Settings?prepared_settings():draft_;
            const bool roi=page_==Page::Roi;validate_profile(p,roi);
            p.revision=std::to_string(GetTickCount64());
            const bool source_changed=source_key(source_profile(p))!=source_key(source_profile(saved_));
            const bool pipeline_changed=roi||source_changed||p.host!=saved_.host||p.port!=saved_.port||p.imaging_context!=saved_.imaging_context;
            const bool start_needed=configured(p)&&(source_changed||!source_started_);
            const bool closing=after==Navigation::Exit||(after==Navigation::Close&&!p.minimize_to_tray);
            const bool apply_existing=source_started_;
            if(pipeline_changed){send_requested_=false;pending_auto_=false;}
            queue(roi?Action::SaveRoi:Action::SaveSettings,[this,p,roi,pipeline_changed,start_needed,closing,apply_existing](TaskResult& r){
                if(pipeline_changed)controller_.set_transmitting(false);
                if(roi){
                    const auto latest=controller_.preview();const auto status=controller_.status();
                    if(!latest||!matches(p,*latest)||status.frame_age_s<0||status.frame_age_s>2.0)
                        throw std::runtime_error("\u5f53\u524d\u7a97\u53e3\u5c3a\u5bf8\u3001DPI \u6216\u66f4\u65b0\u72b6\u6001\u5df2\u53d8\u5316\uff0c\u8bf7\u66f4\u65b0\u53c2\u8003\u5e27\u540e\u91cd\u65b0\u786e\u8ba4");
                }
                save_profile(default_profile_path(),p);r.persisted=p;
                if(!closing&&start_needed){auto source=source_profile(p);controller_.start(source,source.launch_vendor);r.source_started=source;}
                else if(!closing&&pipeline_changed&&apply_existing)controller_.apply_profile(source_profile(p));
                r.message=roi?L"\u56fe\u50cf\u533a\u57df\u5df2\u4fdd\u5b58\uff0c\u4f20\u8f93\u4fdd\u6301\u6682\u505c":L"\u8bbe\u7f6e\u5df2\u4fdd\u5b58";
            },after);
        }catch(const std::exception& e){fail(e);}
    }
    static HRESULT CALLBACK unsaved_callback(HWND dialog,UINT notification,WPARAM,LPARAM,LONG_PTR allow_save) {
        if(notification==TDN_CREATED&&!allow_save)SendMessageW(dialog,TDM_ENABLE_BUTTON,IDYES,FALSE);
        return S_OK;
    }
    int unsaved_choice() {
        TASKDIALOG_BUTTON choices[]={{IDYES,L"\u4fdd\u5b58"},{IDNO,L"\u653e\u5f03\u4fee\u6539"},{IDCANCEL,L"\u7ee7\u7eed\u7f16\u8f91"}};
        TASKDIALOGCONFIG config{};config.cbSize=sizeof(config);config.hwndParent=hwnd_;config.hInstance=instance_;
        config.dwFlags=TDF_ALLOW_DIALOG_CANCELLATION|TDF_POSITION_RELATIVE_TO_WINDOW;
        config.pszWindowTitle=L"\u672a\u4fdd\u5b58\u7684\u4fee\u6539";config.pszMainInstruction=L"\u5f53\u524d\u4fee\u6539\u5c1a\u672a\u4fdd\u5b58";
        config.cButtons=3;config.pButtons=choices;config.nDefaultButton=IDCANCEL;
        config.pfCallback=unsaved_callback;config.lpCallbackData=can_save()?1:0;
        int selected=IDCANCEL;
        if(FAILED(TaskDialogIndirect(&config,&selected,nullptr,nullptr)))
            selected=MessageBoxW(hwnd_,L"\u4fdd\u5b58\u5f53\u524d\u4fee\u6539\uff1f\n\u9009\u62e9\u201c\u5426\u201d\u653e\u5f03\u4fee\u6539\uff0c\u9009\u62e9\u201c\u53d6\u6d88\u201d\u7ee7\u7eed\u7f16\u8f91\u3002",L"\u672a\u4fdd\u5b58\u7684\u4fee\u6539",MB_YESNOCANCEL|MB_ICONQUESTION);
        return selected;
    }
    void request_navigation(Navigation next) {
        if(next!=Navigation::Roi){enter_roi_when_ready_=false;deferred_roi_source_.clear();}
        if(pending_){error_=L"\u64cd\u4f5c\u5c1a\u672a\u5b8c\u6210\uff0c\u8bf7\u7a0d\u5019";refresh_controls();return;}
        update_dirty();
        if(dirty_){
            const int choice=unsaved_choice();
            if(choice==IDYES){save_draft(next);return;}
            if(choice!=IDNO)return;
        }
        navigate(next);
    }
    void navigate(Navigation next) {
        if(next==Navigation::None)return;
        if(next==Navigation::Exit||(next==Navigation::Close&&!saved_.minimize_to_tray)){shutdown();return;}
        if(next==Navigation::Close){activate(Page::Main);if(tray_added_)ShowWindow(hwnd_,SW_HIDE);else shutdown();return;}
        if(next==Navigation::Roi){
            if(!frame_||!current_source()){
                if(configured(saved_)){start_preview(saved_,false,Navigation::Roi);return;}
                error_=L"\u8bf7\u5148\u914d\u7f6e\u5e76\u6253\u5f00\u8d85\u58f0\u8f6f\u4ef6";activate(Page::Settings);return;
            }
            if(status_.transmitting||send_requested_){pause(Navigation::Roi);return;}
            activate(Page::Roi);return;
        }
        activate(next==Navigation::Settings?Page::Settings:Page::Main);
    }
    void activate(Page page) {
        enter_roi_when_ready_=false;deferred_roi_source_.clear();
        page_=page;draft_=saved_;dirty_=false;invalid_roi_=false;invalid_settings_=false;scroll_=0;
        history_.clear();dragging_=false;review_touched_=false;
        if(GetCapture()==c(FullView))ReleaseCapture();
        if(page==Page::Settings){setting_tab_=SettingTab::Source;populate_settings();}
        else if(page==Page::Roi){
            pending_auto_=false;
            reference_=frame_;reference_changed_=false;mode_=EditMode::Move;inspect_ruler_=false;active_edge_=RoiTarget::NW;
            if(!reference_){error_=L"\u5c1a\u672a\u53d6\u5f97\u7a97\u53e3\u753b\u9762";page_=Page::Main;}
            else{
                if(runtime_review_required_||draft_.reference_width!=reference_->width||draft_.reference_height!=reference_->height||draft_.reference_dpi!=reference_->dpi){
                    draft_.roi_confirmed=false;draft_.auto_connect=false;draft_.horizontal_scale.reset();draft_.vertical_scale.reset();
                }
                draft_.reference_width=reference_->width;draft_.reference_height=reference_->height;draft_.reference_dpi=reference_->dpi;
                if(!valid_rect(draft_.roi,reference_->width,reference_->height))draft_.roi={0,0,reference_->width,reference_->height};
                load_rulers();sync_roi();rebuild_edit_crop();
            }
        }
        if(page_==Page::Main){reference_.reset();rebuild_main_crop();}
        update_dirty();update_main();layout();refresh_controls();
        SetFocus(page_==Page::Roi?c(FullView):page_==Page::Settings?c(Executable):c(Transmit));
    }
    void task_done(std::unique_ptr<TaskResult> result) {
        pending_=std::max(0,pending_-1);
        if(result->source_started){active_source_=source_key(*result->source_started);source_started_=true;frame_=controller_.preview();}
        if(result->persisted){
            const bool invalidated=!result->persisted->roi_confirmed;
            saved_=*result->persisted;draft_=saved_;dirty_=false;review_touched_=false;
            if(result->action==Action::SaveRoi)runtime_review_required_=!frame_||!matches(saved_,*frame_);
            else if(invalidated)runtime_review_required_=true;
            if(page_==Page::Settings)populate_settings();
        }
        if(result->failed){error_=L"\u672a\u5b8c\u6210\uff1a"+result->message;pending_auto_=false;send_requested_=false;}
        else{error_.clear();notice_=result->message;}
        status_=controller_.status();update_main();refresh_controls();
        if(result->action==Action::Shutdown){DestroyWindow(hwnd_);return;}
        if(!result->failed||result->persisted){
            if(result->action==Action::EnterRoi){send_requested_=false;activate(Page::Roi);}
            else if(result->next==Navigation::Roi&&!frame_&&page_==Page::Main){enter_roi_when_ready_=true;deferred_roi_source_=active_source_;}
            else navigate(result->next);
        }
        layout();
    }
    void shutdown() {
        if(exiting_)return;
        exiting_=true;pending_auto_=false;send_requested_=false;KillTimer(hwnd_,kRefresh);
        queue(Action::Shutdown,[this](TaskResult&){controller_.stop();});
    }
    void refresh_reference() {
        if(page_!=Page::Roi||pending_)return;
        const auto latest=controller_.preview();
        if(!latest){error_=L"\u5f53\u524d\u6ca1\u6709\u53ef\u7528\u7a97\u53e3\u753b\u9762";refresh_controls();return;}
        const bool different=!reference_||!same_shape(*reference_,*latest);
        reference_=latest;frame_=latest;reference_changed_=false;draft_.roi_confirmed=false;draft_.auto_connect=false;review_touched_=true;
        draft_.reference_width=latest->width;draft_.reference_height=latest->height;draft_.reference_dpi=latest->dpi;
        if(different){draft_.horizontal_scale.reset();draft_.vertical_scale.reset();history_.clear();load_rulers();}
        if(!valid_rect(draft_.roi,latest->width,latest->height))draft_.roi={0,0,latest->width,latest->height};
        invalid_roi_=false;sync_roi();rebuild_edit_crop();error_.clear();update_dirty();layout();refresh_controls();invalidate_editor();
    }
    void tick() {
        status_=controller_.status();auto next=controller_.preview();
        if(next&&next!=frame_){
            const bool changed=frame_&&!same_shape(*frame_,*next);frame_=std::move(next);
            if(changed||(saved_.roi_confirmed&&current_source()&&!matches(saved_,*frame_))){
                runtime_review_required_=true;pending_auto_=false;send_requested_=false;
                notice_=status_.transmitting?L"\u6b63\u5728\u6682\u505c\u4f20\u8f93\uff0c\u8bf7\u590d\u6838\u56fe\u50cf\u533a\u57df":L"\u4f20\u8f93\u5df2\u6682\u505c\uff0c\u8bf7\u590d\u6838\u56fe\u50cf\u533a\u57df";
                if(page_==Page::Roi&&reference_&&!same_shape(*reference_,*frame_)){
                    reference_changed_=true;draft_.roi_confirmed=false;draft_.auto_connect=false;
                    draft_.horizontal_scale.reset();draft_.vertical_scale.reset();load_rulers();
                    dragging_=false;if(GetCapture()==c(FullView))ReleaseCapture();history_.clear();
                    check(Confirm,false);update_dirty();
                }
                if(status_.transmitting&&!pending_)pause();
            }
            if(page_==Page::Main)rebuild_main_crop();
        }else if(!next&&frame_){frame_.reset();main_crop_.clear();InvalidateRect(c(MainImage),nullptr,FALSE);}
        if(frame_&&current_source()&&saved_.roi_confirmed&&!matches(saved_,*frame_)){
            runtime_review_required_=true;pending_auto_=false;send_requested_=false;
            notice_=status_.transmitting?L"\u6b63\u5728\u6682\u505c\u4f20\u8f93\uff0c\u8bf7\u590d\u6838\u56fe\u50cf\u533a\u57df":L"\u4f20\u8f93\u5df2\u6682\u505c\uff0c\u8bf7\u590d\u6838\u56fe\u50cf\u533a\u57df";
        }
        if(enter_roi_when_ready_&&page_==Page::Main&&!dirty_&&frame_&&current_source()&&!pending_&&active_source_==deferred_roi_source_){
            enter_roi_when_ready_=false;activate(Page::Roi);
        }
        if(pending_auto_&&page_==Page::Main&&!pending_&&frame_){
            if(ready_to_transmit())toggle_transmission();else pending_auto_=false;
        }
        update_main();refresh_controls();
    }
    void sync_roi() {
        syncing_=true;text(RoiX,std::to_wstring(draft_.roi.x));text(RoiY,std::to_wstring(draft_.roi.y));
        text(RoiWidth,std::to_wstring(draft_.roi.width));text(RoiHeight,std::to_wstring(draft_.roi.height));
        check(Confirm,draft_.roi_confirmed);syncing_=false;
        text(RoiText,std::to_wstring(draft_.roi.width)+L" x "+std::to_wstring(draft_.roi.height)+L" px");
    }
    void remember(const Rect& value) {
        history_.push_back(value);if(history_.size()>80)history_.erase(history_.begin());
    }
    void change_roi(const Rect& value,bool push_history,bool sync=true) {
        if(!reference_||!valid_rect(value,reference_->width,reference_->height)||same_rect(value,draft_.roi))return;
        if(push_history)remember(draft_.roi);
        draft_.roi=value;draft_.roi_confirmed=false;draft_.auto_connect=false;invalid_roi_=false;
        check(Confirm,false);if(sync)sync_roi();
        text(RoiText,std::to_wstring(value.width)+L" x "+std::to_wstring(value.height)+L" px");
        rebuild_edit_crop();update_dirty();refresh_controls();invalidate_editor();
    }
    void numeric_roi(int id) {
        if(page_!=Page::Roi||syncing_||!reference_)return;
        try{
            Rect r{parse_integer(window_text(c(RoiX))),parse_integer(window_text(c(RoiY))),
                   parse_integer(window_text(c(RoiWidth))),parse_integer(window_text(c(RoiHeight)))};
            if(!valid_rect(r,reference_->width,reference_->height))throw std::invalid_argument("\u9009\u533a\u987b\u5728\u539f\u56fe\u5185\uff0c\u5bbd\u9ad8\u81f3\u5c11\u4e3a 1 px");
            invalid_roi_=false;active_edge_=id==RoiX?RoiTarget::W:id==RoiY?RoiTarget::N:id==RoiWidth?RoiTarget::E:RoiTarget::S;
            inspect_ruler_=false;mode_=EditMode::Move;change_roi(r,true,false);error_.clear();
        }catch(const std::exception& e){invalid_roi_=true;draft_.roi_confirmed=false;draft_.auto_connect=false;check(Confirm,false);error_=wide(e.what());}
        update_dirty();refresh_controls();invalidate_editor();
    }
    void set_mode(EditMode mode) {
        mode_=mode;inspect_ruler_=mode==EditMode::Ruler;
        if(inspect_ruler_)scale_expanded_=true;
        update_inspector();layout();refresh_controls();invalidate_editor();
    }
    void select_edge(RoiTarget edge) {
        active_edge_=edge;anchor_x_=anchor_y_=.5;mode_=EditMode::Move;inspect_ruler_=false;
        update_inspector();layout();refresh_controls();invalidate_editor();
    }
    int scalar(const PixelPoint& point)const{return active_axis_==0?point.y:point.x;}
    int cross_coordinate(const PixelPoint& point)const{return active_axis_==0?point.x:point.y;}
    int axis_limit()const{return reference_?(active_axis_==0?reference_->height:reference_->width):0;}
    std::optional<RulerScale>& active_scale(){return active_axis_==0?draft_.vertical_scale:draft_.horizontal_scale;}
    void load_rulers() {
        rulers_={};
        for(int axis=0;axis<2;++axis){
            const auto& scale=axis==0?draft_.vertical_scale:draft_.horizontal_scale;
            if(!scale)continue;
            auto& r=rulers_[axis];r.coordinates={std::to_wstring(scale->pixel_a),std::to_wstring(scale->pixel_b)};
            r.distance=number(scale->distance_mm,6);
            if(scale->orthogonal_a>=0&&scale->orthogonal_b>=0){
                r.points[0]=axis==0?PixelPoint{scale->orthogonal_a,scale->pixel_a}:PixelPoint{scale->pixel_a,scale->orthogonal_a};
                r.points[1]=axis==0?PixelPoint{scale->orthogonal_b,scale->pixel_b}:PixelPoint{scale->pixel_b,scale->orthogonal_b};
            }
        }
        populate_ruler();
    }
    void populate_ruler() {
        syncing_=true;auto& r=rulers_[active_axis_];
        select(RulerAxis,active_axis_);text(RulerA,r.coordinates[0]);text(RulerB,r.coordinates[1]);
        text(RulerDistance,r.distance);select(RulerUnit,r.unit);check(UnitConfirmed,r.checked);
        select(ActiveTick,active_tick_);syncing_=false;update_scale_summary();update_inspector();
    }
    void update_scale_summary() {
        auto value=[](const std::optional<RulerScale>& scale){return scale?number(scale->mm_per_pixel(),5)+L" mm/px":L"\u672a\u77e5";};
        text(RulerText,L"\u6a2a\u5411\uff1a"+value(draft_.horizontal_scale)+L"    \u7eb5\u5411\uff1a"+value(draft_.vertical_scale));
        const auto& r=rulers_[active_axis_];
        if(r.pending)text(RulerNote,L"\u523b\u5ea6\u4fee\u6539\u5c1a\u672a\u8bb0\u5f55");
        else if(active_scale()&&(!r.points[0]||!r.points[1]))text(RulerNote,L"\u5df2\u5b58\u6bd4\u4f8b\u7f3a\u5c11\u5b8c\u6574\u523b\u5ea6\u4f4d\u7f6e\uff1b\u5c40\u90e8\u68c0\u67e5\u9700\u91cd\u65b0\u9009\u70b9");
        else text(RulerNote,L"\u663e\u793a\u6807\u5c3a\u4f30\u8ba1\uff1b\u673a\u5668\u4eba\u7a7a\u95f4\u5173\u7cfb\u672a\u6807\u5b9a");
    }
    void ruler_input(int id) {
        if(syncing_||page_!=Page::Roi)return;
        auto& r=rulers_[active_axis_];r.pending=true;
        if(id==RulerDistance)r.distance=window_text(c(RulerDistance));
        if(id==RulerUnit)r.unit=selection(RulerUnit);
        if(id==UnitConfirmed)r.checked=checked(UnitConfirmed);
        if(id==RulerA||id==RulerB){
            inspect_ruler_=true;mode_=EditMode::Ruler;
            const int index=id==RulerA?0:1;r.coordinates[index]=window_text(c(id));active_tick_=index;
            try{
                const int coordinate=parse_integer(r.coordinates[index]);
                if(!r.points[index]||coordinate>=axis_limit())throw std::invalid_argument("\u8bf7\u5148\u9009\u62e9\u523b\u5ea6\uff0c\u5e76\u8f93\u5165\u539f\u56fe\u8303\u56f4\u5185\u7684\u5750\u6807");
                if(active_axis_==0)r.points[index]->y=coordinate;else r.points[index]->x=coordinate;
            }catch(const std::exception& e){error_=wide(e.what());}
        }
        r.invalid=!ruler_coordinates_valid(r);
        if(id==RulerDistance||id==RulerUnit){r.checked=false;check(UnitConfirmed,false);}
        update_dirty();update_scale_summary();update_inspector();refresh_controls();invalidate_editor();
    }
    void record_ruler() {
        auto& r=rulers_[active_axis_];
        try{
            if(!reference_||!ruler_coordinates_valid(r))throw std::invalid_argument("\u8bf7\u6838\u5bf9\u4e24\u4e2a\u523b\u5ea6\u53ca\u5176\u6709\u6548\u539f\u56fe\u5750\u6807");
            std::size_t end=0;const double input=std::stod(r.distance,&end);
            const double mm=input*(r.unit==1?10.0:1.0);
            if(end!=r.distance.size()||!std::isfinite(mm)||mm<=0||!r.checked)throw std::invalid_argument("\u8bf7\u6838\u5bf9\u6709\u6548\u8ddd\u79bb\u4e0e\u5355\u4f4d");
            const int a=scalar(*r.points[0]),b=scalar(*r.points[1]);
            if(std::abs(a-b)<2)throw std::invalid_argument("\u4e24\u4e2a\u523b\u5ea6\u6cbf\u6240\u9009\u65b9\u5411\u81f3\u5c11\u76f8\u9694 2 px");
            const RulerScale candidate{a,b,mm,cross_coordinate(*r.points[0]),cross_coordinate(*r.points[1])};
            candidate.mm_per_pixel();
            active_scale()=candidate;
            r.pending=false;r.invalid=false;error_.clear();update_scale_summary();update_dirty();refresh_controls();
        }catch(const std::exception& e){fail(e);}
    }
    bool ruler_coordinates_valid(const RulerDraft& value)const {
        if(!reference_)return false;
        try{
            for(int i=0;i<2;++i){
                if(!value.points[i])return false;
                const int coordinate=parse_integer(value.coordinates[i]);
                if(coordinate>=axis_limit()||coordinate!=scalar(*value.points[i]))return false;
                const auto& point=*value.points[i];
                if(point.x<0||point.y<0||point.x>=reference_->width||point.y>=reference_->height)return false;
            }
            return true;
        }catch(...){return false;}
    }
    void clear_ruler() {
        active_scale().reset();rulers_[active_axis_]={};active_tick_=0;error_.clear();
        populate_ruler();update_dirty();refresh_controls();invalidate_editor();
    }
    std::optional<PixelPoint> inspection_focus()const {
        if(!reference_)return std::nullopt;
        if(inspect_ruler_)return rulers_[active_axis_].points[active_tick_];
        if(!valid_rect(draft_.roi,reference_->width,reference_->height))return std::nullopt;
        return inspection_point(draft_.roi,active_edge_,anchor_x_,anchor_y_);
    }
    void update_inspector() {
        syncing_=true;select(InspectorKind,inspect_ruler_?1:0);select(ActiveEdge,static_cast<int>(active_edge_));
        select(ActiveTick,active_tick_);select(Zoom,zoom_==8?1:0);syncing_=false;
        text(MagnifierTitle,inspect_ruler_?L"\u523b\u5ea6\u5c40\u90e8\u653e\u5927":L"\u8fb9\u754c\u5c40\u90e8\u653e\u5927");
        if(page_==Page::Roi){show(ActiveTick,inspect_ruler_);show(ActiveEdge,!inspect_ruler_);}
        const auto point=inspection_focus();
        text(MagnifierPosition,point?(inspect_ruler_?(active_tick_?L"\u523b\u5ea6 B":L"\u523b\u5ea6 A"):target_name(active_edge_))+
            L"  x "+std::to_wstring(point->x)+L"  y "+std::to_wstring(point->y)+L"  "+std::to_wstring(zoom_)+L"x":L"\u672a\u9009\u523b\u5ea6\u4f4d\u7f6e");
    }
    bool can_nudge(int dx,int dy)const {
        if(!reference_||pending_)return false;
        if(!inspect_ruler_)return bool(nudge_roi(draft_.roi,reference_->width,reference_->height,active_edge_,dx,dy));
        const auto& point=rulers_[active_axis_].points[active_tick_];
        if(!point||(active_axis_==0&&dx)||(active_axis_==1&&dy))return false;
        const int next=(active_axis_==0?point->y+dy:point->x+dx);return next>=0&&next<axis_limit();
    }
    void nudge(int dx,int dy) {
        if(!can_nudge(dx,dy))return;
        if(inspect_ruler_){
            auto& r=rulers_[active_axis_];auto& point=*r.points[active_tick_];point.x+=dx;point.y+=dy;
            r.coordinates[active_tick_]=std::to_wstring(scalar(point));r.pending=true;r.invalid=!ruler_coordinates_valid(r);
            populate_ruler();update_dirty();refresh_controls();invalidate_editor();
        }else if(auto value=nudge_roi(draft_.roi,reference_->width,reference_->height,active_edge_,dx,dy))change_roi(*value,true);
    }
    RECT image_area(HWND view,int width,int height)const {
        RECT bounds{};GetClientRect(view,&bounds);if(width<=0||height<=0)return bounds;
        const double scale=std::min(static_cast<double>(bounds.right)/width,static_cast<double>(bounds.bottom)/height);
        const int w=static_cast<int>(std::round(width*scale)),h=static_cast<int>(std::round(height*scale));
        const int x=(bounds.right-w)/2,y=(bounds.bottom-h)/2;return {x,y,x+w,y+h};
    }
    PixelPoint source_point(HWND view,LPARAM lp)const {
        const RECT rect=image_area(view,reference_->width,reference_->height);
        return {std::clamp(MulDiv(GET_X_LPARAM(lp)-rect.left,reference_->width,std::max(1L,rect.right-rect.left)),0,reference_->width),
                std::clamp(MulDiv(GET_Y_LPARAM(lp)-rect.top,reference_->height,std::max(1L,rect.bottom-rect.top)),0,reference_->height)};
    }
    PixelPoint source_pixel(HWND view,LPARAM lp)const {
        const RECT rect=image_area(view,reference_->width,reference_->height);
        const auto x=static_cast<std::int64_t>(GET_X_LPARAM(lp)-rect.left)*reference_->width/std::max(1L,rect.right-rect.left);
        const auto y=static_cast<std::int64_t>(GET_Y_LPARAM(lp)-rect.top)*reference_->height/std::max(1L,rect.bottom-rect.top);
        return {static_cast<int>(std::clamp<std::int64_t>(x,0,reference_->width-1)),static_cast<int>(std::clamp<std::int64_t>(y,0,reference_->height-1))};
    }
    POINT view_point(HWND view,PixelPoint point)const {
        const RECT rect=image_area(view,reference_->width,reference_->height);
        return {rect.left+MulDiv(point.x,rect.right-rect.left,reference_->width),rect.top+MulDiv(point.y,rect.bottom-rect.top,reference_->height)};
    }
    POINT view_pixel_center(HWND view,PixelPoint point)const {
        const RECT rect=image_area(view,reference_->width,reference_->height);
        return {rect.left+static_cast<LONG>(std::llround((point.x+.5)*(rect.right-rect.left)/reference_->width)),
                rect.top+static_cast<LONG>(std::llround((point.y+.5)*(rect.bottom-rect.top)/reference_->height))};
    }
    std::optional<RoiTarget> hit_boundary(HWND view,POINT point)const {
        if(!reference_||!valid_rect(draft_.roi,reference_->width,reference_->height))return std::nullopt;
        const int hit=px(8);
        for(RoiTarget target:{RoiTarget::NW,RoiTarget::NE,RoiTarget::SE,RoiTarget::SW}){
            const auto p=view_point(view,inspection_point(draft_.roi,target));
            if(std::abs(point.x-p.x)<=hit&&std::abs(point.y-p.y)<=hit)return target;
        }
        const auto a=view_point(view,{draft_.roi.x,draft_.roi.y});
        const auto b=view_point(view,{draft_.roi.x+draft_.roi.width,draft_.roi.y+draft_.roi.height});
        if(point.x>=a.x-hit&&point.x<=b.x+hit){
            if(std::abs(point.y-a.y)<=hit)return RoiTarget::N;
            if(std::abs(point.y-b.y)<=hit)return RoiTarget::S;
        }
        if(point.y>=a.y-hit&&point.y<=b.y+hit){
            if(std::abs(point.x-a.x)<=hit)return RoiTarget::W;
            if(std::abs(point.x-b.x)<=hit)return RoiTarget::E;
        }
        return std::nullopt;
    }
    void pointer_down(HWND view,LPARAM lp) {
        if(page_!=Page::Roi||!reference_||pending_||reference_changed_)return;
        const POINT screen{GET_X_LPARAM(lp),GET_Y_LPARAM(lp)};const auto area=image_area(view,reference_->width,reference_->height);
        const auto edge_hit=mode_==EditMode::Move?hit_boundary(view,screen):std::optional<RoiTarget>{};
        if(!PtInRect(&area,screen)&&!edge_hit)return;
        SetFocus(view);
        auto point=source_point(view,lp);
        if(mode_==EditMode::Ruler){
            point=source_pixel(view,lp);
            auto& r=rulers_[active_axis_];
            if(!r.points[0]||r.points[1]){r.points={};r.coordinates={};active_tick_=0;}else active_tick_=1;
            r.points[active_tick_]=point;r.coordinates[active_tick_]=std::to_wstring(scalar(point));r.pending=true;r.invalid=false;
            populate_ruler();update_dirty();refresh_controls();invalidate_editor();return;
        }
        drag_old_=draft_.roi;drag_start_=point;drag_draw_=mode_==EditMode::Draw;drag_changed_=false;
        drag_old_confirmed_=draft_.roi_confirmed;drag_old_auto_=draft_.auto_connect;
        if(!drag_draw_){
            if(edge_hit){
                active_edge_=drag_target_=*edge_hit;
                anchor_x_=std::clamp(static_cast<double>(point.x-draft_.roi.x)/draft_.roi.width,0.0,1.0);
                anchor_y_=std::clamp(static_cast<double>(point.y-draft_.roi.y)/draft_.roi.height,0.0,1.0);
            }else if(point.x>=draft_.roi.x&&point.y>=draft_.roi.y&&point.x<draft_.roi.x+draft_.roi.width&&point.y<draft_.roi.y+draft_.roi.height){
                active_edge_=drag_target_=RoiTarget::Move;
                const auto pixel=source_pixel(view,lp);
                anchor_x_=std::clamp(static_cast<double>(pixel.x-draft_.roi.x)/std::max(1,draft_.roi.width-1),0.0,1.0);
                anchor_y_=std::clamp(static_cast<double>(pixel.y-draft_.roi.y)/std::max(1,draft_.roi.height-1),0.0,1.0);
            }else return;
        }
        dragging_=true;SetCapture(view);update_inspector();refresh_controls();invalidate_editor();
    }
    void pointer_move(HWND view,LPARAM lp) {
        if(!dragging_||!reference_||reference_changed_)return;
        const auto point=source_point(view,lp);std::optional<Rect> value;
        if(drag_draw_){
            Rect candidate{std::min(point.x,drag_start_.x),std::min(point.y,drag_start_.y),std::abs(point.x-drag_start_.x),std::abs(point.y-drag_start_.y)};
            if(valid_rect(candidate,reference_->width,reference_->height))value=candidate;
        }else value=drag_roi(drag_old_,reference_->width,reference_->height,drag_target_,point.x-drag_start_.x,point.y-drag_start_.y);
        if(value&&!same_rect(*value,draft_.roi)){
            if(!drag_changed_){remember(drag_old_);drag_changed_=true;}
            if(drag_draw_)active_edge_=RoiTarget::SE;
            change_roi(*value,false);
        }
    }
    void pointer_up() {
        if(!dragging_)return;
        dragging_=false;
        if(GetCapture()==c(FullView))ReleaseCapture();
        if(drag_changed_&&same_rect(drag_old_,draft_.roi)){
            if(!history_.empty())history_.pop_back();
            draft_.roi_confirmed=drag_old_confirmed_;draft_.auto_connect=drag_old_auto_;sync_roi();
        }
        if(drag_changed_)mode_=EditMode::Move;
        update_dirty();refresh_controls();invalidate_editor();
    }
    void cancel_drag() {
        if(!dragging_)return;
        draft_.roi=drag_old_;draft_.roi_confirmed=drag_old_confirmed_;draft_.auto_connect=drag_old_auto_;
        if(drag_changed_&&!history_.empty())history_.pop_back();
        dragging_=false;drag_changed_=false;
        if(GetCapture()==c(FullView))ReleaseCapture();
        sync_roi();rebuild_edit_crop();update_dirty();refresh_controls();invalidate_editor();
    }
    void invalidate_editor() {
        update_inspector();InvalidateRect(c(FullView),nullptr,FALSE);InvalidateRect(c(MagnifierView),nullptr,FALSE);
    }
    void update_main() {
        text(MainSource,wide(!saved_.title_contains.empty()?saved_.title_contains:!saved_.process_name.empty()?saved_.process_name:!saved_.executable.empty()?source_profile(saved_).process_name:"\u672a\u914d\u7f6e\u8d85\u58f0\u8f6f\u4ef6"));
        const bool stale=frame_&&status_.frame_age_s>1.0;
        text(StatusText,pending_?L"\u6b63\u5728\u5904\u7406\u2026":stale?L"\u753b\u9762\u66f4\u65b0\u8d85\u65f6":state_name(status_));
        text(MainCaption,stale?L"\u8f93\u51fa\u9884\u89c8 \u00b7 \u6700\u540e\u4e00\u5e27":L"\u8f93\u51fa\u9884\u89c8");
        const bool confirmed=saved_.roi_confirmed&&!runtime_review_required_;
        const std::wstring scale=runtime_review_required_&&(saved_.horizontal_scale||saved_.vertical_scale)?L"\u5f85\u590d\u6838":saved_.horizontal_scale&&saved_.vertical_scale?L"\u53cc\u8f74\u5df2\u8bb0\u5f55":saved_.horizontal_scale?L"\u6a2a\u5411\u5df2\u8bb0\u5f55\uff0c\u7eb5\u5411\u672a\u77e5":saved_.vertical_scale?L"\u7eb5\u5411\u5df2\u8bb0\u5f55\uff0c\u6a2a\u5411\u672a\u77e5":L"\u6a2a\u7eb5\u672a\u77e5";
        text(MainMeta,L"\u8f93\u51fa\u5c3a\u5bf8  "+std::to_wstring(saved_.roi.width)+L" x "+std::to_wstring(saved_.roi.height)+
            L" px       \u56fe\u50cf\u533a\u57df  "+(confirmed?L"\u5df2\u786e\u8ba4":L"\u5f85\u786e\u8ba4")+L"       \u5c3a\u5bf8\u6bd4\u4f8b  "+scale);
        text(MainNotice,profile_read_error_?L"\u914d\u7f6e\u8bfb\u53d6\u5931\u8d25\uff0c\u539f\u6587\u4ef6\u672a\u6539\u52a8":runtime_review_required_?L"\u7a97\u53e3\u5c3a\u5bf8\u6216\u914d\u7f6e\u5df2\u53d8\u5316\uff0c\u8bf7\u91cd\u65b0\u590d\u6838\u56fe\u50cf\u533a\u57df":L"\u56fe\u50cf\u533a\u57df\u5f85\u786e\u8ba4");
        text(PageTitle,page_==Page::Settings?L"\u8bbe\u5907\u8bbe\u7f6e":L"\u56fe\u50cf\u533a\u57df");
        text(FooterHint,page_==Page::Main?wide(saved_.host)+L":"+std::to_wstring(saved_.port):page_==Page::Roi?L"\u4fdd\u5b58\u540e\u4fdd\u6301\u6682\u505c":default_profile_path().filename().wstring());
        text(Receipt,!error_.empty()?error_:notice_);
        text(RoiError,reference_changed_?L"\u7a97\u53e3\u5c3a\u5bf8\u6216 DPI \u5df2\u53d8\u5316\uff0c\u8bf7\u66f4\u65b0\u53c2\u8003\u5e27\u540e\u91cd\u65b0\u786e\u8ba4":page_==Page::Roi?error_:L"");
        if(reference_)text(SourceText,L"\u56fa\u5b9a\u53c2\u8003\u5e27  "+std::to_wstring(reference_->width)+L" x "+std::to_wstring(reference_->height)+L" px / "+std::to_wstring(reference_->dpi)+L" DPI");
        button_label(Transmit,status_.transmitting||send_requested_?L"\u6682\u505c\u4f20\u8f93":L"\u5f00\u59cb\u4f20\u8f93",status_.transmitting||send_requested_?UiIcon::Pause:UiIcon::Play);
        button_label(SaveDraft,page_==Page::Roi?L"\u4fdd\u5b58\u533a\u57df":L"\u4fdd\u5b58\u8bbe\u7f6e",UiIcon::Save);
        const bool retry=!frame_||!current_source()||status_.state=="ERROR"||status_.state=="CAPTURE_ERROR";
        button_label(StartPreview,!configured(saved_)||profile_read_error_?L"\u8bbe\u7f6e\u8bbe\u5907":L"\u6253\u5f00\u8d85\u58f0\u8f6f\u4ef6",UiIcon::Refresh);
        const bool notice_visible=profile_read_error_||runtime_review_required_||!saved_.roi_confirmed||retry;
        if(page_==Page::Main&&bool(IsWindowVisible(c(MainNotice)))!=notice_visible)layout();
        if(page_==Page::Main&&bool(IsWindowVisible(c(StartPreview)))!=retry)layout();
    }
    void selected_button(int id,bool selected) {
        auto& button=buttons_.at(id);if(button.selected==selected)return;button.selected=selected;InvalidateRect(c(id),nullptr,TRUE);
    }
    void refresh_controls() {
        if(!body_)return;
        const bool edit=pending_==0&&!exiting_;
        for(const auto& item:controls_)if(IsWindowVisible(item.second))EnableWindow(item.second,edit);
        EnableWindow(c(OpenSettings),edit);EnableWindow(c(OpenRoi),edit&&!profile_read_error_&&configured(saved_));
        EnableWindow(c(Transmit),edit&&(status_.transmitting||send_requested_||ready_to_transmit()));
        EnableWindow(c(SaveDraft),edit&&can_save());
        if(page_==Page::Settings){
            bool valid_auto=draft_.roi_confirmed&&!runtime_review_required_;
            try{valid_auto=valid_auto&&prepared_settings().roi_confirmed;}catch(...){valid_auto=false;}
            EnableWindow(c(AutoConnect),edit&&valid_auto);
            selected_button(TabSource,setting_tab_==SettingTab::Source);selected_button(TabReceiver,setting_tab_==SettingTab::Receiver);
            selected_button(TabStartup,setting_tab_==SettingTab::Startup);selected_button(Advanced,advanced_);
            if(profile_read_error_)text(SettingsError,error_);
        }
        if(page_==Page::Roi){
            const bool usable=edit&&reference_&&!reference_changed_;
            for(int id:{Draw,Move,Undo,RestoreRoi,FullWindow,RoiX,RoiY,RoiWidth,RoiHeight,Confirm,PickRuler,RulerA,RulerB,RulerDistance,RulerUnit,UnitConfirmed,RecordRuler,ClearRuler})EnableWindow(c(id),usable);
            for(int i=0;i<4;++i)EnableWindow(c(500+i),usable);
            EnableWindow(c(Undo),usable&&!history_.empty());
            EnableWindow(c(RestoreRoi),usable&&valid_rect(saved_.roi,reference_->width,reference_->height));
            EnableWindow(c(Confirm),usable&&!invalid_roi_&&usable_reference());
            const auto& r=rulers_[active_axis_];
            EnableWindow(c(RecordRuler),usable&&r.checked&&!r.invalid&&ruler_coordinates_valid(r));
            EnableWindow(c(NudgeLeft),usable&&can_nudge(-1,0));EnableWindow(c(NudgeUp),usable&&can_nudge(0,-1));
            EnableWindow(c(NudgeDown),usable&&can_nudge(0,1));EnableWindow(c(NudgeRight),usable&&can_nudge(1,0));
            EnableWindow(c(RefreshReference),edit&&frame_&&current_source());
            selected_button(Draw,mode_==EditMode::Draw);selected_button(Move,mode_==EditMode::Move);
            selected_button(PickRuler,mode_==EditMode::Ruler);selected_button(ScaleExpanded,scale_expanded_);
            check(Confirm,draft_.roi_confirmed);
        }
        text(DirtyState,dirty_?L"\u672a\u4fdd\u5b58":L"\u5df2\u4fdd\u5b58");
    }
    void crop(const std::shared_ptr<const PreviewFrame>& frame,const Rect& roi,std::vector<std::uint8_t>& output) {
        output.clear();if(!frame||!valid_rect(roi,frame->width,frame->height))return;
        const auto mono=crop_mono8(*frame,roi);output.resize(mono.size()*4);
        for(std::size_t i=0;i<mono.size();++i){output[i*4]=output[i*4+1]=output[i*4+2]=mono[i];output[i*4+3]=255;}
    }
    void rebuild_main_crop() {crop(frame_,saved_.roi,main_crop_);InvalidateRect(c(MainImage),nullptr,FALSE);}
    void rebuild_edit_crop() {crop(reference_,draft_.roi,edit_crop_);InvalidateRect(c(CropView),nullptr,FALSE);}
    void clear_icons() {for(const auto& item:icons_)DeleteObject(item.second);icons_.clear();}
    HBITMAP icon(UiIcon kind,int size,COLORREF ink) {
        const auto key=std::make_tuple(static_cast<int>(kind),size,ink);auto it=icons_.find(key);
        if(it==icons_.end())it=icons_.emplace(key,create_ui_icon(kind,size,ink)).first;
        return it->second;
    }
    void draw_button(const DRAWITEMSTRUCT& draw) {
        const auto found=buttons_.find(static_cast<int>(draw.CtlID));if(found==buttons_.end())return;
        const auto& button=found->second;const bool disabled=(draw.itemState&ODS_DISABLED)!=0;
        const bool pressed=(draw.itemState&ODS_SELECTED)!=0,hover=hover_id_==static_cast<int>(draw.CtlID);
        COLORREF background=kPaper,ink=disabled?RGB(160,170,174):kInk,border=kLine;
        if(!disabled&&button.primary){background=hover||pressed?RGB(4,91,88):kAccent;ink=kPaper;border=background;}
        else if(!disabled&&button.selected){background=kSelected;ink=kAccent;border=kAccent;}
        else if(!disabled&&(hover||pressed))background=kBand;
        if(disabled)background=kBand;
        HBRUSH brush=CreateSolidBrush(background);HPEN pen=CreatePen(PS_SOLID,1,border);
        auto old_brush=SelectObject(draw.hDC,brush),old_pen=SelectObject(draw.hDC,pen);
        if(button.tab){
            FillRect(draw.hDC,&draw.rcItem,paper_brush_);
            if(button.selected){RECT line=draw.rcItem;line.top=line.bottom-px(2);HBRUSH accent=CreateSolidBrush(kAccent);FillRect(draw.hDC,&line,accent);DeleteObject(accent);ink=kAccent;}
        }else RoundRect(draw.hDC,draw.rcItem.left,draw.rcItem.top,draw.rcItem.right,draw.rcItem.bottom,px(5),px(5));
        SelectObject(draw.hDC,old_brush);SelectObject(draw.hDC,old_pen);DeleteObject(brush);DeleteObject(pen);
        SelectObject(draw.hDC,font_);SetTextColor(draw.hDC,ink);SetBkMode(draw.hDC,TRANSPARENT);
        SIZE extent{};if(button.text)GetTextExtentPoint32W(draw.hDC,button.label.c_str(),static_cast<int>(button.label.size()),&extent);
        const int size=px(17),gap=button.icon&&button.text?px(7):0;
        const int available=draw.rcItem.right-draw.rcItem.left-px(16);
        const int content=std::min(available,(button.icon?size:0)+gap+static_cast<int>(extent.cx));
        int x=draw.rcItem.left+(draw.rcItem.right-draw.rcItem.left-content)/2;
        if(button.icon){paint_ui_icon(draw.hDC,icon(*button.icon,size,ink),x,draw.rcItem.top+(draw.rcItem.bottom-draw.rcItem.top-size)/2,size);x+=size+gap;}
        if(button.text){RECT text_rect{ x,draw.rcItem.top,draw.rcItem.right-px(7),draw.rcItem.bottom};DrawTextW(draw.hDC,button.label.c_str(),-1,&text_rect,DT_LEFT|DT_VCENTER|DT_SINGLELINE|DT_END_ELLIPSIS);}
        if(draw.itemState&ODS_FOCUS){RECT focus=draw.rcItem;InflateRect(&focus,-px(3),-px(3));DrawFocusRect(draw.hDC,&focus);}
    }
    void draw_bitmap(HDC dc,const RECT& target,int width,int height,const std::uint8_t* pixels) {
        if(width<=0||height<=0||!pixels)return;
        BITMAPINFO info{};info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);info.bmiHeader.biWidth=width;
        info.bmiHeader.biHeight=-height;info.bmiHeader.biPlanes=1;info.bmiHeader.biBitCount=32;info.bmiHeader.biCompression=BI_RGB;
        SetStretchBltMode(dc,COLORONCOLOR);
        StretchDIBits(dc,target.left,target.top,target.right-target.left,target.bottom-target.top,0,0,width,height,pixels,&info,DIB_RGB_COLORS,SRCCOPY);
    }
    void shade(HDC dc,RECT area,const RECT& clip,BYTE alpha=78) {
        RECT bounded{};if(!IntersectRect(&bounded,&area,&clip))return;
        HDC solid=CreateCompatibleDC(dc);HBITMAP pixel=CreateCompatibleBitmap(dc,1,1);auto old=SelectObject(solid,pixel);
        SetPixel(solid,0,0,RGB(0,0,0));BLENDFUNCTION blend{AC_SRC_OVER,0,alpha,0};
        AlphaBlend(dc,bounded.left,bounded.top,bounded.right-bounded.left,bounded.bottom-bounded.top,solid,0,0,1,1,blend);
        SelectObject(solid,old);DeleteObject(pixel);DeleteDC(solid);
    }
    void mask_outside(HDC dc,RECT roi,const RECT& clip) {
        shade(dc,{clip.left,clip.top,clip.right,roi.top},clip);
        shade(dc,{clip.left,roi.bottom,clip.right,clip.bottom},clip);
        shade(dc,{clip.left,roi.top,roi.left,roi.bottom},clip);
        shade(dc,{roi.right,roi.top,clip.right,roi.bottom},clip);
    }
    void line(HDC dc,POINT a,POINT b,COLORREF color,int thickness=1) {
        HPEN pen=CreatePen(PS_SOLID,thickness,color);auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);
    }
    void ring(HDC dc,POINT point,COLORREF color,int radius) {
        HPEN pen=CreatePen(PS_SOLID,1,color);auto old_pen=SelectObject(dc,pen);auto old_brush=SelectObject(dc,GetStockObject(HOLLOW_BRUSH));
        Ellipse(dc,point.x-radius,point.y-radius,point.x+radius+1,point.y+radius+1);
        SelectObject(dc,old_brush);SelectObject(dc,old_pen);DeleteObject(pen);
    }
    void draw_reference(HDC dc,HWND view,const RECT& bounds) {
        if(!reference_)return;
        const RECT area=image_area(view,reference_->width,reference_->height);
        draw_bitmap(dc,area,reference_->width,reference_->height,reference_->bgra.data());
        if(!valid_rect(draft_.roi,reference_->width,reference_->height))return;
        const auto a=view_point(view,{draft_.roi.x,draft_.roi.y}),b=view_point(view,{draft_.roi.x+draft_.roi.width,draft_.roi.y+draft_.roi.height});
        if(!inspect_ruler_)mask_outside(dc,{a.x,a.y,b.x,b.y},area);
        const COLORREF border=inspect_ruler_?RGB(150,170,165):kSelection;
        line(dc,a,{b.x,a.y},border,px(2));line(dc,{b.x,a.y},b,border,px(2));line(dc,b,{a.x,b.y},border,px(2));line(dc,{a.x,b.y},a,border,px(2));
        if(!inspect_ruler_){
            HBRUSH handle=CreateSolidBrush(kSelection);
            for(RoiTarget target:{RoiTarget::NW,RoiTarget::N,RoiTarget::NE,RoiTarget::E,RoiTarget::SE,RoiTarget::S,RoiTarget::SW,RoiTarget::W}){
                const auto p=view_point(view,inspection_point(draft_.roi,target));RECT box{p.x-px(3),p.y-px(3),p.x+px(4),p.y+px(4)};
                FillRect(dc,&box,handle);
            }
            DeleteObject(handle);
            if(const auto focus=inspection_focus())ring(dc,active_edge_==RoiTarget::Move?view_pixel_center(view,*focus):view_point(view,*focus),RGB(255,209,134),px(5));
        }else{
            for(int i=0;i<2;++i)if(const auto p=rulers_[active_axis_].points[i]){
                const auto v=view_pixel_center(view,*p);const auto color=i?RGB(255,202,117):RGB(72,221,255);
                line(dc,{v.x-px(7),v.y},{v.x+px(7),v.y},color,px(2));line(dc,{v.x,v.y-px(7)},{v.x,v.y+px(7)},color,px(2));
                SelectObject(dc,font_);SetTextColor(dc,color);SetBkMode(dc,TRANSPARENT);
                RECT label_rect{v.x+px(9),std::max(0L,v.y-px(18)),std::min(bounds.right,v.x+px(30)),v.y+px(4)};
                DrawTextW(dc,i?L"B":L"A",1,&label_rect,DT_LEFT|DT_SINGLELINE);
                if(i==active_tick_)ring(dc,v,kPaper,px(10));
            }
        }
    }
    void draw_magnifier(HDC dc,const RECT& bounds) {
        const auto focus=inspection_focus();if(!reference_||!focus)return;
        const int cx=bounds.right/2,cy=bounds.bottom/2;
        const int half_pixel=inspect_ruler_||active_edge_==RoiTarget::Move?zoom_/2:0;
        const int x=cx-focus->x*zoom_-half_pixel,y=cy-focus->y*zoom_-half_pixel;
        // Keep out-of-image padding in place: the selected boundary stays at the center.
        draw_bitmap(dc,{x,y,x+reference_->width*zoom_,y+reference_->height*zoom_},reference_->width,reference_->height,reference_->bgra.data());
        if(!inspect_ruler_){
            const RECT roi{x+draft_.roi.x*zoom_,y+draft_.roi.y*zoom_,x+(draft_.roi.x+draft_.roi.width)*zoom_,y+(draft_.roi.y+draft_.roi.height)*zoom_};
            mask_outside(dc,roi,bounds);
            line(dc,{roi.left,roi.top},{roi.right,roi.top},kSelection);line(dc,{roi.right,roi.top},{roi.right,roi.bottom},kSelection);
            line(dc,{roi.right,roi.bottom},{roi.left,roi.bottom},kSelection);line(dc,{roi.left,roi.bottom},{roi.left,roi.top},kSelection);
            ring(dc,{cx,cy},RGB(255,209,134),px(4));
        }else{
            const COLORREF color=active_tick_?RGB(255,202,117):RGB(72,221,255);
            line(dc,{cx-px(10),cy},{cx+px(10),cy},color);line(dc,{cx,cy-px(10)},{cx,cy+px(10)},color);ring(dc,{cx,cy},color,px(4));
        }
    }
    void paint_image(HWND view) {
        PAINTSTRUCT paint{};HDC screen=BeginPaint(view,&paint);RECT bounds{};GetClientRect(view,&bounds);
        HDC dc=CreateCompatibleDC(screen);HBITMAP bitmap=CreateCompatibleBitmap(screen,std::max(1L,bounds.right),std::max(1L,bounds.bottom));auto old=SelectObject(dc,bitmap);
        HBRUSH dark=CreateSolidBrush(RGB(16,19,20));FillRect(dc,&bounds,dark);DeleteObject(dark);
        const int id=GetDlgCtrlID(view);bool content=false;
        if(id==FullView&&reference_){draw_reference(dc,view,bounds);content=true;}
        else if(id==MagnifierView&&reference_&&inspection_focus()){draw_magnifier(dc,bounds);content=true;}
        else if(id==MainImage&&!main_crop_.empty()){draw_bitmap(dc,image_area(view,saved_.roi.width,saved_.roi.height),saved_.roi.width,saved_.roi.height,main_crop_.data());content=true;}
        else if(id==CropView&&!edit_crop_.empty()){draw_bitmap(dc,image_area(view,draft_.roi.width,draft_.roi.height),draft_.roi.width,draft_.roi.height,edit_crop_.data());content=true;}
        if(!content){
            SelectObject(dc,font_);SetTextColor(dc,RGB(185,198,202));SetBkMode(dc,TRANSPARENT);
            const wchar_t* message=id==MagnifierView?L"\u672a\u9009\u523b\u5ea6\u4f4d\u7f6e":id==MainImage?L"\u7b49\u5f85\u56fe\u50cf\u533a\u57df":L"\u7b49\u5f85\u7a97\u53e3\u753b\u9762";
            DrawTextW(dc,message,-1,&bounds,DT_CENTER|DT_VCENTER|DT_SINGLELINE);
        }
        BitBlt(screen,0,0,bounds.right,bounds.bottom,dc,0,0,SRCCOPY);SelectObject(dc,old);DeleteObject(bitmap);DeleteDC(dc);EndPaint(view,&paint);
    }
    void paint_main() {
        PAINTSTRUCT paint{};HDC dc=BeginPaint(hwnd_,&paint);RECT area{};GetClientRect(hwnd_,&area);FillRect(dc,&area,paper_brush_);
        RECT footer=area;footer.top=std::max(0L,footer.bottom-px(76));FillRect(dc,&footer,band_brush_);
        line(dc,{0,px(58)},{area.right,px(58)},kLine);line(dc,{0,footer.top},{area.right,footer.top},kLine);EndPaint(hwnd_,&paint);
    }
    void browse_executable() {
        std::array<wchar_t,32768> path{};const auto value=window_text(c(Executable));std::copy_n(value.begin(),std::min(value.size(),path.size()-1),path.begin());
        OPENFILENAMEW dialog{};dialog.lStructSize=sizeof(dialog);dialog.hwndOwner=hwnd_;
        dialog.lpstrFilter=L"Windows program (*.exe)\0*.exe\0All files\0*.*\0";dialog.lpstrFile=path.data();dialog.nMaxFile=static_cast<DWORD>(path.size());
        dialog.Flags=OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR;if(GetOpenFileNameW(&dialog))text(Executable,path.data());
    }
    void browse_directory() {
        BROWSEINFOW dialog{};dialog.hwndOwner=hwnd_;dialog.lpszTitle=L"\u9009\u62e9\u8d85\u58f0\u8f6f\u4ef6\u5de5\u4f5c\u76ee\u5f55";dialog.ulFlags=BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE;
        auto folder=SHBrowseForFolderW(&dialog);if(!folder)return;
        wchar_t path[MAX_PATH]{};if(SHGetPathFromIDListW(folder,path))text(WorkingDir,path);CoTaskMemFree(folder);
    }
    void command(int id,int notification) {
        if(syncing_)return;
        if(notification==EN_CHANGE){
            if(page_==Page::Settings&&(id==Executable||id==WorkingDir||id==ProcessName||id==Title||id==Host||id==Port||id==Context))settings_changed(id);
            else if(page_==Page::Roi&&id>=RoiX&&id<=RoiHeight)numeric_roi(id);
            else if(page_==Page::Roi&&(id==RulerA||id==RulerB||id==RulerDistance))ruler_input(id);
            return;
        }
        if(notification==CBN_SELCHANGE){
            if(id==ActiveEdge)select_edge(static_cast<RoiTarget>(selection(ActiveEdge)));
            else if(id==ActiveTick){active_tick_=selection(ActiveTick);update_inspector();refresh_controls();invalidate_editor();}
            else if(id==InspectorKind)set_mode(selection(InspectorKind)?EditMode::Ruler:EditMode::Move);
            else if(id==Zoom){zoom_=selection(Zoom)?8:4;update_inspector();InvalidateRect(c(MagnifierView),nullptr,FALSE);}
            else if(id==RulerAxis){active_axis_=selection(RulerAxis);active_tick_=0;populate_ruler();refresh_controls();invalidate_editor();}
            else if(id==RulerUnit)ruler_input(id);
            return;
        }
        if(notification!=BN_CLICKED)return;
        if(id==TrayShow){ShowWindow(hwnd_,SW_RESTORE);SetForegroundWindow(hwnd_);return;}
        if(id==TrayPause){if(!pending_)pause();return;}
        if(id==TrayExit){request_navigation(Navigation::Exit);return;}
        if(pending_)return;
        switch(id){
        case IDCANCEL:
            if(dragging_)cancel_drag();else if(page_!=Page::Main)request_navigation(Navigation::Main);
            break;
        case OpenSettings:request_navigation(Navigation::Settings);break;
        case StartPreview:
            if(configured(saved_)&&!profile_read_error_)start_preview(saved_);else request_navigation(Navigation::Settings);
            break;
        case OpenRoi:request_navigation(Navigation::Roi);break;
        case Back:case Cancel:request_navigation(Navigation::Main);break;
        case Transmit:toggle_transmission();break;
        case SaveDraft:save_draft();break;
        case TabSource:setting_tab_=SettingTab::Source;scroll_=0;layout();break;
        case TabReceiver:setting_tab_=SettingTab::Receiver;scroll_=0;layout();break;
        case TabStartup:setting_tab_=SettingTab::Startup;scroll_=0;layout();break;
        case Advanced:advanced_=!advanced_;layout();break;
        case BrowseExe:browse_executable();break;
        case BrowseDir:browse_directory();break;
        case LaunchVendor:case AutoConnect:case MinimizeToTray:settings_changed(id);break;
        case ReloadConfig:
            if(!dirty_||unsaved_choice()==IDNO){activate(Page::Main);load_saved();}
            break;
        case Draw:set_mode(EditMode::Draw);break;
        case Move:set_mode(EditMode::Move);break;
        case Undo:
            if(!history_.empty()){const auto old=history_.back();history_.pop_back();change_roi(old,false);}
            break;
        case RestoreRoi:change_roi(saved_.roi,true);break;
        case FullWindow:if(reference_)change_roi({0,0,reference_->width,reference_->height},true);break;
        case RefreshReference:refresh_reference();break;
        case Confirm:draft_.roi_confirmed=checked(Confirm)&&usable_reference()&&!invalid_roi_;review_touched_=true;update_dirty();break;
        case ScaleExpanded:scale_expanded_=!scale_expanded_;layout();break;
        case PickRuler:set_mode(EditMode::Ruler);break;
        case UnitConfirmed:ruler_input(id);break;
        case RecordRuler:record_ruler();break;
        case ClearRuler:clear_ruler();break;
        case NudgeLeft:nudge(-1,0);break;
        case NudgeUp:nudge(0,-1);break;
        case NudgeDown:nudge(0,1);break;
        case NudgeRight:nudge(1,0);break;
        }
        update_main();refresh_controls();
    }
    void scroll_body(int command,int wheel=0) {
        if(page_==Page::Main)return;
        SCROLLINFO info{};info.cbSize=sizeof(info);info.fMask=SIF_ALL;GetScrollInfo(body_,SB_VERT,&info);
        if(wheel)scroll_-=wheel/WHEEL_DELTA*60;
        else switch(command){
        case SB_LINEUP:scroll_-=30;break;case SB_LINEDOWN:scroll_+=30;break;
        case SB_PAGEUP:scroll_-=static_cast<int>(info.nPage);break;case SB_PAGEDOWN:scroll_+=static_cast<int>(info.nPage);break;
        case SB_TOP:scroll_=0;break;case SB_BOTTOM:scroll_=info.nMax;break;case SB_THUMBTRACK:scroll_=info.nTrackPos;break;default:break;
        }
        scroll_=std::clamp(scroll_,0,std::max(0,info.nMax-static_cast<int>(info.nPage)+1));layout();refresh_controls();
    }
    void add_tray() {
        NOTIFYICONDATAW icon{};icon.cbSize=sizeof(icon);icon.hWnd=hwnd_;icon.uID=1;icon.uFlags=NIF_MESSAGE|NIF_ICON|NIF_TIP;
        icon.uCallbackMessage=kTray;icon.hIcon=LoadIconW(nullptr,IDI_APPLICATION);wcscpy_s(icon.szTip,L"\u8d85\u58f0\u56fe\u50cf\u63a5\u5165");tray_added_=Shell_NotifyIconW(NIM_ADD,&icon)!=FALSE;
    }
    void remove_tray() {
        if(!tray_added_)return;
        NOTIFYICONDATAW icon{};icon.cbSize=sizeof(icon);icon.hWnd=hwnd_;icon.uID=1;Shell_NotifyIconW(NIM_DELETE,&icon);tray_added_=false;
    }
    void tray_menu() {
        HMENU menu=CreatePopupMenu();AppendMenuW(menu,MF_STRING,TrayShow,L"\u663e\u793a\u4e3b\u7a97\u53e3");AppendMenuW(menu,MF_STRING,TrayPause,L"\u6682\u505c\u4f20\u8f93");
        AppendMenuW(menu,MF_SEPARATOR,0,nullptr);AppendMenuW(menu,MF_STRING,TrayExit,L"\u9000\u51fa\u91c7\u96c6\u5668");
        POINT point{};GetCursorPos(&point);SetForegroundWindow(hwnd_);
        const UINT chosen=TrackPopupMenu(menu,TPM_RETURNCMD|TPM_RIGHTBUTTON,point.x,point.y,0,hwnd_,nullptr);DestroyMenu(menu);PostMessageW(hwnd_,WM_NULL,0,0);
        if(chosen)SendMessageW(hwnd_,WM_COMMAND,chosen,0);
    }
    LRESULT colors(UINT message,WPARAM wp,LPARAM lp) {
        HDC dc=reinterpret_cast<HDC>(wp);HWND child=reinterpret_cast<HWND>(lp);const int id=GetDlgCtrlID(child);
        COLORREF ink=kInk;
        if((id==Receipt&&!error_.empty())||id==SettingsError||id==RoiError)ink=kError;
        else if(id==MainNotice||(id==DirtyState&&dirty_))ink=kWarning;
        else if(std::find(small_labels_.begin(),small_labels_.end(),id)!=small_labels_.end())ink=kMuted;
        SetTextColor(dc,ink);SetBkMode(dc,TRANSPARENT);
        const bool band=id==FooterHint||id==Receipt;
        SetBkColor(dc,band?kBand:kPaper);
        if(message==WM_CTLCOLOREDIT)return reinterpret_cast<LRESULT>(paper_brush_);
        return reinterpret_cast<LRESULT>(band?band_brush_:paper_brush_);
    }
    static LRESULT CALLBACK button_proc(HWND hwnd,UINT message,WPARAM wp,LPARAM lp,UINT_PTR,DWORD_PTR data) {
        auto* self=reinterpret_cast<Desktop*>(data);
        if(message==WM_MOUSEMOVE){
            const int id=GetDlgCtrlID(hwnd);if(self->hover_id_!=id){const int previous=self->hover_id_;self->hover_id_=id;if(self->c(previous))InvalidateRect(self->c(previous),nullptr,TRUE);InvalidateRect(hwnd,nullptr,TRUE);}
            TRACKMOUSEEVENT tracking{sizeof(tracking),TME_LEAVE,hwnd,0};TrackMouseEvent(&tracking);
        }else if(message==WM_MOUSELEAVE){self->hover_id_=0;InvalidateRect(hwnd,nullptr,TRUE);}
        else if(message==WM_NCDESTROY)RemoveWindowSubclass(hwnd,button_proc,1);
        return DefSubclassProc(hwnd,message,wp,lp);
    }
    static LRESULT CALLBACK body_proc(HWND hwnd,UINT message,WPARAM wp,LPARAM lp) {
        auto* self=reinterpret_cast<Desktop*>(GetWindowLongPtrW(hwnd,GWLP_USERDATA));
        if(message==WM_NCCREATE){self=static_cast<Desktop*>(reinterpret_cast<CREATESTRUCTW*>(lp)->lpCreateParams);SetWindowLongPtrW(hwnd,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(self));}
        if(!self)return DefWindowProcW(hwnd,message,wp,lp);
        switch(message){
        case WM_COMMAND:case WM_DRAWITEM:case WM_CTLCOLORSTATIC:case WM_CTLCOLOREDIT:case WM_CTLCOLORBTN:case WM_NOTIFY:
            return SendMessageW(self->hwnd_,message,wp,lp);
        case WM_VSCROLL:self->scroll_body(LOWORD(wp));return 0;
        case WM_MOUSEWHEEL:self->scroll_body(0,GET_WHEEL_DELTA_WPARAM(wp));return 0;
        case WM_ERASEBKGND:{RECT r{};GetClientRect(hwnd,&r);FillRect(reinterpret_cast<HDC>(wp),&r,self->paper_brush_);return 1;}
        }
        return DefWindowProcW(hwnd,message,wp,lp);
    }
    static LRESULT CALLBACK main_proc(HWND hwnd,UINT message,WPARAM wp,LPARAM lp) {
        auto* self=reinterpret_cast<Desktop*>(GetWindowLongPtrW(hwnd,GWLP_USERDATA));
        if(message==WM_NCCREATE){self=static_cast<Desktop*>(reinterpret_cast<CREATESTRUCTW*>(lp)->lpCreateParams);self->hwnd_=hwnd;SetWindowLongPtrW(hwnd,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(self));}
        if(!self)return DefWindowProcW(hwnd,message,wp,lp);
        try{
            if(self->taskbar_message_&&message==self->taskbar_message_){self->add_tray();return 0;}
            switch(message){
            case WM_CREATE:self->create_controls();return 0;
            case WM_SIZE:self->layout();self->refresh_controls();return 0;
            case WM_PAINT:self->paint_main();return 0;
            case WM_ERASEBKGND:return 1;
            case WM_GETMINMAXINFO:reinterpret_cast<MINMAXINFO*>(lp)->ptMinTrackSize={self->px(760),self->px(520)};return 0;
            case WM_DPICHANGED:{self->dpi_=HIWORD(wp);self->make_fonts();const auto* rect=reinterpret_cast<RECT*>(lp);SetWindowPos(hwnd,nullptr,rect->left,rect->top,rect->right-rect->left,rect->bottom-rect->top,SWP_NOZORDER|SWP_NOACTIVATE);self->layout();return 0;}
            case WM_COMMAND:self->command(LOWORD(wp),HIWORD(wp));return 0;
            case WM_DRAWITEM:self->draw_button(*reinterpret_cast<DRAWITEMSTRUCT*>(lp));return TRUE;
            case WM_CTLCOLORSTATIC:case WM_CTLCOLOREDIT:case WM_CTLCOLORBTN:return self->colors(message,wp,lp);
            case WM_TIMER:self->tick();return 0;
            case kTaskDone:self->task_done(std::unique_ptr<TaskResult>(reinterpret_cast<TaskResult*>(lp)));return 0;
            case kTray:
                if(lp==WM_LBUTTONUP||lp==WM_LBUTTONDBLCLK){ShowWindow(hwnd,SW_RESTORE);SetForegroundWindow(hwnd);}
                else if(lp==WM_RBUTTONUP||lp==WM_CONTEXTMENU)self->tray_menu();
                return 0;
            case WM_CLOSE:self->request_navigation(Navigation::Close);return 0;
            case WM_QUERYENDSESSION:return TRUE;
            case WM_ENDSESSION:if(wp)DestroyWindow(hwnd);return 0;
            case WM_DESTROY:self->remove_tray();PostQuitMessage(0);return 0;
            }
        }catch(const std::exception& e){self->fail(e);}catch(...){self->error_=L"\u754c\u9762\u64cd\u4f5c\u5931\u8d25";self->refresh_controls();}
        return DefWindowProcW(hwnd,message,wp,lp);
    }
    static LRESULT CALLBACK image_proc(HWND hwnd,UINT message,WPARAM wp,LPARAM lp) {
        auto* self=reinterpret_cast<Desktop*>(GetWindowLongPtrW(hwnd,GWLP_USERDATA));
        if(message==WM_NCCREATE){self=static_cast<Desktop*>(reinterpret_cast<CREATESTRUCTW*>(lp)->lpCreateParams);SetWindowLongPtrW(hwnd,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(self));}
        if(!self)return DefWindowProcW(hwnd,message,wp,lp);
        try{
            if(message==WM_PAINT){self->paint_image(hwnd);return 0;}if(message==WM_ERASEBKGND)return 1;
            const int id=GetDlgCtrlID(hwnd);
            if(id==FullView){
                if(message==WM_LBUTTONDOWN){self->pointer_down(hwnd,lp);return 0;}
                if(message==WM_MOUSEMOVE){self->pointer_move(hwnd,lp);return 0;}
                if(message==WM_LBUTTONUP||message==WM_CAPTURECHANGED){self->pointer_up();return 0;}
                if(message==WM_SETCURSOR){
                    LPCWSTR cursor=IDC_ARROW;
                    if(self->mode_!=EditMode::Move)cursor=IDC_CROSS;
                    else{POINT point{};GetCursorPos(&point);ScreenToClient(hwnd,&point);const auto target=self->hit_boundary(hwnd,point);
                        if(target){switch(*target){case RoiTarget::NW:case RoiTarget::SE:cursor=IDC_SIZENWSE;break;case RoiTarget::NE:case RoiTarget::SW:cursor=IDC_SIZENESW;break;case RoiTarget::N:case RoiTarget::S:cursor=IDC_SIZENS;break;default:cursor=IDC_SIZEWE;break;}}
                        else cursor=IDC_SIZEALL;}
                    SetCursor(LoadCursorW(nullptr,cursor));return TRUE;
                }
            }
            if(id==FullView||id==MagnifierView){
                if(message==WM_LBUTTONDOWN&&id==MagnifierView){SetFocus(hwnd);return 0;}
                if(message==WM_GETDLGCODE)return DLGC_WANTARROWS;
                if(message==WM_KEYDOWN){
                    if(wp==VK_ESCAPE){self->cancel_drag();self->set_mode(EditMode::Move);return 0;}
                    if(wp==VK_LEFT){self->nudge(-1,0);return 0;}if(wp==VK_UP){self->nudge(0,-1);return 0;}
                    if(wp==VK_DOWN){self->nudge(0,1);return 0;}if(wp==VK_RIGHT){self->nudge(1,0);return 0;}
                }
            }
        }catch(const std::exception& e){self->fail(e);}
        return DefWindowProcW(hwnd,message,wp,lp);
    }

    HINSTANCE instance_{};HWND hwnd_{},body_{},tooltip_{};
    HFONT font_{},heading_font_{},small_font_{};HBRUSH paper_brush_{},band_brush_{};
    int dpi_=96,scroll_=0,content_height_=0,pending_=0,hover_id_=0;
    std::map<int,HWND> controls_;std::map<int,Button> buttons_;std::vector<int> small_labels_;
    std::map<std::tuple<int,int,COLORREF>,HBITMAP> icons_;
    CaptureController controller_;Profile saved_,draft_;Status status_;
    std::shared_ptr<const PreviewFrame> frame_,reference_;
    std::vector<std::uint8_t> main_crop_,edit_crop_;std::vector<Rect> history_;
    std::string active_source_,deferred_roi_source_;
    std::wstring error_,notice_;
    Page page_=Page::Main;SettingTab setting_tab_=SettingTab::Source;EditMode mode_=EditMode::Move;
    bool syncing_=false,laying_out_=false,advanced_=false,scale_expanded_=false,inspect_ruler_=false;
    bool dirty_=false,invalid_roi_=false,invalid_settings_=false,profile_read_error_=false;
    bool runtime_review_required_=false,reference_changed_=false,pending_auto_=false,send_requested_=false;
    bool review_touched_=false,enter_roi_when_ready_=false;
    bool source_started_=false,exiting_=false,tray_added_=false,dragging_=false,drag_changed_=false;
    RoiTarget active_edge_=RoiTarget::NW,drag_target_=RoiTarget::Move;
    PixelPoint drag_start_{};Rect drag_old_{};bool drag_draw_=false,drag_old_confirmed_=false,drag_old_auto_=false;
    double anchor_x_=.5,anchor_y_=.5;
    std::array<RulerDraft,2> rulers_;int active_axis_=0,active_tick_=0,zoom_=4;
    UINT taskbar_message_=0;
    std::thread worker_;std::mutex task_mutex_;std::condition_variable task_changed_;std::deque<Job> tasks_;bool quitting_=false;
};
} // namespace

int WINAPI wWinMain(HINSTANCE instance,HINSTANCE,PWSTR,int show) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    HANDLE singleton=CreateMutexW(nullptr,FALSE,L"Local\\PausUltrasoundCaptureDesktop");
    if(singleton&&GetLastError()==ERROR_ALREADY_EXISTS){
        if(HWND existing=FindWindowW(kMainClass,nullptr)){ShowWindow(existing,SW_RESTORE);SetForegroundWindow(existing);}
        CloseHandle(singleton);return 0;
    }
    const HRESULT apartment=RoInitialize(RO_INIT_SINGLETHREADED);int code=1;
    try{Desktop desktop(instance);code=desktop.run(show);}catch(const std::exception& e){MessageBoxW(nullptr,wide(e.what()).c_str(),L"Ultrasound Capture",MB_OK|MB_ICONERROR);}
    if(SUCCEEDED(apartment))RoUninitialize();
    if(singleton)CloseHandle(singleton);
    return code;
}
