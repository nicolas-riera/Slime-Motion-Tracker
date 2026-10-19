// Slime Motion Tracker dashboard overlay.
//
// One Dear ImGui UI is rendered into an offscreen D3D11 texture, submitted to a SteamVR
// dashboard overlay and copied into the back buffer of a regular desktop window, so the
// same UI works with and without a headset. While the dashboard shows it, the UI renders
// at the fixed overlay size (1280x720, stretched in the desktop window); otherwise at the
// desktop window's client size (resizable, the layout adapts). Nothing is rendered while
// neither is visible (window minimized or hidden in the notification area).
//
// Input: the desktop window feeds ImGui through the Win32 backend; SteamVR laser-pointer
// events (mouse move / buttons / scroll) are injected into the same ImGui IO.

#include "driver_link.h"  // includes winsock2.h, which must come before windows.h

#include <windows.h>
#include <windowsx.h>
#include <shellapi.h>
#include <d3d11.h>
#include <dxgi1_2.h>
#include <tlhelp32.h>
#include <wrl/client.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <openvr.h>

#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "app_prefs.h"
#include "ui.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {

using Microsoft::WRL::ComPtr;
using Clock = std::chrono::steady_clock;
using namespace smt;

constexpr wchar_t kWindowClass[] = L"SlimeMotionTrackerWindow";
constexpr wchar_t kWindowTitle[] = L"Slime Motion Tracker";
constexpr wchar_t kMutexName[] = L"SlimeMotionTrackerApp";
constexpr DXGI_FORMAT kFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
constexpr auto kFramePeriod = std::chrono::microseconds(16667);  // ~60 Hz
constexpr auto kIdlePeriod = std::chrono::milliseconds(100);  // nothing visible: link, sounds, VR events
constexpr UINT WM_APP_TRAY = WM_APP + 1;  // notification area icon events
constexpr UINT WM_APP_SHOW = WM_APP + 2;  // another instance asks us to show the window
constexpr UINT kMenuOpen = 1001;
constexpr UINT kMenuQuit = 1002;
constexpr auto kVrRetryPeriod = std::chrono::seconds(2);
constexpr auto kFloorPeriod = std::chrono::seconds(1);

// ------------------------------------------------------------------ D3D11

struct Gpu {
    ComPtr<ID3D11Device> device;
    ComPtr<ID3D11DeviceContext> context;
    ComPtr<IDXGISwapChain1> swapChain;
    ComPtr<ID3D11Texture2D> uiTexture;  // offscreen UI render target, shared by desktop + VR
    ComPtr<ID3D11RenderTargetView> uiRtv;
    LUID adapterLuid{};
    int uiWidth = 0, uiHeight = 0;  // UI texture size
    int scWidth = 0, scHeight = 0;  // swap chain buffers size

    void reset() { *this = Gpu{}; }
};

// Returns the DXGI adapter with the given index, or null (= default adapter).
ComPtr<IDXGIAdapter1> adapterByIndex(int32_t index) {
    if (index < 0) return nullptr;
    ComPtr<IDXGIFactory1> factory;
    if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) return nullptr;
    ComPtr<IDXGIAdapter1> adapter;
    if (FAILED(factory->EnumAdapters1(static_cast<UINT>(index), &adapter))) return nullptr;
    return adapter;
}

// (Re)creates the offscreen UI texture at the given size.
bool createUiTarget(Gpu& gpu, int width, int height) {
    gpu.uiRtv.Reset();
    gpu.uiTexture.Reset();
    gpu.uiWidth = gpu.uiHeight = 0;
    D3D11_TEXTURE2D_DESC td{};
    td.Width = static_cast<UINT>(width);
    td.Height = static_cast<UINT>(height);
    td.MipLevels = 1;
    td.ArraySize = 1;
    td.Format = kFormat;
    td.SampleDesc.Count = 1;
    td.Usage = D3D11_USAGE_DEFAULT;
    td.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    if (FAILED(gpu.device->CreateTexture2D(&td, nullptr, &gpu.uiTexture))) return false;
    if (FAILED(gpu.device->CreateRenderTargetView(gpu.uiTexture.Get(), nullptr, &gpu.uiRtv))) return false;
    gpu.uiWidth = width;
    gpu.uiHeight = height;
    return true;
}

// Swap chain buffers of the UI size (the back buffer receives a CopyResource of the UI
// texture; DXGI stretches it to the window when the sizes differ).
bool resizeSwapChain(Gpu& gpu, int width, int height) {
    if (gpu.scWidth == width && gpu.scHeight == height) return true;
    if (FAILED(gpu.swapChain->ResizeBuffers(0, static_cast<UINT>(width), static_cast<UINT>(height), DXGI_FORMAT_UNKNOWN,
                                            0)))
        return false;
    gpu.scWidth = width;
    gpu.scHeight = height;
    return true;
}

bool createGpu(HWND hwnd, int32_t adapterIndex, Gpu& gpu) {
    gpu.reset();
    ComPtr<IDXGIAdapter1> adapter = adapterByIndex(adapterIndex);

    UINT flags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0};
    HRESULT hr = D3D11CreateDevice(adapter.Get(), adapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE,
                                   nullptr, flags, levels, static_cast<UINT>(std::size(levels)),
                                   D3D11_SDK_VERSION, &gpu.device, nullptr, &gpu.context);
    if (FAILED(hr)) {
        // No usable GPU (e.g. remote desktop): fall back to the software rasterizer.
        hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, flags, levels,
                               static_cast<UINT>(std::size(levels)), D3D11_SDK_VERSION, &gpu.device, nullptr,
                               &gpu.context);
    }
    if (FAILED(hr)) return false;

    // Swap chain created from the device's own factory.
    ComPtr<IDXGIDevice> dxgiDevice;
    ComPtr<IDXGIAdapter> dxgiAdapter;
    ComPtr<IDXGIFactory2> factory;
    if (FAILED(gpu.device.As(&dxgiDevice)) || FAILED(dxgiDevice->GetAdapter(&dxgiAdapter)) ||
        FAILED(dxgiAdapter->GetParent(IID_PPV_ARGS(&factory))))
        return false;
    DXGI_ADAPTER_DESC adapterDesc{};
    dxgiAdapter->GetDesc(&adapterDesc);
    gpu.adapterLuid = adapterDesc.AdapterLuid;

    DXGI_SWAP_CHAIN_DESC1 sc{};
    sc.Width = ui::kWidth;
    sc.Height = ui::kHeight;
    sc.Format = kFormat;  // same format as the UI texture -> CopyResource (sizes follow it)
    sc.SampleDesc.Count = 1;
    sc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sc.BufferCount = 2;
    sc.Scaling = DXGI_SCALING_STRETCH;
    sc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    sc.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
    if (FAILED(factory->CreateSwapChainForHwnd(gpu.device.Get(), hwnd, &sc, nullptr, nullptr, &gpu.swapChain)))
        return false;
    factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
    gpu.scWidth = ui::kWidth;
    gpu.scHeight = ui::kHeight;
    return createUiTarget(gpu, ui::kWidth, ui::kHeight);
}

bool sameLuid(const LUID& a, const LUID& b) { return a.LowPart == b.LowPart && a.HighPart == b.HighPart; }

LUID adapterLuidByIndex(int32_t index) {
    LUID luid{};
    if (ComPtr<IDXGIAdapter1> adapter = adapterByIndex(index)) {
        DXGI_ADAPTER_DESC1 desc{};
        adapter->GetDesc1(&desc);
        luid = desc.AdapterLuid;
    }
    return luid;
}

// ------------------------------------------------------------------ SteamVR

bool isProcessRunning(const wchar_t* exeName) {
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) return false;
    PROCESSENTRY32W entry{};
    entry.dwSize = sizeof(entry);
    bool found = false;
    for (BOOL ok = Process32FirstW(snapshot, &entry); ok; ok = Process32NextW(snapshot, &entry)) {
        if (_wcsicmp(entry.szExeFile, exeName) == 0) {
            found = true;
            break;
        }
    }
    CloseHandle(snapshot);
    return found;
}

// ------------------------------------------------------------------ standalone engine

// Without SteamVR the overlay runs the engine itself (smt_engine_host with a
// fake headset, next to the overlay), so the trackers and the dongle can be checked
// without a headset. The host exits with the overlay and as soon as SteamVR starts,
// leaving the tracker port and the dongle to the driver.
class StandaloneEngine {
public:
    ~StandaloneEngine() {
        if (process_) CloseHandle(process_);
    }

    void update(bool driverConnected) {
        const auto now = Clock::now();
        if (now < nextCheck_) return;
        nextCheck_ = now + std::chrono::seconds(1);
        if (process_ && WaitForSingleObject(process_, 0) == WAIT_OBJECT_0) {
            CloseHandle(process_);
            process_ = nullptr;
        }
        if (process_ || driverConnected || isProcessRunning(L"vrserver.exe")) {
            missingSince_ = now;
            return;
        }
        if (now - missingSince_ >= std::chrono::seconds(3)) start();
    }

    [[nodiscard]] bool running() const { return process_ != nullptr; }

private:
    void start() {
        wchar_t path[MAX_PATH];
        const DWORD n = GetModuleFileNameW(nullptr, path, MAX_PATH);
        if (n == 0 || n == MAX_PATH) return;
        std::wstring dir(path, n);
        dir.resize(dir.find_last_of(L'\\') + 1);
        const std::wstring host = dir + L"smt_engine_host.exe";
        const std::wstring engine = dir + L"smt_engine.dll";
        if (GetFileAttributesW(host.c_str()) == INVALID_FILE_ATTRIBUTES ||
            GetFileAttributesW(engine.c_str()) == INVALID_FILE_ATTRIBUTES)
            return;
        std::wstring cmd = L"\"" + host + L"\" \"" + engine + L"\" --parent " +
                           std::to_wstring(GetCurrentProcessId()) + L" --until-steamvr";
        // The engine's log goes to %LOCALAPPDATA%\Slime Motion Tracker\standalone.log
        HANDLE logFile = INVALID_HANDLE_VALUE;
        if (const wchar_t* appData = _wgetenv(L"LOCALAPPDATA")) {
            SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
            const std::wstring logPath = std::wstring(appData) + L"\\Slime Motion Tracker\\standalone.log";
            logFile = CreateFileW(logPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ, &sa, CREATE_ALWAYS,
                                  FILE_ATTRIBUTE_NORMAL, nullptr);
        }
        STARTUPINFOW si{};
        si.cb = sizeof(si);
        if (logFile != INVALID_HANDLE_VALUE) {
            si.dwFlags = STARTF_USESTDHANDLES;
            si.hStdOutput = si.hStdError = logFile;
        }
        PROCESS_INFORMATION pi{};
        const BOOL ok = CreateProcessW(host.c_str(), cmd.data(), nullptr, nullptr, logFile != INVALID_HANDLE_VALUE,
                                       CREATE_NO_WINDOW, nullptr, dir.c_str(), &si, &pi);
        if (logFile != INVALID_HANDLE_VALUE) CloseHandle(logFile);
        if (!ok) return;
        CloseHandle(pi.hThread);
        process_ = pi.hProcess;
    }

    HANDLE process_ = nullptr;
    Clock::time_point nextCheck_{};
    Clock::time_point missingSince_ = Clock::now();
};

// 128x128 RGBA dashboard thumbnail: a rounded accent square with "SMT" in a 5x7 pixel font
// (same design as the app icon, tools/icon/make_icon.py).
std::vector<uint8_t> makeThumbnail(int size) {
    static constexpr std::array<const char*, 7> kS = {".####", "#....", "#....", ".###.", "....#", "....#", "####."};
    static constexpr std::array<const char*, 7> kM = {"#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"};
    static constexpr std::array<const char*, 7> kT = {"#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."};
    std::vector<uint8_t> px(static_cast<size_t>(size) * size * 4, 0);
    const auto put = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        uint8_t* p = &px[(static_cast<size_t>(y) * size + x) * 4];
        p[0] = r, p[1] = g, p[2] = b, p[3] = 255;
    };

    const int radius = size / 6;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            // Distance to the nearest corner center, for the rounded corners.
            const int cx = x < radius ? radius : (x >= size - radius ? size - radius - 1 : x);
            const int cy = y < radius ? radius : (y >= size - radius ? size - radius - 1 : y);
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) put(x, y, 0x5a, 0x4c, 0xd8);
        }
    }

    const int scale = size / 20;  // glyph pixel size
    const int glyphW = 5 * scale, glyphH = 7 * scale, gap = scale;
    const int x0 = (size - (3 * glyphW + 2 * gap)) / 2;
    const int y0 = (size - glyphH) / 2;
    const auto drawGlyph = [&](const std::array<const char*, 7>& glyph, int left) {
        for (int gy = 0; gy < 7; ++gy)
            for (int gx = 0; gx < 5; ++gx)
                if (glyph[static_cast<size_t>(gy)][gx] == '#')
                    for (int y = 0; y < scale; ++y)
                        for (int x = 0; x < scale; ++x) put(left + gx * scale + x, y0 + gy * scale + y, 255, 255, 255);
    };
    drawGlyph(kS, x0);
    drawGlyph(kM, x0 + glyphW + gap);
    drawGlyph(kT, x0 + 2 * (glyphW + gap));
    return px;
}

class VrOverlay {
public:
    ~VrOverlay() { shutdown(); }

    [[nodiscard]] bool active() const { return active_; }

    // Connects to SteamVR and creates the dashboard overlay. Does nothing (and does not
    // launch SteamVR) when SteamVR is not running.
    bool tryInit() {
        if (active_) return true;
        if (!isProcessRunning(L"vrserver.exe")) return false;

        vr::EVRInitError err = vr::VRInitError_None;
        vr::VR_Init(&err, vr::VRApplication_Overlay);
        if (err != vr::VRInitError_None) return false;
        if (!vr::VROverlay() || !vr::VRSystem()) {
            vr::VR_Shutdown();
            return false;
        }

        vr::IVROverlay* ov = vr::VROverlay();
        if (ov->CreateDashboardOverlay("slime_motion_tracker.dashboard", "Slime Motion Tracker", &handle_, &thumbnail_) !=
            vr::VROverlayError_None) {
            vr::VR_Shutdown();
            return false;
        }
        ov->SetOverlayWidthInMeters(handle_, 2.0f);
        ov->SetOverlayInputMethod(handle_, vr::VROverlayInputMethod_Mouse);
        const vr::HmdVector2_t mouseScale{{static_cast<float>(ui::kWidth), static_cast<float>(ui::kHeight)}};
        ov->SetOverlayMouseScale(handle_, &mouseScale);
        ov->SetOverlayFlag(handle_, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

        constexpr int kThumb = 128;
        std::vector<uint8_t> thumb = makeThumbnail(kThumb);
        ov->SetOverlayRaw(thumbnail_, thumb.data(), kThumb, kThumb, 4);

        active_ = true;
        submitted_ = false;
        lastFloor_ = {};
        return true;
    }

    void shutdown() {
        if (!active_) return;
        vr::VR_Shutdown();  // also destroys the overlays owned by this process
        active_ = false;
        handle_ = thumbnail_ = vr::k_ulOverlayHandleInvalid;
    }

    // DXGI adapter index of the HMD's GPU (-1 when unknown).
    [[nodiscard]] int32_t adapterIndex() const {
        int32_t index = -1;
        if (active_) vr::VRSystem()->GetDXGIOutputInfo(&index);
        return index;
    }

    // Drains SteamVR events, forwarding laser-pointer input to ImGui. Returns false when
    // SteamVR asks applications to quit.
    bool pollEvents(ImGuiIO& io) {
        if (!active_) return true;
        bool keepRunning = true;
        vr::VREvent_t ev{};
        while (vr::VRSystem()->PollNextEvent(&ev, sizeof(ev))) {
            if (ev.eventType == vr::VREvent_Quit) keepRunning = false;
        }
        while (vr::VROverlay()->PollNextOverlayEvent(handle_, &ev, sizeof(ev))) {
            switch (ev.eventType) {
            case vr::VREvent_MouseMove:
                addMousePos(io, ev.data.mouse);
                break;
            case vr::VREvent_MouseButtonDown:
            case vr::VREvent_MouseButtonUp:
                addMousePos(io, ev.data.mouse);
                if (const int button = imguiButton(ev.data.mouse.button); button >= 0) {
                    io.AddMouseButtonEvent(button, ev.eventType == vr::VREvent_MouseButtonDown);
                    vrButtons_ = ev.eventType == vr::VREvent_MouseButtonDown ? (vrButtons_ | (1u << button))
                                                                           : (vrButtons_ & ~(1u << button));
                }
                break;
            case vr::VREvent_ScrollDiscrete:
                io.AddMouseWheelEvent(ev.data.scroll.xdelta, ev.data.scroll.ydelta);
                break;
            case vr::VREvent_FocusLeave:
                // Laser left the overlay: release anything still held and hide the cursor.
                for (int b = 0; b < 3; ++b)
                    if (vrButtons_ & (1u << b)) io.AddMouseButtonEvent(b, false);
                vrButtons_ = 0;
                io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
                break;
            case vr::VREvent_Quit:
                keepRunning = false;
                break;
            default:
                break;
            }
        }
        // SteamVR is shutting down: tell it we are exiting (extends the kill timeout).
        if (!keepRunning) vr::VRSystem()->AcknowledgeQuit_Exiting();
        return keepRunning;
    }

    // Sends the floor height to the driver once per second (docs/IPC.md, `floor`).
    void updateFloor(DriverLink& link) {
        if (!active_) return;
        const auto now = Clock::now();
        if (now - lastFloor_ < kFloorPeriod) return;
        lastFloor_ = now;

        bool valid = true;
        if (vr::IVRChaperone* chaperone = vr::VRChaperone())
            valid = chaperone->GetCalibrationState() < vr::ChaperoneCalibrationState_Error;
        const vr::HmdMatrix34_t m = vr::VRSystem()->GetRawZeroPoseToStandingAbsoluteTrackingPose();
        link.sendFloor(valid, -m.m[1][3]);
    }

    // The dashboard tab is shown
    [[nodiscard]] bool visible() const { return active_ && vr::VROverlay()->IsOverlayVisible(handle_); }
    // No texture submitted yet since the overlay was created
    [[nodiscard]] bool needsTexture() const { return active_ && !submitted_; }

    // Submits the UI texture (overlay size), while visible and once after creation.
    void submit(ID3D11Texture2D* texture) {
        if (!active_) return;
        vr::Texture_t tex{texture, vr::TextureType_DirectX, vr::ColorSpace_Gamma};
        submitted_ = vr::VROverlay()->SetOverlayTexture(handle_, &tex) == vr::VROverlayError_None;
    }

private:
    static void addMousePos(ImGuiIO& io, const vr::VREvent_Mouse_t& mouse) {
        // OpenVR mouse coordinates are bottom-up, ImGui's are top-down.
        io.AddMousePosEvent(mouse.x, static_cast<float>(ui::kHeight) - mouse.y);
    }

    static int imguiButton(uint32_t vrButton) {
        switch (vrButton) {
        case vr::VRMouseButton_Left: return ImGuiMouseButton_Left;
        case vr::VRMouseButton_Right: return ImGuiMouseButton_Right;
        case vr::VRMouseButton_Middle: return ImGuiMouseButton_Middle;
        default: return -1;
        }
    }

    bool active_ = false;
    bool submitted_ = false;
    uint32_t vrButtons_ = 0;
    vr::VROverlayHandle_t handle_ = vr::k_ulOverlayHandleInvalid;
    vr::VROverlayHandle_t thumbnail_ = vr::k_ulOverlayHandleInvalid;
    Clock::time_point lastFloor_{};
};

// ------------------------------------------------------------------ Win32

constexpr DWORD kWindowStyle = WS_OVERLAPPEDWINDOW;

struct WindowState {
    HWND hwnd = nullptr;
    bool inTray = false;  // hidden, with an icon in the notification area
    UINT taskbarCreated = 0;  // Explorer restarted: the tray icon must be added again
    // UI pixels per client pixel (the UI renders at the VR size while the dashboard shows it)
    float mouseScaleX = 1.0f, mouseScaleY = 1.0f;
};
WindowState g_win;

void addTrayIcon() {
    NOTIFYICONDATAW nid{};
    nid.cbSize = sizeof(nid);
    nid.hWnd = g_win.hwnd;
    nid.uID = 1;
    nid.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
    nid.uCallbackMessage = WM_APP_TRAY;
    // Icon resource 1 (overlay/resources/overlay.rc)
    nid.hIcon = static_cast<HICON>(LoadImageW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(1), IMAGE_ICON,
                                              GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), 0));
    wcscpy_s(nid.szTip, kWindowTitle);
    Shell_NotifyIconW(NIM_ADD, &nid);
    if (nid.hIcon) DestroyIcon(nid.hIcon);
}

void removeTrayIcon() {
    NOTIFYICONDATAW nid{};
    nid.cbSize = sizeof(nid);
    nid.hWnd = g_win.hwnd;
    nid.uID = 1;
    Shell_NotifyIconW(NIM_DELETE, &nid);
}

void hideToTray() {
    if (!g_win.inTray) addTrayIcon();
    g_win.inTray = true;
    ShowWindow(g_win.hwnd, SW_HIDE);
}

void showWindow() {
    if (g_win.inTray) removeTrayIcon();
    g_win.inTray = false;
    ShowWindow(g_win.hwnd, SW_SHOW);
    if (IsIconic(g_win.hwnd)) ShowWindow(g_win.hwnd, SW_RESTORE);
    SetForegroundWindow(g_win.hwnd);
}

void showTrayMenu() {
    HMENU menu = CreatePopupMenu();
    if (!menu) return;
    AppendMenuW(menu, MF_STRING, kMenuOpen, L"Open");
    AppendMenuW(menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(menu, MF_STRING, kMenuQuit, L"Quit");
    SetMenuDefaultItem(menu, kMenuOpen, FALSE);
    POINT pt{};
    GetCursorPos(&pt);
    SetForegroundWindow(g_win.hwnd);  // so the menu closes when clicking elsewhere
    TrackPopupMenu(menu, TPM_RIGHTBUTTON | TPM_BOTTOMALIGN, pt.x, pt.y, 0, g_win.hwnd, nullptr);
    PostMessageW(g_win.hwnd, WM_NULL, 0, 0);
    DestroyMenu(menu);
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    // The UI may render at another size than the client area (VR size): scale the mouse
    if (msg == WM_MOUSEMOVE && (g_win.mouseScaleX != 1.0f || g_win.mouseScaleY != 1.0f)) {
        const int x = static_cast<int>(static_cast<float>(GET_X_LPARAM(lParam)) * g_win.mouseScaleX);
        const int y = static_cast<int>(static_cast<float>(GET_Y_LPARAM(lParam)) * g_win.mouseScaleY);
        lParam = MAKELPARAM(x, y);
    }
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wParam, lParam)) return true;
    if (g_win.taskbarCreated != 0 && msg == g_win.taskbarCreated) {
        if (g_win.inTray) addTrayIcon();
        return 0;
    }
    switch (msg) {
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) return 0;  // no ALT application menu
        if ((wParam & 0xfff0) == SC_MINIMIZE && appPrefs().minimizeToTray) {
            hideToTray();
            return 0;
        }
        break;
    case WM_GETMINMAXINFO: {
        RECT r{0, 0, ui::kMinWidth, ui::kMinHeight};
        AdjustWindowRect(&r, kWindowStyle, FALSE);
        auto* info = reinterpret_cast<MINMAXINFO*>(lParam);
        info->ptMinTrackSize = POINT{r.right - r.left, r.bottom - r.top};
        return 0;
    }
    case WM_APP_TRAY:
        switch (LOWORD(lParam)) {
        case WM_LBUTTONUP:
        case WM_LBUTTONDBLCLK:
            showWindow();
            break;
        case WM_RBUTTONUP:
        case WM_CONTEXTMENU:
            showTrayMenu();
            break;
        default:
            break;
        }
        return 0;
    case WM_APP_SHOW:
        showWindow();
        return 0;
    case WM_COMMAND:
        if (LOWORD(wParam) == kMenuOpen) showWindow();
        if (LOWORD(wParam) == kMenuQuit) DestroyWindow(hwnd);
        return 0;
    case WM_DESTROY:
        if (g_win.inTray) removeTrayIcon();
        g_win.inTray = false;
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

HWND createMainWindow(HINSTANCE instance) {
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = wndProc;
    wc.hInstance = instance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    // Icon resource 1 (overlay/resources/overlay.rc)
    wc.hIcon = LoadIconW(instance, MAKEINTRESOURCEW(1));
    wc.hIconSm = static_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(1), IMAGE_ICON,
                                               GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), 0));
    wc.lpszClassName = kWindowClass;
    if (!RegisterClassExW(&wc)) return nullptr;

    // Resizable; starts with the client area of the VR overlay size
    RECT rect{0, 0, ui::kWidth, ui::kHeight};
    AdjustWindowRect(&rect, kWindowStyle, FALSE);
    return CreateWindowExW(0, kWindowClass, kWindowTitle, kWindowStyle, CW_USEDEFAULT, CW_USEDEFAULT,
                           rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, instance, nullptr);
}

// Sleeps until `deadline` while still dispatching window messages as they arrive.
// Returns false if WM_QUIT was received.
bool pumpMessagesUntil(HANDLE timer, Clock::time_point deadline) {
    if (timer) {
        const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - Clock::now());
        LARGE_INTEGER due{};
        due.QuadPart = -std::max<long long>(remaining.count() / 100, 0);  // relative, 100 ns units
        SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE);
    }
    for (;;) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) return false;
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (Clock::now() >= deadline) return true;
        if (timer) {
            if (MsgWaitForMultipleObjects(1, &timer, FALSE, INFINITE, QS_ALLINPUT) == WAIT_OBJECT_0) return true;
        } else {
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
            MsgWaitForMultipleObjects(0, nullptr, FALSE, static_cast<DWORD>(std::max<long long>(ms, 0)),
                                      QS_ALLINPUT);
        }
    }
}

}  // namespace

int WINAPI wWinMain(_In_ HINSTANCE instance, _In_opt_ HINSTANCE, _In_ PWSTR, _In_ int showCmd) {
    // Single instance: the driver may start us while the user already started one. The
    // running one shows its window (it may be hidden in the notification area).
    HANDLE mutex = CreateMutexW(nullptr, FALSE, kMutexName);
    if (!mutex) return 1;
    if (GetLastError() == ERROR_ALREADY_EXISTS) {
        if (HWND other = FindWindowW(kWindowClass, nullptr)) PostMessageW(other, WM_APP_SHOW, 0, 0);
        CloseHandle(mutex);
        return 0;
    }

    AppPrefs& prefs = appPrefs();
    prefs.load();

    DriverLink link;
    link.start();  // on failure the UI simply waits for the engine

    HWND hwnd = createMainWindow(instance);
    if (!hwnd) return 1;
    g_win.hwnd = hwnd;
    g_win.taskbarCreated = RegisterWindowMessageW(L"TaskbarCreated");

    // Connect to SteamVR first so the D3D device can be created on the HMD's GPU.
    VrOverlay vrOverlay;
    vrOverlay.tryInit();
    auto nextVrRetry = Clock::now() + kVrRetryPeriod;
    StandaloneEngine standalone;

    Gpu gpu;
    if (!createGpu(hwnd, vrOverlay.adapterIndex(), gpu)) {
        MessageBoxW(hwnd, L"Could not initialize Direct3D 11.", kWindowTitle, MB_ICONERROR);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ui::setup();
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(gpu.device.Get(), gpu.context.Get());

    if (prefs.startInTray) {
        hideToTray();
    } else {
        ShowWindow(hwnd, showCmd);
        UpdateWindow(hwnd);
    }

    HANDLE frameTimer =
        CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    if (!frameTimer) frameTimer = CreateWaitableTimerW(nullptr, TRUE, nullptr);

    bool running = true;
    auto nextFrame = Clock::now();
    while (running) {
        link.poll();
        standalone.update(link.connected());

        if (!vrOverlay.active() && Clock::now() >= nextVrRetry) {
            nextVrRetry = Clock::now() + kVrRetryPeriod;
            // SteamVR came up later: if the HMD is on another GPU (laptops), recreate the
            // device there so SetOverlayTexture gets a texture from the right adapter.
            if (vrOverlay.tryInit() && vrOverlay.adapterIndex() >= 0 &&
                !sameLuid(adapterLuidByIndex(vrOverlay.adapterIndex()), gpu.adapterLuid)) {
                ImGui_ImplDX11_Shutdown();
                if (!createGpu(hwnd, vrOverlay.adapterIndex(), gpu) && !createGpu(hwnd, -1, gpu)) break;
                ImGui_ImplDX11_Init(gpu.device.Get(), gpu.context.Get());
            }
        }

        vrOverlay.updateFloor(link);

        // What is visible decides what (and whether) to render
        RECT client{};
        GetClientRect(hwnd, &client);
        const int clientW = client.right - client.left, clientH = client.bottom - client.top;
        const bool desktopVisible = IsWindowVisible(hwnd) && !IsIconic(hwnd) && clientW > 0 && clientH > 0;
        const bool vrVisible = vrOverlay.visible();
        const bool vrNeedsTexture = vrOverlay.needsTexture();
        const bool vrSize = vrVisible || (vrNeedsTexture && !desktopVisible);
        const bool render = desktopVisible || vrVisible || vrNeedsTexture;
        ui::update(link, render);

        if (render) {
            const int width = vrSize ? ui::kWidth : clientW;
            const int height = vrSize ? ui::kHeight : clientH;
            if ((width != gpu.uiWidth || height != gpu.uiHeight) && !createUiTarget(gpu, width, height)) break;
            if (desktopVisible && !resizeSwapChain(gpu, width, height)) break;
            g_win.mouseScaleX = desktopVisible ? static_cast<float>(width) / static_cast<float>(clientW) : 1.0f;
            g_win.mouseScaleY = desktopVisible ? static_cast<float>(height) / static_cast<float>(clientH) : 1.0f;

            ImGui_ImplDX11_NewFrame();
            ImGui_ImplWin32_NewFrame();
            // VR input is queued after the Win32 backend's so the laser pointer wins.
            if (!vrOverlay.pollEvents(io)) running = false;
            io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));
            ImGui::NewFrame();
            ui::draw(link, ui::FrameInfo{.vrActive = vrOverlay.active(), .standalone = standalone.running()});
            ImGui::Render();

            // Render the UI offscreen.
            ID3D11RenderTargetView* rtv = gpu.uiRtv.Get();
            const float clear[4] = {0.063f, 0.063f, 0.086f, 1.0f};
            gpu.context->OMSetRenderTargets(1, &rtv, nullptr);
            gpu.context->ClearRenderTargetView(rtv, clear);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
            gpu.context->OMSetRenderTargets(0, nullptr, nullptr);

            // Desktop window: copy the texture into the back buffer.
            if (desktopVisible) {
                ComPtr<ID3D11Texture2D> backBuffer;
                if (SUCCEEDED(gpu.swapChain->GetBuffer(0, IID_PPV_ARGS(&backBuffer))))
                    gpu.context->CopyResource(backBuffer.Get(), gpu.uiTexture.Get());
                const HRESULT hr = gpu.swapChain->Present(0, 0);
                if (hr == DXGI_ERROR_DEVICE_REMOVED || hr == DXGI_ERROR_DEVICE_RESET) break;
            }

            // SteamVR dashboard (always the overlay size).
            if (running && vrSize) vrOverlay.submit(gpu.uiTexture.Get());
        } else {
            g_win.mouseScaleX = g_win.mouseScaleY = 1.0f;
            if (!vrOverlay.pollEvents(io)) running = false;  // SteamVR quit
        }

        nextFrame += render ? std::chrono::duration_cast<Clock::duration>(kFramePeriod)
                            : std::chrono::duration_cast<Clock::duration>(kIdlePeriod);
        if (nextFrame < Clock::now()) nextFrame = Clock::now();  // don't try to catch up after a stall
        if (running && !pumpMessagesUntil(frameTimer, nextFrame)) running = false;
    }

    ui::shutdown();
    vrOverlay.shutdown();
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    gpu.reset();
    link.stop();
    if (frameTimer) CloseHandle(frameTimer);
    if (IsWindow(hwnd)) DestroyWindow(hwnd);
    if (g_win.inTray) removeTrayIcon();
    CloseHandle(mutex);
    return 0;
}
