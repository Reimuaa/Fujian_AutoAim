#include "win_capture.hpp"

#include <windows.h>

#include <d3d11.h>
#include <dxgi1_2.h>
#include <psapi.h>

#include <atlbase.h>  // CComPtr

#include <algorithm>
#include <cstdio>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

namespace io::gestalt
{
namespace
{
struct FoundWindow
{
  HWND hwnd = nullptr;
  RECT screen_rect{};  // client rect in screen coords
  std::string title;
};

std::string exe_of_pid(DWORD pid)
{
  HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
  if (!h) return "";
  char buf[MAX_PATH]{};
  DWORD size = MAX_PATH;
  std::string out;
  if (QueryFullProcessImageNameA(h, 0, buf, &size)) {
    out = buf;
    auto pos = out.find_last_of("\\/");
    if (pos != std::string::npos) out = out.substr(pos + 1);
    std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  }
  CloseHandle(h);
  return out;
}

BOOL CALLBACK enum_cb(HWND hwnd, LPARAM lp)
{
  auto * found = reinterpret_cast<FoundWindow *>(lp);
  if (!IsWindowVisible(hwnd)) return TRUE;
  char title[512]{};
  if (!GetWindowTextA(hwnd, title, sizeof(title)) || !title[0]) return TRUE;
  DWORD pid = 0;
  GetWindowThreadProcessId(hwnd, &pid);
  std::string exe = exe_of_pid(pid);
  if (exe.rfind("unrealeditor", 0) != 0 && exe.rfind("gestalt", 0) != 0 &&
      exe.rfind("robotbridge", 0) != 0)
    return TRUE;
  RECT cr{};
  GetClientRect(hwnd, &cr);
  if (cr.right < 800 || cr.bottom < 500) return TRUE;
  POINT tl{0, 0};
  ClientToScreen(hwnd, &tl);
  found->hwnd = hwnd;
  found->screen_rect = {tl.x, tl.y, tl.x + cr.right, tl.y + cr.bottom};
  found->title = title;
  return TRUE;  // keep last match, mirroring the python bridge
}

FoundWindow find_game_window()
{
  FoundWindow f;
  EnumWindows(enum_cb, reinterpret_cast<LPARAM>(&f));
  return f;
}

double qpc_to_seconds(LONGLONG qpc)
{
  static LARGE_INTEGER freq = [] {
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return f;
  }();
  return static_cast<double>(qpc) / static_cast<double>(freq.QuadPart);
}
}  // namespace

struct WinCapture::Impl
{
  CComPtr<ID3D11Device> device;
  CComPtr<ID3D11DeviceContext> context;
  CComPtr<IDXGIOutputDuplication> dupl;
  CComPtr<ID3D11Texture2D> staging;
  int desk_w = 0, desk_h = 0;
  int desk_x = 0, desk_y = 0;  // desktop coords of the duplicated output
  HWND hwnd = nullptr;

  bool init_duplication(HMONITOR target_monitor)
  {
    dupl.Release();
    staging.Release();
    context.Release();
    device.Release();

    D3D_FEATURE_LEVEL fl;
    if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
                                 D3D11_SDK_VERSION, &device, &fl, &context)))
      return false;

    CComPtr<IDXGIDevice> dxgi_device;
    if (FAILED(device->QueryInterface(&dxgi_device))) return false;
    CComPtr<IDXGIAdapter> adapter;
    if (FAILED(dxgi_device->GetAdapter(&adapter))) return false;

    for (UINT i = 0;; ++i) {
      CComPtr<IDXGIOutput> output;
      if (adapter->EnumOutputs(i, &output) == DXGI_ERROR_NOT_FOUND) break;
      DXGI_OUTPUT_DESC desc;
      output->GetDesc(&desc);
      if (target_monitor && desc.Monitor != target_monitor) continue;
      CComPtr<IDXGIOutput1> output1;
      if (FAILED(output->QueryInterface(&output1))) continue;
      if (FAILED(output1->DuplicateOutput(device, &dupl))) continue;
      desk_x = desc.DesktopCoordinates.left;
      desk_y = desc.DesktopCoordinates.top;
      desk_w = desc.DesktopCoordinates.right - desc.DesktopCoordinates.left;
      desk_h = desc.DesktopCoordinates.bottom - desc.DesktopCoordinates.top;

      D3D11_TEXTURE2D_DESC td{};
      td.Width = desk_w;
      td.Height = desk_h;
      td.MipLevels = 1;
      td.ArraySize = 1;
      td.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
      td.SampleDesc.Count = 1;
      td.Usage = D3D11_USAGE_STAGING;
      td.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
      if (FAILED(device->CreateTexture2D(&td, nullptr, &staging))) return false;
      return true;
    }
    return false;
  }
};

WinCapture::WinCapture(int window_bottom_crop) : impl_(new Impl), crop_bottom_(window_bottom_crop)
{
}

WinCapture::~WinCapture() { delete impl_; }

bool WinCapture::init()
{
  FoundWindow f = find_game_window();
  if (!f.hwnd) return false;
  impl_->hwnd = f.hwnd;
  title_ = f.title;
  HMONITOR mon = MonitorFromWindow(f.hwnd, MONITOR_DEFAULTTOPRIMARY);
  if (!impl_->init_duplication(mon)) return false;
  full_client_h_ = f.screen_rect.bottom - f.screen_rect.top;
  width_ = f.screen_rect.right - f.screen_rect.left;
  height_ = full_client_h_ - crop_bottom_;
  raise_window();
  return true;
}

void WinCapture::raise_window()
{
  if (!impl_->hwnd) return;
  ShowWindow(impl_->hwnd, SW_RESTORE);
  SetWindowPos(impl_->hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
  SetForegroundWindow(impl_->hwnd);
}

CapturedFrame WinCapture::grab(int timeout_ms)
{
  CapturedFrame out;
  if (!impl_->dupl) return out;

  DXGI_OUTDUPL_FRAME_INFO info{};
  CComPtr<IDXGIResource> resource;
  HRESULT hr = impl_->dupl->AcquireNextFrame(timeout_ms, &info, &resource);
  if (hr == DXGI_ERROR_WAIT_TIMEOUT) return out;
  if (FAILED(hr)) {
    // Device lost / access lost: caller should re-init().
    impl_->dupl.Release();
    return out;
  }

  CComPtr<ID3D11Texture2D> tex;
  resource->QueryInterface(&tex);
  impl_->context->CopyResource(impl_->staging, tex);
  impl_->dupl->ReleaseFrame();

  // Re-read the window rect every frame (window may move/resize).
  RECT cr{};
  GetClientRect(impl_->hwnd, &cr);
  POINT tl{0, 0};
  ClientToScreen(impl_->hwnd, &tl);
  const int x0 = tl.x - impl_->desk_x;
  const int y0 = tl.y - impl_->desk_y;
  const int w = cr.right;
  const int h = cr.bottom - crop_bottom_;
  if (w < 100 || h < 100 || x0 < 0 || y0 < 0 || x0 + w > impl_->desk_w ||
      y0 + h > impl_->desk_h)
    return out;  // window off-screen / minimised
  full_client_h_ = cr.bottom;
  width_ = w;
  height_ = h;

  D3D11_MAPPED_SUBRESOURCE mapped{};
  if (FAILED(impl_->context->Map(impl_->staging, 0, D3D11_MAP_READ, 0, &mapped))) return out;
  cv::Mat desktop(impl_->desk_h, impl_->desk_w, CV_8UC4, mapped.pData,
                  static_cast<size_t>(mapped.RowPitch));
  cv::Mat roi = desktop(cv::Rect(x0, y0, w, h));
  cv::cvtColor(roi, out.img, cv::COLOR_BGRA2BGR);  // deep copy out of mapped mem
  impl_->context->Unmap(impl_->staging, 0);

  // LastPresentTime==0 means the frame content is unchanged (mouse-only
  // update); treat as no new frame for the vision loop.
  if (info.LastPresentTime.QuadPart == 0) return out;
  out.t_present = qpc_to_seconds(info.LastPresentTime.QuadPart);
  out.ok = true;
  return out;
}

}  // namespace io::gestalt
