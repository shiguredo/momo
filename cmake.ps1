$ErrorActionPreference = 'Stop'

$WEBRTC_VERSION_FILE = Join-Path (Resolve-Path ".").Path "build\windows\_install\webrtc\VERSIONS"
Get-Content $WEBRTC_VERSION_FILE | Foreach-Object{
  if (!$_) {
    continue
  }
  $var = $_.Split('=')
  New-Variable -Name $var[0] -Value $var[1] -Force
}
$MOMO_VERSION_FILE = Join-Path (Resolve-Path ".").Path "VERSION"
Get-Content $MOMO_VERSION_FILE | Foreach-Object{
  if (!$_) {
    continue
  }
  $var = $_.Split('=')
  New-Variable -Name $var[0] -Value $var[1] -Force
}

$MOMO_COMMIT = "$(git rev-parse HEAD)"

$INSTALL_DIR = (Join-Path (Resolve-Path ".").Path "build\windows\_install").Replace("\", "/")
$MODULE_PATH = (Join-Path (Resolve-Path ".").Path "cmake").Replace("\", "/")
$SDL_MODULE_PATH = (Join-Path (Resolve-Path ".").Path "build\windows\_install\SDL2\cmake").Replace("\", "/")

mkdir _build\windows -Force -ErrorAction Ignore
Push-Location _build\windows
  cmake ..\.. -G "Visual Studio 16 2019" `
    -DMOMO_VERSION="$MOMO_VERSION" `
    -DMOMO_COMMIT="$MOMO_COMMIT" `
    -DWEBRTC_BUILD_VERSION="$WEBRTC_BUILD_VERSION" `
    -DWEBRTC_READABLE_VERSION="$WEBRTC_READABLE_VERSION" `
    -DWEBRTC_COMMIT="$WEBRTC_COMMIT" `
    -DWEBRTC_ROOT_DIR="$INSTALL_DIR/webrtc" `
    -DNVCODEC_ROOT_DIR="$INSTALL_DIR/Video_Codec_SDK" `
    -DJSON_ROOT_DIR="$INSTALL_DIR/json" `
    -DCLI11_ROOT_DIR="$INSTALL_DIR/CLI11" `
    -DCMAKE_MODULE_PATH="$MODULE_PATH;$SDL_MODULE_PATH" `
    -DCMAKE_PREFIX_PATH="$INSTALL_DIR/boost;$INSTALL_DIR/SDL2"
  cmake --build . --config Release
Pop-Location