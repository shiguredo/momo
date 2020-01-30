$ErrorActionPreference = 'Stop'

Push-Location windows
  .\install_deps.ps1
Pop-Location

$WEBRTC_VERSION_FILE = Join-Path (Resolve-Path ".").Path "windows\_install\webrtc\VERSIONS"
Get-Content $WEBRTC_VERSION_FILE | Foreach-Object {
  if (!$_) {
    continue
  }
  $var = $_.Split('=')
  New-Variable -Name $var[0] -Value $var[1] -Force
}
$MOMO_VERSION_FILE = Join-Path (Resolve-Path ".").Path "..\VERSION"
Get-Content $MOMO_VERSION_FILE | Foreach-Object {
  if (!$_) {
    continue
  }
  $var = $_.Split('=')
  New-Variable -Name $var[0] -Value $var[1] -Force
}

$MOMO_COMMIT = "$(git rev-parse HEAD)"

mkdir ..\_build\windows -Force -ErrorAction Ignore
Push-Location ..\_build\windows
  cmake ..\.. -G "Visual Studio 16 2019" `
    -DMOMO_PACKAGE_NAME="windows" `
    -DMOMO_VERSION="$MOMO_VERSION" `
    -DMOMO_COMMIT="$MOMO_COMMIT" `
    -DWEBRTC_BUILD_VERSION="$WEBRTC_BUILD_VERSION" `
    -DWEBRTC_READABLE_VERSION="$WEBRTC_READABLE_VERSION" `
    -DWEBRTC_COMMIT="$WEBRTC_COMMIT"
  cmake --build . --config Release
Pop-Location