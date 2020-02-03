Param([switch]$clean, [switch]$package)

$ErrorActionPreference = 'Stop'

if ($clean) {
  if (Test-Path "windows\_source") {
    Remove-Item "windows\_source" -Force -Recurse
  }
  if (Test-Path "windows\_build") {
    Remove-Item "windows\_build" -Force -Recurse
  }
  if (Test-Path "windows\_install") {
    Remove-Item "windows\_install" -Force -Recurse
  }
  if (Test-Path "..\_build\windows") {
    Remove-Item "..\_build\windows" -Force -Recurse
  }
  exit 0
}

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

if ($package) {
  # パッケージのバイナリを作る
  Push-Location ..
    $WINVER_MAJOR = [System.Environment]::OSVersion.Version.Major
    $RELEASE_ID = (Get-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion" -Name ReleaseId).ReleaseID
    if (Test-Path "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}.zip") {
      Remove-Item "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}.zip" -Force
    }
    if (Test-Path "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}") {
      Remove-Item "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}" -Force -Recurse
    }
    mkdir -Force "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}"
    Copy-Item _build\windows\Release\momo.exe _package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}\
    Copy-Item LICENSE                         _package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}\
    Copy-Item NOTICE                          _package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}\
    Copy-Item html                            _package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}\html\ -Recurse
    Push-Location _package
      Compress-Archive -Path "momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}" -DestinationPath "momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}.zip"
    Pop-Location
    Remove-Item "_package\momo-${MOMO_VERSION}_windows-${WINVER_MAJOR}.${RELEASE_ID}" -Force -Recurse
  Pop-Location
}