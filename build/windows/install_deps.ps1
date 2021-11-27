$ErrorActionPreference = 'Stop'

$SOURCE_DIR = Join-Path (Resolve-Path ".").Path "_source"
$BUILD_DIR = Join-Path (Resolve-Path ".").Path "_build"
$INSTALL_DIR = Join-Path (Resolve-Path ".").Path "_install"
$INSTALL_DIR_SLASH = $INSTALL_DIR.Replace("\", "/")
$CACHE_DIR = [IO.Path]::Combine((Resolve-Path ".").Path, "..", "..", "_cache")

$VERSION_FILE = Join-Path (Resolve-Path ".").Path "..\..\VERSION"
Get-Content $VERSION_FILE | Foreach-Object{
  if (!$_) {
    continue
  }
  $var = $_.Split('=')
  New-Variable -Name $var[0] -Value $var[1]
}

mkdir $SOURCE_DIR -ErrorAction Ignore
mkdir $BUILD_DIR -ErrorAction Ignore
mkdir $INSTALL_DIR -ErrorAction Ignore
mkdir $CACHE_DIR -ErrorAction Ignore

# WebRTC の取得

if (!(Test-Path "$INSTALL_DIR\webrtc\lib\webrtc.lib")) {
  # shiguredo-webrtc-windows のバイナリをダウンロードする
  $_URL = "https://github.com/shiguredo-webrtc-build/webrtc-build/releases/download/m$WEBRTC_BUILD_VERSION/webrtc.windows.zip"
  $_FILE = "$SOURCE_DIR\webrtc-m$WEBRTC_BUILD_VERSION.zip"
  Push-Location $SOURCE_DIR
    if (!(Test-Path $_FILE)) {
      Invoke-WebRequest -Uri $_URL -OutFile $_FILE
    }
  Pop-Location
  # 展開
  Remove-Item $SOURCE_DIR\webrtc -Recurse -Force -ErrorAction Ignore
  # Expand-Archive -Path $_FILE -DestinationPath "$SOURCE_DIR\webrtc"
  Push-Location $SOURCE_DIR
    7z x $_FILE
  Pop-Location

  # インストール
  Remove-Item $INSTALL_DIR\webrtc -Recurse -Force -ErrorAction Ignore
  Move-Item $SOURCE_DIR\webrtc $INSTALL_DIR\webrtc
}

# Boost のビルド

if (!(Test-Path "$INSTALL_DIR\boost\include\boost\version.hpp")) {
  $_BOOST_UNDERSCORE_VERSION = $BOOST_VERSION.Replace(".", "_")
  $_URL = "https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${_BOOST_UNDERSCORE_VERSION}.zip"
  $_FILE = "$CACHE_DIR\boost\boost_${_BOOST_UNDERSCORE_VERSION}.zip"
  mkdir "$CACHE_DIR\boost" -ErrorAction Ignore
  # ダウンロードと展開
  Push-Location $SOURCE_DIR
    if (!(Test-Path $_FILE)) {
      # curl に擬態しないとアーカイブではなく HTML コンテンツが降ってきてしまう
      Invoke-WebRequest -Headers @{"User-Agent"="curl/7.55.1"} -Uri $_URL -OutFile $_FILE
    }
    Remove-Item boost -Force -Recurse -ErrorAction Ignore
    Remove-Item boost_${_BOOST_UNDERSCORE_VERSION} -Force -Recurse -ErrorAction Ignore
    # Expand-Archive -Path $_FILE -DestinationPath .
    7z x $_FILE
    Move-Item boost_${_BOOST_UNDERSCORE_VERSION} boost
  Pop-Location
  # ビルドとインストール
  Push-Location $SOURCE_DIR\boost
    Remove-Item $BUILD_DIR\boost -Force -Recurse -ErrorAction Ignore
    .\bootstrap.bat
    .\b2.exe install `
      -j8 `
      --prefix=$INSTALL_DIR\boost `
      --build-dir=$BUILD_DIR\boost `
      --with-filesystem `
      --with-json `
      --layout=system `
      address-model=64 `
      link=static `
      threading=multi `
      variant=release `
      runtime-link=static
  Pop-Location
}

# SDL2 のビルド

if (!(Test-Path "$INSTALL_DIR\SDL2\include\SDL2\SDL.h")) {
  $_URL = "http://www.libsdl.org/release/SDL2-$SDL2_VERSION.zip"
  $_FILE = "SDL2.zip"
  # ダウンロードと展開
  Push-Location $SOURCE_DIR
    if (!(Test-Path $_FILE)) {
      Invoke-WebRequest -Uri $_URL -OutFile $_FILE
    }
    Remove-Item SDL2 -Force -Recurse -ErrorAction Ignore
    Remove-Item SDL2-$SDL2_VERSION -Force -Recurse -ErrorAction Ignore
    # Expand-Archive -Path $_FILE -DestinationPath .
    7z x $_FILE
    Move-Item SDL2-$SDL2_VERSION SDL2
  Pop-Location

  mkdir $BUILD_DIR\SDL2 -ErrorAction Ignore
  Push-Location $BUILD_DIR\SDL2
    cmake `
      -G "Visual Studio 16 2019" `
      -DFORCE_STATIC_VCRT=ON `
      -DBUILD_SHARED_LIBS=OFF `
      -DHAVE_LIBC=ON `
      "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR_SLASH}/SDL2" `
      $SOURCE_DIR\SDL2

    cmake --build . --config Release
    cmake --build . --config Release --target INSTALL
  Pop-Location
}


# CLI11 の取得

if (!(Test-Path "$INSTALL_DIR\CLI11\include\CLI\CLI.hpp")) {
  $_URL = "https://github.com/CLIUtils/CLI11/archive/v${CLI11_VERSION}.zip"
  $_FILE = "$SOURCE_DIR\CLI11.zip"
  # ダウンロード
  Push-Location $SOURCE_DIR
    if (!(Test-Path $_FILE)) {
      Invoke-WebRequest -Uri $_URL -OutFile $_FILE
    }
  Pop-Location
  # 展開(=インストール)
  Remove-Item $INSTALL_DIR\CLI11 -Recurse -Force -ErrorAction Ignore
  Remove-Item $INSTALL_DIR\CLI11-${CLI11_VERSION} -Recurse -Force -ErrorAction Ignore
  # Expand-Archive -Path $_FILE -DestinationPath "$INSTALL_DIR"
  Push-Location $INSTALL_DIR
    7z x $_FILE
    Move-Item CLI11-${CLI11_VERSION} CLI11
  Pop-Location
}
