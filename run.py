import argparse
import hashlib
import logging
import multiprocessing
import os
import shutil
import tarfile
import zipfile
from typing import List, Optional

from buildbase import (
    Platform,
    add_path,
    add_webrtc_build_arguments,
    build_and_install_boost,
    build_webrtc,
    cd,
    cmake_path,
    cmd,
    cmdcap,
    enum_all_files,
    get_macos_osver,
    get_webrtc_info,
    get_webrtc_platform,
    get_windows_osver,
    install_cli11,
    install_cmake,
    install_cuda_windows,
    install_llvm,
    install_openh264,
    install_rootfs,
    install_sdl3,
    install_vpl,
    install_webrtc,
    mkdir_p,
    read_version_file,
    rm_rf,
)

logging.basicConfig(level=logging.DEBUG)


BASE_DIR = os.path.abspath(os.path.dirname(__file__))


def install_deps(
    platform: Platform,
    source_dir: str,
    build_dir: str,
    install_dir: str,
    debug: bool,
    local_webrtc_build_dir: Optional[str],
    local_webrtc_build_args: List[str],
):
    with cd(BASE_DIR):
        version = read_version_file("VERSION")

        # multistrap を使った sysroot の構築
        if platform.target.os == "jetson" or platform.target.os == "raspberry-pi-os":
            conf = os.path.join(BASE_DIR, "multistrap", f"{platform.target.package_name}.conf")
            # conf ファイルのハッシュ値をバージョンとする
            version_md5 = hashlib.md5(open(conf, "rb").read()).hexdigest()
            install_rootfs_args = {
                "version": version_md5,
                "version_file": os.path.join(install_dir, "rootfs.version"),
                "install_dir": install_dir,
                "conf": conf,
                "arch": "arm64",
            }
            install_rootfs(**install_rootfs_args)

        # WebRTC
        webrtc_platform = get_webrtc_platform(platform)

        if local_webrtc_build_dir is None:
            install_webrtc_args = {
                "version": version["WEBRTC_BUILD_VERSION"],
                "version_file": os.path.join(install_dir, "webrtc.version"),
                "source_dir": source_dir,
                "install_dir": install_dir,
                "platform": webrtc_platform,
            }

            install_webrtc(**install_webrtc_args)
        else:
            build_webrtc_args = {
                "platform": webrtc_platform,
                "local_webrtc_build_dir": local_webrtc_build_dir,
                "local_webrtc_build_args": local_webrtc_build_args,
                "debug": debug,
            }

            build_webrtc(**build_webrtc_args)

        webrtc_info = get_webrtc_info(webrtc_platform, local_webrtc_build_dir, install_dir, debug)
        webrtc_version = read_version_file(webrtc_info.version_file)
        webrtc_deps = read_version_file(webrtc_info.deps_file)

        # Windows は MSVC を使うので不要
        # macOS は Apple Clang を使うので不要
        if platform.target.os not in ("windows", "macos") and local_webrtc_build_dir is None:
            # LLVM
            tools_url = webrtc_version["WEBRTC_SRC_TOOLS_URL"]
            tools_commit = webrtc_version["WEBRTC_SRC_TOOLS_COMMIT"]
            libcxx_url = webrtc_version["WEBRTC_SRC_THIRD_PARTY_LIBCXX_SRC_URL"]
            libcxx_commit = webrtc_version["WEBRTC_SRC_THIRD_PARTY_LIBCXX_SRC_COMMIT"]
            buildtools_url = webrtc_version["WEBRTC_SRC_BUILDTOOLS_URL"]
            buildtools_commit = webrtc_version["WEBRTC_SRC_BUILDTOOLS_COMMIT"]
            install_llvm_args = {
                "version": f"{tools_url}.{tools_commit}."
                f"{libcxx_url}.{libcxx_commit}."
                f"{buildtools_url}.{buildtools_commit}",
                "version_file": os.path.join(install_dir, "llvm.version"),
                "install_dir": install_dir,
                "tools_url": tools_url,
                "tools_commit": tools_commit,
                "libcxx_url": libcxx_url,
                "libcxx_commit": libcxx_commit,
                "buildtools_url": buildtools_url,
                "buildtools_commit": buildtools_commit,
            }
            install_llvm(**install_llvm_args)

        # Boost
        install_boost_args = {
            "version": version["BOOST_VERSION"],
            "version_file": os.path.join(install_dir, "boost.version"),
            "source_dir": source_dir,
            "build_dir": build_dir,
            "install_dir": install_dir,
            "cxx": "",
            "cflags": [],
            "cxxflags": [],
            "linkflags": [],
            "toolset": "",
            "visibility": "global",
            "target_os": "",
            "debug": debug,
            "android_ndk": "",
            "native_api_level": "",
            "architecture": "x86",
            "address_model": "64",
        }
        if platform.target.os == "windows":
            install_boost_args["cxxflags"] = ["-D_ITERATOR_DEBUG_LEVEL=0"]
            install_boost_args["toolset"] = "msvc"
            install_boost_args["target_os"] = "windows"
        elif platform.target.os == "macos":
            sysroot = cmdcap(["xcrun", "--sdk", "macosx", "--show-sdk-path"])
            install_boost_args["target_os"] = "darwin"
            install_boost_args["toolset"] = "clang"
            install_boost_args["cxx"] = "clang++"
            install_boost_args["cflags"] = [
                f"--sysroot={sysroot}",
                f"-mmacosx-version-min={webrtc_deps['MACOS_DEPLOYMENT_TARGET']}",
            ]
            install_boost_args["cxxflags"] = [
                "-fPIC",
                f"--sysroot={sysroot}",
                "-std=gnu++17",
                f"-mmacosx-version-min={webrtc_deps['MACOS_DEPLOYMENT_TARGET']}",
            ]
            install_boost_args["visibility"] = "hidden"
            if platform.target.arch == "x86_64":
                install_boost_args["cflags"].extend(["-target", "x86_64-apple-darwin"])
                install_boost_args["cxxflags"].extend(["-target", "x86_64-apple-darwin"])
                install_boost_args["architecture"] = "x86"
            if platform.target.arch == "arm64":
                install_boost_args["cflags"].extend(["-target", "aarch64-apple-darwin"])
                install_boost_args["cxxflags"].extend(["-target", "aarch64-apple-darwin"])
                install_boost_args["architecture"] = "arm"
        elif platform.target.os in ("jetson", "raspberry-pi-os"):
            triplet = "aarch64-linux-gnu"
            sysroot = os.path.join(install_dir, "rootfs")
            install_boost_args["target_os"] = "linux"
            install_boost_args["cxx"] = os.path.join(webrtc_info.clang_dir, "bin", "clang++")
            install_boost_args["cflags"] = [
                "-fPIC",
                f"--sysroot={sysroot}",
                f"--target={triplet}",
                f"-I{os.path.join(sysroot, 'usr', 'include', triplet)}",
            ]
            install_boost_args["cxxflags"] = [
                "-fPIC",
                f"--target={triplet}",
                f"--sysroot={sysroot}",
                f"-I{os.path.join(sysroot, 'usr', 'include', triplet)}",
                "-D_LIBCPP_ABI_NAMESPACE=Cr",
                "-D_LIBCPP_ABI_VERSION=2",
                "-D_LIBCPP_DISABLE_AVAILABILITY",
                "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE",
                "-nostdinc++",
                "-std=gnu++17",
                f"-isystem{os.path.join(webrtc_info.libcxx_dir, 'include')}",
            ]
            install_boost_args["linkflags"] = [
                f"-L{os.path.join(sysroot, 'usr', 'lib', triplet)}",
                f"-B{os.path.join(sysroot, 'usr', 'lib', triplet)}",
            ]
            install_boost_args["toolset"] = "clang"
            install_boost_args["architecture"] = "arm"
        else:
            install_boost_args["target_os"] = "linux"
            install_boost_args["cxx"] = os.path.join(webrtc_info.clang_dir, "bin", "clang++")
            install_boost_args["cxxflags"] = [
                "-D_LIBCPP_ABI_NAMESPACE=Cr",
                "-D_LIBCPP_ABI_VERSION=2",
                "-D_LIBCPP_DISABLE_AVAILABILITY",
                "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE",
                "-nostdinc++",
                f"-isystem{os.path.join(webrtc_info.libcxx_dir, 'include')}",
                "-fPIC",
            ]
            install_boost_args["toolset"] = "clang"

        build_and_install_boost(**install_boost_args)

        # CMake
        install_cmake_args = {
            "version": version["CMAKE_VERSION"],
            "version_file": os.path.join(install_dir, "cmake.version"),
            "source_dir": source_dir,
            "install_dir": install_dir,
            "platform": "",
            "ext": "tar.gz",
        }
        if platform.build.os == "windows" and platform.build.arch == "x86_64":
            install_cmake_args["platform"] = "windows-x86_64"
            install_cmake_args["ext"] = "zip"
        elif platform.build.os == "macos":
            install_cmake_args["platform"] = "macos-universal"
        elif platform.build.os == "ubuntu" and platform.build.arch == "x86_64":
            install_cmake_args["platform"] = "linux-x86_64"
        elif platform.build.os == "ubuntu" and platform.build.arch == "arm64":
            install_cmake_args["platform"] = "linux-aarch64"
        else:
            raise Exception("Failed to install CMake")
        install_cmake(**install_cmake_args)

        if platform.build.os == "macos":
            add_path(os.path.join(install_dir, "cmake", "CMake.app", "Contents", "bin"))
        else:
            add_path(os.path.join(install_dir, "cmake", "bin"))

        # CUDA
        if platform.target.os == "windows":
            install_cuda_args = {
                "version": version["CUDA_VERSION"],
                "version_file": os.path.join(install_dir, "cuda.version"),
                "source_dir": source_dir,
                "build_dir": build_dir,
                "install_dir": install_dir,
            }
            install_cuda_windows(**install_cuda_args)

        # Intel oneVPL
        if platform.target.os in ("windows", "ubuntu") and platform.target.arch == "x86_64":
            install_vpl_args = {
                "version": version["VPL_VERSION"],
                "version_file": os.path.join(install_dir, "vpl.version"),
                "configuration": "Debug" if debug else "Release",
                "source_dir": source_dir,
                "build_dir": build_dir,
                "install_dir": install_dir,
                "cmake_args": [],
            }
            if platform.target.os == "windows":
                cxxflags = [
                    "/DWIN32",
                    "/D_WINDOWS",
                    "/W3",
                    "/GR",
                    "/EHsc",
                    "/D_ITERATOR_DEBUG_LEVEL=0",
                ]
                install_vpl_args["cmake_args"].append(f"-DCMAKE_CXX_FLAGS={' '.join(cxxflags)}")
            if platform.target.os == "ubuntu":
                cmake_args = []
                cmake_args.append("-DCMAKE_C_COMPILER=clang-18")
                cmake_args.append("-DCMAKE_CXX_COMPILER=clang++-18")
                path = cmake_path(os.path.join(webrtc_info.libcxx_dir, "include"))
                cmake_args.append(f"-DCMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES={path}")
                flags = [
                    "-nostdinc++",
                    "-D_LIBCPP_ABI_NAMESPACE=Cr",
                    "-D_LIBCPP_ABI_VERSION=2",
                    "-D_LIBCPP_DISABLE_AVAILABILITY",
                    "-D_LIBCPP_DISABLE_VISIBILITY_ANNOTATIONS",
                    "-D_LIBCXXABI_DISABLE_VISIBILITY_ANNOTATIONS",
                    "-D_LIBCPP_ENABLE_NODISCARD",
                    "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE",
                ]
                cmake_args.append(f"-DCMAKE_CXX_FLAGS={' '.join(flags)}")
                install_vpl_args["cmake_args"] += cmake_args
            install_vpl(**install_vpl_args)

        # SDL3
        install_sdl3_args = {
            "version": version["SDL3_VERSION"],
            "version_file": os.path.join(install_dir, "sdl3.version"),
            "source_dir": source_dir,
            "build_dir": build_dir,
            "install_dir": install_dir,
            "platform": "",
            "debug": debug,
            "cmake_args": [],
        }
        if platform.target.os == "windows":
            install_sdl3_args["platform"] = "windows"
        elif platform.target.os == "macos":
            install_sdl3_args["platform"] = "macos"
        elif platform.target.os == "ubuntu":
            install_sdl3_args["platform"] = "linux"
        elif platform.target.os in ("jetson", "raspberry-pi-os"):
            install_sdl3_args["platform"] = "linux-cross"
            triplet = "aarch64-linux-gnu"
            arch = "aarch64"
            sysroot = os.path.join(install_dir, "rootfs")
            install_sdl3_args["cmake_args"] = [
                "-DCMAKE_SYSTEM_NAME=Linux",
                f"-DCMAKE_SYSTEM_PROCESSOR={arch}",
                f"-DCMAKE_C_COMPILER={os.path.join(webrtc_info.clang_dir, 'bin', 'clang')}",
                f"-DCMAKE_C_COMPILER_TARGET={triplet}",
                f"-DCMAKE_CXX_COMPILER={os.path.join(webrtc_info.clang_dir, 'bin', 'clang++')}",
                f"-DCMAKE_CXX_COMPILER_TARGET={triplet}",
                f"-DCMAKE_FIND_ROOT_PATH={sysroot}",
                "-DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER",
                "-DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH",
                "-DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH",
                "-DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH",
                f"-DCMAKE_SYSROOT={sysroot}",
            ]
        else:
            raise Exception("Not supported platform")

        install_sdl3(**install_sdl3_args)

        # CLI11
        install_cli11_args = {
            "version": version["CLI11_VERSION"],
            "version_file": os.path.join(install_dir, "cli11.version"),
            "install_dir": install_dir,
        }
        install_cli11(**install_cli11_args)

        # OpenH264
        install_openh264_args = {
            "version": version["OPENH264_VERSION"],
            "version_file": os.path.join(install_dir, "openh264.version"),
            "source_dir": source_dir,
            "install_dir": install_dir,
            "is_windows": platform.target.os == 'windows',
        }
        install_openh264(**install_openh264_args)


AVAILABLE_TARGETS = [
    "windows_x86_64",
    "macos_x86_64",
    "macos_arm64",
    "ubuntu-22.04_x86_64",
    "ubuntu-24.04_x86_64",
    "raspberry-pi-os_armv8",
    "ubuntu-22.04_armv8_jetson",
]
WINDOWS_SDK_VERSION = "10.0.20348.0"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("target", choices=AVAILABLE_TARGETS)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--relwithdebinfo", action="store_true")
    add_webrtc_build_arguments(parser)
    parser.add_argument("--package", action="store_true")

    args = parser.parse_args()
    if args.target == "windows_x86_64":
        platform = Platform("windows", get_windows_osver(), "x86_64")
    elif args.target == "macos_x86_64":
        platform = Platform("macos", get_macos_osver(), "x86_64")
    elif args.target == "macos_arm64":
        platform = Platform("macos", get_macos_osver(), "arm64")
    elif args.target == "ubuntu-22.04_x86_64":
        platform = Platform("ubuntu", "22.04", "x86_64")
    elif args.target == "ubuntu-24.04_x86_64":
        platform = Platform("ubuntu", "24.04", "x86_64")
    elif args.target == "raspberry-pi-os_armv8":
        platform = Platform("raspberry-pi-os", None, "armv8")
    elif args.target == "ubuntu-22.04_armv8_jetson":
        platform = Platform("jetson", None, "armv8", extra="ubuntu-22.04")
    else:
        raise Exception(f"Unknown target {args.target}")

    logging.info(f"Build platform: {platform.build.package_name}")
    logging.info(f"Target platform: {platform.target.package_name}")

    configuration = "debug" if args.debug else "release"
    dir = platform.target.package_name
    source_dir = os.path.join(BASE_DIR, "_source", dir, configuration)
    build_dir = os.path.join(BASE_DIR, "_build", dir, configuration)
    install_dir = os.path.join(BASE_DIR, "_install", dir, configuration)
    package_dir = os.path.join(BASE_DIR, "_package", dir, configuration)
    mkdir_p(source_dir)
    mkdir_p(build_dir)
    mkdir_p(install_dir)

    install_deps(
        platform,
        source_dir,
        build_dir,
        install_dir,
        args.debug,
        local_webrtc_build_dir=args.local_webrtc_build_dir,
        local_webrtc_build_args=args.local_webrtc_build_args,
    )

    configuration = "Release"
    if args.debug:
        configuration = "Debug"
    if args.relwithdebinfo:
        configuration = "RelWithDebInfo"

    momo_build_dir = os.path.join(build_dir, "momo")
    mkdir_p(momo_build_dir)
    with cd(momo_build_dir):
        cmake_args = []
        cmake_args.append(f"-DCMAKE_BUILD_TYPE={configuration}")
        cmake_args.append(f"-DCMAKE_INSTALL_PREFIX={cmake_path(os.path.join(install_dir, 'momo'))}")
        cmake_args.append(f"-DBOOST_ROOT={cmake_path(os.path.join(install_dir, 'boost'))}")
        webrtc_platform = get_webrtc_platform(platform)
        webrtc_info = get_webrtc_info(
            webrtc_platform, args.local_webrtc_build_dir, install_dir, args.debug
        )
        webrtc_version = read_version_file(webrtc_info.version_file)
        webrtc_deps = read_version_file(webrtc_info.deps_file)
        with cd(BASE_DIR):
            version = read_version_file("VERSION")
            momo_version = version["MOMO_VERSION"]
            momo_commit = cmdcap(["git", "rev-parse", "HEAD"])
        cmake_args.append(f"-DWEBRTC_INCLUDE_DIR={cmake_path(webrtc_info.webrtc_include_dir)}")
        cmake_args.append(f"-DWEBRTC_LIBRARY_DIR={cmake_path(webrtc_info.webrtc_library_dir)}")
        cmake_args.append(f"-DMOMO_VERSION={momo_version}")
        cmake_args.append(f"-DMOMO_COMMIT={momo_commit}")
        cmake_args.append(f"-DMOMO_TARGET={platform.target.package_name}")
        cmake_args.append(f"-DWEBRTC_BUILD_VERSION={webrtc_version['WEBRTC_BUILD_VERSION']}")
        cmake_args.append(f"-DWEBRTC_READABLE_VERSION={webrtc_version['WEBRTC_READABLE_VERSION']}")
        cmake_args.append(f"-DWEBRTC_COMMIT={webrtc_version['WEBRTC_COMMIT']}")
        if platform.target.os == "windows":
            cmake_args.append(f"-DCMAKE_SYSTEM_VERSION={WINDOWS_SDK_VERSION}")
        if platform.target.os == "ubuntu":
            if platform.target.package_name in ("ubuntu-22.04_x86_64", "ubuntu-24.04_x86_64"):
                cmake_args.append("-DCMAKE_C_COMPILER=clang-18")
                cmake_args.append("-DCMAKE_CXX_COMPILER=clang++-18")
            else:
                cmake_args.append(
                    f"-DCMAKE_C_COMPILER={cmake_path(os.path.join(webrtc_info.clang_dir, 'bin', 'clang'))}"
                )
                cmake_args.append(
                    f"-DCMAKE_CXX_COMPILER={cmake_path(os.path.join(webrtc_info.clang_dir, 'bin', 'clang++'))}"
                )
            cmake_args.append("-DUSE_LIBCXX=ON")
            cmake_args.append(
                f"-DLIBCXX_INCLUDE_DIR={cmake_path(os.path.join(webrtc_info.libcxx_dir, 'include'))}"
            )
        if platform.target.os == "macos":
            sysroot = cmdcap(["xcrun", "--sdk", "macosx", "--show-sdk-path"])
            target = (
                "x86_64-apple-darwin"
                if platform.target.arch == "x86_64"
                else "aarch64-apple-darwin"
            )
            cmake_args.append(f"-DCMAKE_SYSTEM_PROCESSOR={platform.target.arch}")
            cmake_args.append(f"-DCMAKE_OSX_ARCHITECTURES={platform.target.arch}")
            cmake_args.append(
                f"-DCMAKE_OSX_DEPLOYMENT_TARGET={webrtc_deps['MACOS_DEPLOYMENT_TARGET']}"
            )
            cmake_args.append(f"-DCMAKE_C_COMPILER_TARGET={target}")
            cmake_args.append(f"-DCMAKE_CXX_COMPILER_TARGET={target}")
            cmake_args.append(f"-DCMAKE_OBJCXX_COMPILER_TARGET={target}")
            cmake_args.append(f"-DCMAKE_SYSROOT={sysroot}")
        if platform.target.os in ("jetson", "raspberry-pi-os"):
            triplet = "aarch64-linux-gnu"
            arch = "aarch64"
            sysroot = os.path.join(install_dir, "rootfs")
            cmake_args.append("-DCMAKE_SYSTEM_NAME=Linux")
            cmake_args.append(f"-DCMAKE_SYSTEM_PROCESSOR={arch}")
            cmake_args.append(f"-DCMAKE_SYSROOT={sysroot}")
            cmake_args.append(f"-DCMAKE_C_COMPILER_TARGET={triplet}")
            cmake_args.append(f"-DCMAKE_CXX_COMPILER_TARGET={triplet}")
            cmake_args.append(f"-DCMAKE_FIND_ROOT_PATH={sysroot}")
            cmake_args.append("-DUSE_LIBCXX=ON")
            cmake_args.append(
                f"-DLIBCXX_INCLUDE_DIR={cmake_path(os.path.join(webrtc_info.libcxx_dir, 'include'))}"
            )
            cmake_args.append(
                f"-DCMAKE_C_COMPILER={cmake_path(os.path.join(webrtc_info.clang_dir, 'bin', 'clang'))}"
            )
            cmake_args.append(
                f"-DCMAKE_CXX_COMPILER={cmake_path(os.path.join(webrtc_info.clang_dir, 'bin', 'clang++'))}"
            )
            if platform.target.os == "jetson":
                cmake_args.append("-DUSE_JETSON_ENCODER=ON")
            if platform.target.os == "raspberry-pi-os":
                cmake_args.append("-DUSE_V4L2_ENCODER=ON")

        # スクリーンキャプチャ
        if platform.target.package_name in (
            "windows_x86_64",
            "macos_x86_64",
            "macos_arm64",
            "ubuntu-22.04_x86_64",
            "ubuntu-24.04_x86_64",
        ):
            cmake_args.append("-DUSE_SCREEN_CAPTURER=ON")

        # NvCodec
        if platform.target.os in ("windows", "ubuntu") and platform.target.arch == "x86_64":
            cmake_args.append("-DUSE_NVCODEC_ENCODER=ON")
            if platform.target.os == "windows":
                cmake_args.append(
                    f"-DCUDA_TOOLKIT_ROOT_DIR={cmake_path(os.path.join(install_dir, 'cuda'))}"
                )

        if platform.target.os in ("windows", "ubuntu") and platform.target.arch == "x86_64":
            cmake_args.append("-DUSE_VPL_ENCODER=ON")
            cmake_args.append(f"-DVPL_ROOT_DIR={cmake_path(os.path.join(install_dir, 'vpl'))}")

        cmake_args.append(f"-DSDL3_ROOT_DIR={os.path.join(install_dir, 'sdl3')}")
        cmake_args.append(f"-DCLI11_ROOT_DIR={os.path.join(install_dir, 'cli11')}")
        cmake_args.append(f"-DOPENH264_ROOT_DIR={os.path.join(install_dir, 'openh264')}")

        cmd(["cmake", BASE_DIR] + cmake_args)
        cmd(
            [
                "cmake",
                "--build",
                ".",
                f"-j{multiprocessing.cpu_count()}",
                "--config",
                configuration,
            ]
        )
        cmd(["cmake", "--install", ".", "--config", configuration])

    if args.package:
        # 必要なファイルをコピー
        with cd(BASE_DIR):
            momo_install_dir = os.path.join(install_dir, "momo")
            momo_build_dir = os.path.join(build_dir, "momo")
            rm_rf(momo_install_dir)
            mkdir_p(momo_install_dir)
            if platform.target.os == "windows":
                shutil.copyfile(
                    os.path.join(momo_build_dir, "Release", "momo.exe"),
                    os.path.join(momo_install_dir, "momo.exe"),
                )
            else:
                shutil.copyfile(
                    os.path.join(momo_build_dir, "momo"), os.path.join(momo_install_dir, "momo")
                )
            shutil.copyfile("LICENSE", os.path.join(momo_install_dir, "LICENSE"))
            shutil.copyfile("NOTICE", os.path.join(momo_install_dir, "NOTICE"))
            shutil.copytree("html", os.path.join(momo_install_dir, "html"))
            if os.path.exists(os.path.join(momo_build_dir, "libcamerac.so")):
                shutil.copyfile(
                    os.path.join(momo_build_dir, "libcamerac.so"),
                    os.path.join(momo_install_dir, "libcamerac.so"),
                )

        mkdir_p(package_dir)
        rm_rf(os.path.join(package_dir, "momo"))
        rm_rf(os.path.join(package_dir, "momo.env"))

        with cd(BASE_DIR):
            version = read_version_file("VERSION")
            momo_version = version["MOMO_VERSION"]

        def archive(archive_path, files, is_windows):
            if is_windows:
                with zipfile.ZipFile(archive_path, "w") as f:
                    for file in files:
                        f.write(filename=file, arcname=file)
            else:
                with tarfile.open(archive_path, "w:gz") as f:
                    for file in files:
                        f.add(name=file, arcname=file)

        ext = "zip" if platform.target.os == "windows" else "tar.gz"
        is_windows = platform.target.os == "windows"
        content_type = "application/zip" if platform.target.os == "windows" else "application/gzip"

        with cd(install_dir):
            archive_name = f"momo-{momo_version}_{platform.target.package_name}.{ext}"
            archive_path = os.path.join(package_dir, archive_name)
            archive(archive_path, enum_all_files("momo", "."), is_windows)

            with open(os.path.join(package_dir, "momo.env"), "w") as f:
                f.write(f"CONTENT_TYPE={content_type}\n")
                f.write(f"PACKAGE_NAME={archive_name}\n")


if __name__ == "__main__":
    main()
