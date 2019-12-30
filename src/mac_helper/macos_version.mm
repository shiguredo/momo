#include "macos_version.h"

#import <Foundation/Foundation.h>
#include <TargetConditionals.h>

// TARGET_OS_* から OS 名を調べる。
// アーキテクチャもマクロから分かるけど、それは実行時に uname を使って調べるので不要

// 以下の情報は /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk/usr/include/TargetConditionals.h からのコピペ

/****************************************************************************************************

    TARGET_CPU_*    
    These conditionals specify which microprocessor instruction set is being
    generated.  At most one of these is true, the rest are false.

        TARGET_CPU_PPC          - Compiler is generating PowerPC instructions for 32-bit mode
        TARGET_CPU_PPC64        - Compiler is generating PowerPC instructions for 64-bit mode
        TARGET_CPU_68K          - Compiler is generating 680x0 instructions
        TARGET_CPU_X86          - Compiler is generating x86 instructions for 32-bit mode
        TARGET_CPU_X86_64       - Compiler is generating x86 instructions for 64-bit mode
        TARGET_CPU_ARM          - Compiler is generating ARM instructions for 32-bit mode
        TARGET_CPU_ARM64        - Compiler is generating ARM instructions for 64-bit mode
        TARGET_CPU_MIPS         - Compiler is generating MIPS instructions
        TARGET_CPU_SPARC        - Compiler is generating Sparc instructions
        TARGET_CPU_ALPHA        - Compiler is generating Dec Alpha instructions


    TARGET_OS_* 
    These conditionals specify in which Operating System the generated code will
    run.  Indention is used to show which conditionals are evolutionary subclasses.  
    
    The MAC/WIN32/UNIX conditionals are mutually exclusive.
    The IOS/TV/WATCH conditionals are mutually exclusive.
    
    
        TARGET_OS_WIN32           - Generated code will run under 32-bit Windows
        TARGET_OS_UNIX            - Generated code will run under some Unix (not OSX) 
        TARGET_OS_MAC             - Generated code will run under Mac OS X variant
           TARGET_OS_OSX          - Generated code will run under OS X devices
           TARGET_OS_IPHONE          - Generated code for firmware, devices, or simulator
              TARGET_OS_IOS             - Generated code will run under iOS 
              TARGET_OS_TV              - Generated code will run under Apple TV OS
              TARGET_OS_WATCH           - Generated code will run under Apple Watch OS
              TARGET_OS_BRIDGE          - Generated code will run under Bridge devices
              TARGET_OS_MACCATALYST     - Generated code will run under macOS
           TARGET_OS_SIMULATOR      - Generated code will run under a simulator
       
        TARGET_OS_EMBEDDED        - DEPRECATED: Use TARGET_OS_IPHONE and/or TARGET_OS_SIMULATOR instead
        TARGET_IPHONE_SIMULATOR   - DEPRECATED: Same as TARGET_OS_SIMULATOR
        TARGET_OS_NANO            - DEPRECATED: Same as TARGET_OS_WATCH

      +----------------------------------------------------------------+
      |                TARGET_OS_MAC                                   |
      | +---+  +-----------------------------------------------------+ |
      | |   |  |          TARGET_OS_IPHONE                           | |
      | |OSX|  | +-----+ +----+ +-------+ +--------+ +-------------+ | |
      | |   |  | | IOS | | TV | | WATCH | | BRIDGE | | MACCATALYST | | |
      | |   |  | +-----+ +----+ +-------+ +--------+ +-------------+ | |
      | +---+  +-----------------------------------------------------+ |
      +----------------------------------------------------------------+

    TARGET_RT_* 
    These conditionals specify in which runtime the generated code will
    run. This is needed when the OS and CPU support more than one runtime
    (e.g. Mac OS X supports CFM and mach-o).

        TARGET_RT_LITTLE_ENDIAN - Generated code uses little endian format for integers
        TARGET_RT_BIG_ENDIAN    - Generated code uses big endian format for integers    
        TARGET_RT_64_BIT        - Generated code uses 64-bit pointers    
        TARGET_RT_MAC_CFM       - TARGET_OS_MAC is true and CFM68K or PowerPC CFM (TVectors) are used
        TARGET_RT_MAC_MACHO     - TARGET_OS_MAC is true and Mach-O/dlyd runtime is used
        

****************************************************************************************************/

std::string MacosVersion::GetOSName() {
// 今は Mac かどうかだけ分かれば良いだけなので、TARGET_OS_MAC で分ける
#if TARGET_OS_MAC
  return "macOS";
#else
  return "Unknown OS";
#endif
}

std::string MacosVersion::GetOSVersion() {
  // "Version 10.8.2 (Build 12C60)" みたいな文字列を取得できる
  NSString* str = NSProcessInfo.processInfo.operatingSystemVersionString;
  return [str UTF8String];
}
