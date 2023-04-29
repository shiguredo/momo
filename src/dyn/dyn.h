#ifndef DYN_DYN_H_
#define DYN_DYN_H_

#include <map>
#include <memory>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
// Linux
#include <dlfcn.h>
#endif

namespace dyn {
class DynModule {
 public:
  static DynModule& Instance() {
    static DynModule instance;
    return instance;
  }
#if defined(_WIN32)
  typedef HINSTANCE__ module_t;
  typedef HMODULE module_ptr_t;
#else
  typedef void module_t;
  typedef void* module_ptr_t;
#endif

  bool IsLoadable(const char* name) {
#if defined(_WIN32)
    module_ptr_t module = LoadLibraryA(name);
    if (module == nullptr) {
      return false;
    }
    FreeLibrary(module);
    return true;
#elif defined(__linux__)
    module_ptr_t module = dlopen(name, RTLD_LAZY);
    if (module == nullptr) {
      return false;
    }
    dlclose(module);
    return true;
#else
    return false;
#endif
  }

  module_ptr_t Get(const char* name) {
    auto it = modules_.find(name);
    if (it != modules_.end()) {
      return it->second.get();
    }
#if defined(_WIN32)
    module_ptr_t module = LoadLibraryA(name);
#elif defined(__linux__)
    module_ptr_t module = dlopen(name, RTLD_LAZY);
#else
    module_ptr_t module = nullptr;
#endif
    if (module == nullptr) {
      return nullptr;
    }
    modules_.insert(decltype(modules_)::value_type(
        name, std::unique_ptr<module_t, dlcloser>(module)));
    return module;
  }

  void* GetFunc(const char* soname, const char* name) {
    module_ptr_t module = Get(soname);
    if (module == nullptr) {
      return nullptr;
    }
#if defined(_WIN32)
    return ::GetProcAddress(module, name);
#elif defined(__linux__)
    return dlsym(module, name);
#else
    return nullptr;
#endif
  }

 private:
  struct dlcloser {
    void operator()(module_ptr_t p) {
      if (p != nullptr) {
#if defined(_WIN32)
        FreeLibrary(p);
#elif defined(__linux__)
        ::dlclose(p);
#else
#endif
      }
    }
  };

  std::map<std::string, std::unique_ptr<module_t, dlcloser>> modules_;
};
}  // namespace dyn

// text の定義を全て展開した上で文字列化する。
// 単純に #text とした場合、全て展開する前に文字列化されてしまう
#if defined(_WIN32)
#define DYN_STRINGIZE(text) DYN_STRINGIZE_((text))
#define DYN_STRINGIZE_(x) DYN_STRINGIZE_I x
#else
#define DYN_STRINGIZE(x) DYN_STRINGIZE_I(x)
#endif

#define DYN_STRINGIZE_I(text) #text

#define DYN_REGISTER(soname, func)                                             \
  template <class... Args>                                                     \
  inline auto func(Args... args) {                                             \
    typedef std::add_pointer<decltype(::func)>::type func_type;                \
    auto f =                                                                   \
        (func_type)DynModule::Instance().GetFunc(soname, DYN_STRINGIZE(func)); \
    if (f == nullptr) {                                                        \
      std::cerr << "Failed to load function " DYN_STRINGIZE(                   \
                       func) " in " soname                                     \
                << std::endl;                                                  \
      exit(1);                                                                 \
    }                                                                          \
    return f(args...);                                                         \
  }

#endif  // DYN_DYN_H_