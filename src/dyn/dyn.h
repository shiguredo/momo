#ifndef DYN_DYN_H_
#define DYN_DYN_H_

#include <map>
#include <memory>
#include <string>

// Linux
#include <dlfcn.h>

namespace dyn {
class DynModule {
 public:
  static DynModule& Instance() {
    static DynModule instance;
    return instance;
  }

  bool IsLoadable(const char* name) {
    void* module = dlopen(name, RTLD_LAZY);
    if (module == nullptr) {
      return false;
    }
    dlclose(module);
    return true;
  }

  void* Get(const char* name) {
    auto it = modules_.find(name);
    if (it != modules_.end()) {
      return it->second.get();
    }
    void* module = dlopen(name, RTLD_LAZY);
    if (module == nullptr) {
      return nullptr;
    }
    modules_.insert(decltype(modules_)::value_type(
        name, std::unique_ptr<void, dlcloser>(module)));
    return module;
  }

 private:
  struct dlcloser {
    void operator()(void* p) {
      if (p != nullptr) {
        ::dlclose(p);
      }
    }
  };

  std::map<std::string, std::unique_ptr<void, dlcloser>> modules_;
};
}  // namespace dyn

#define DYN_REGISTER(soname, func)                              \
  template <class... Args>                                      \
  inline auto func(Args... args) {                              \
    typedef std::add_pointer<decltype(::func)>::type func_type; \
    void* module = DynModule::Instance().Get(soname);           \
    if (module == nullptr) {                                    \
      exit(1);                                                  \
    }                                                           \
    auto f = (func_type)dlsym(module, #func);                   \
    if (f == nullptr) {                                         \
      exit(1);                                                  \
    }                                                           \
    return f(args...);                                          \
  }

#endif // DYN_DYN_H_
