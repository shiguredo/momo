#ifndef SORA_VPL_SESSION_H_
#define SORA_VPL_SESSION_H_

#include <memory>

namespace sora {

struct VplSession {
  // Intel VPL のセッションを作成する
  // 対応してない場合やエラーが発生した場合は nullptr を返す
  static std::shared_ptr<VplSession> Create();
};

}  // namespace sora

#endif
