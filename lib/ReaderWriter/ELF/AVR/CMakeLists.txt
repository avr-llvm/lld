add_llvm_library(lldAVRELFTarget
  AVRCtorsOrderPass.cpp
  AVRLinkingContext.cpp
  AVRRelocationHandler.cpp
  AVRTargetHandler.cpp
  LINK_LIBS
    lldCore
    lldELF
    LLVMObject
    LLVMSupport
  )
