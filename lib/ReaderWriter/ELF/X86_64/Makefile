##===- lld/lib/ReaderWriter/ELF/X86_64/Makefile ----------*- Makefile -*-===##
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
##===----------------------------------------------------------------------===##

LLD_LEVEL := ../../../..
LIBRARYNAME := lldX86_64ELFTarget
USEDLIBS = lldCore.a

CPP.Flags += -I$(PROJ_SRC_DIR)/$(LLD_LEVEL)/lib/ReaderWriter/ELF
CPP.Flags += -I$(PROJ_SRC_DIR)/$(LLD_LEVEL)/lib/ReaderWriter/ELF/X86_64/

PARALLEL_DIRS := ExampleSubTarget

include $(LLD_LEVEL)/Makefile
