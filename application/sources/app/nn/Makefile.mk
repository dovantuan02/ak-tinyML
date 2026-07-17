include sources/app/nn/inference/motion_direct_classify/Makefile.mk

CFLAGS		+= -I./sources/app/nn
CPPFLAGS	+= -I./sources/app/nn


VPATH += sources/app/nn

# CPP source files
SOURCES_CPP += sources/app/nn/nn_infer.cpp
