JETSON_MMAPI=$(shell [ -d "/usr/src/jetson_multimedia_api" ] && echo 1 || echo 0 )
ifeq ($(JETSON_MMAPI), 1)
	include /usr/src/jetson_multimedia_api/samples/Rules.mk
else
	include /usr/src/tegra_multimedia_api/samples/Rules.mk
endif

#compile-time definitions
CPPFLAGS += -DLT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
CPPFLAGS:=$(filter-out -std=c++11,$(CPPFLAGS))
CPPFLAGS += -std=c++14

src = $(wildcard *.cpp)
obj = $(src:.cpp=.o)
dep = $(obj:.o=.d)  # one dependency file for each source
LDFLAGS += -L/home/radian/LT_ivision/inc/include -L/home/radian/LT_ivision/inc/jetson-inference -L/home/nvidia/LT_iVision_from_git/inc/include -L/usr/local/lib -L/usr/local/lib/tegra -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_calib3d -lopencv_features2d -lopencv_photo -lopencv_cudaarithm -lopencv_cudaimgproc -lopencv_cudafilters -lopencv_cudawarping -lpthread -lnveglstream_camconsumer -lnvargus -lprotobuf -L/home/nvidia/jetson-inference/build/aarch64/lib -ljetson-inference -ljetson-utils
EXE=ivision

ARGUS_UTILS_DIR := $(TOP_DIR)/argus/samples/utils

SRCS := $(wildcard $(CLASS_DIR)/*.cpp)

OBJS := $(SRCS:.cpp=.o)

inc =-I../inc -I../inc/include -I"$(ARGUS_UTILS_DIR)" -I/usr/local/include -I/usr/include -I/usr/local/include/opencv4 -I../inc/jetson-inference -I../inc/jetson-utils -I/home/nvidia/jetson-inference/build/aarch64/include -I/home/nvidia/jetson-inference/build/aarch64/include/jetson-inference -I/home/nvidia/jetson-inference/build/aarch64/include/jetson-utils

$(EXE): $(obj)
	$(CXX) -o $@ $^ $(CLASS_DIR)/NvJpegEncoder.o $(CLASS_DIR)/NvElement.o $(CLASS_DIR)/NvElementProfiler.o $(CLASS_DIR)/NvLogging.o $(LDFLAGS)

.PHONY: debug
debug: CPPFLAGS += -g
debug: $(obj)
	$(CXX) -g -o $(EXE) $^ $(CLASS_DIR)/NvJpegEncoder.o $(CLASS_DIR)/NvElement.o $(CLASS_DIR)/NvElementProfiler.o $(CLASS_DIR)/NvLogging.o $(LDFLAGS)

-include $(dep)   # include all dep files in the makefile

# rule to generate a dep file by using the C preprocessor
# (see man cpp for details on the -MM and -MT options)
# %.d: %.c
# 	@$(CXX) $(inc) $< -MM -MT $(@:.d=.o) >$@
.cpp.o:
	 $(CXX) $(CPPFLAGS) $(inc) -c $<

.PHONY: clean
clean:
	rm -f $(obj) $(EXE)

.PHONY: cleandep
cleandep:
	rm -f $(dep)
