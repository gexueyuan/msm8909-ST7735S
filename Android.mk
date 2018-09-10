LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	screencap.cpp \
	7735s_spi.c

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libutils \
	libbinder \
	libskia \
    libui \
    libgui

LOCAL_MODULE:= fb2spi

LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)
