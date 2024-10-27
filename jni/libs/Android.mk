LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libhyperion
LOCAL_SRC_FILES := libhyperion.so
include $(PREBUILT_SHARED_LIBRARY)