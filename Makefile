#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := gpioTask

EXTRA_COMPONENT_DIRS = $(IDF_PATH)/examples/common_components/protocol_examples_common $(CURDIR)/../../components

EXCLUDE_COMPONENTS = max7219 mcp23x17 led_strip

include $(IDF_PATH)/make/project.mk

