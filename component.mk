#
# Component Makefile
#
ifdef CONFIG_WMNGR_ENABLED
COMPONENT_ADD_INCLUDEDIRS := include
COMPONENT_SRCDIRS := src

CFLAGS += -Wall -Wextra -Werror
endif