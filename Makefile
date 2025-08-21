# include app info
APP ?= basic
# APP ?= svpwm_test
# APP ?= pwm_test
# APP ?= encoder_test
# APP ?= filter_low_pass_test
# APP ?= idle_test

# include port info
PORT ?= euler_stm32g431
# PORT ?= boot_stm32g474re


# set special cflag, if need.
COMMON_FLAGS  := -O2
# COMMON_FLAGS  := -Os

# show compile debug info.
# V := 1

# For user show current app info.







# don't edit below this line.
# include app info
APP_ROOT_PATH = example
APP_PATH = $(APP_ROOT_PATH)/$(APP)
include $(APP_PATH)/build.mk

# include egui src
EASY_FOC_PATH := src
include $(EASY_FOC_PATH)/build.mk


# include port info
CUST_PORT_ROOT_PATH = porting
CUST_PORT_PATH = $(CUST_PORT_ROOT_PATH)/$(PORT)
include $(CUST_PORT_PATH)/build.mk
