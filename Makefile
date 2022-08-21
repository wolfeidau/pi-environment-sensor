BSEC_DIR = ./bsec_2-2-0-0_generic_release_30052022

CFILES   = $(wildcard *.c)
CFILES   += $(BSEC_DIR)/examples/Bosch_BME68x_Library/src/bme68x/bme68x.c
OBJFILES = $(CFILES:.c=.o)
OUT      = bsec_bme680

CC      = gcc
CFLAGS  = -Wall -Wno-unused-but-set-variable -Wno-unused-variable -static -std=c99 -pedantic -I $(BSEC_DIR)/examples/Bosch_BME68x_Library/src/bme68x -I $(BSEC_DIR)/algo/normal_version/inc
LDFLAGS = -L $(BSEC_DIR)/examples/Bosch_BME68x_Library/src/bme68x
LDLIBS  = -lalgobsec -lm -lrt

PLATFORM := $(shell uname -m)

ifeq ($(PLATFORM),x86_64)
CFLAGS += -iquote"${BSEC_DIR}"/algo/normal_version/bin/gcc/Linux/m64
LDFLAGS += -L"${BSEC_DIR}"/algo/normal_version/bin/gcc/Linux/m64
endif

ifeq ($(PLATFORM),armv6l)
CFLAGS += -iquote"${BSEC_DIR}"/algo/normal_version/bin/RaspberryPi/PiThree_ArmV6
LDFLAGS += -L"${BSEC_DIR}"/algo/normal_version/bin/RaspberryPi/PiThree_ArmV6
endif

$(OUT): $(OBJFILES)

.PHONY: clean
clean:
	rm -f $(OBJFILES) $(OUT)

print:
	echo "arch $(PLATFORM)"


# build:
# 	cc -Wall -Wno-unused-but-set-variable -Wno-unused-variable -static \
# 		-std=c99 -pedantic \
# 		./bsec_bme680.c \
# 		-lm -lrt \
# 		-o bsec_bme680

# SRC = *.c
# OBJ = $(SRC:.c=.o)
# BIN = mybin
# CFLAGS = -Wall -Wno-unused-but-set-variable -Wno-unused-variable -static -std=c99 -pedantic -lm -lrt 
