TARGET = main

# repertoire des modules
AVERSIVE_DIR = ../libs/aversive4dspic
#AVERSIVE_DIR = ../libs/microb_2010

SRC  = $(TARGET).c 
SRC += cmdline.c commands_gen.c
SRC += commands_cs.c commands_mainboard.c commands_traj.c commands.c
SRC += actuator.c cs.c sensor.c strat.c strat_base.c strat_utils.c
SRC += telemetry.c

ifeq ($(H),1)
SRC += robotsim.c
endif

ASRC =

CFLAGS += -Wall
#-Werror -Wno-unused-but-set-variable -Wno-unused-value -Wno-unused-result

########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk

AVRDUDE_DELAY=50

program_noerase: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	echo $(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
