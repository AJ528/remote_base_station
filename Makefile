# tool macros
CC ?= gcc


# path macros
BIN_DIR = bin
OBJ_DIR = obj
DEP_DIR = dep


SRC_DIRS = \
	src \
	src/driver

INC_DIRS = \
	src/driver/include

INC_FLAGS := $(addprefix -I,$(INC_DIRS))

VPATH = $(SRC_DIRS)

TARGET := $(BIN_DIR)/test.out


# DBGFLAGS := -g


CSRCS := $(foreach x, $(SRC_DIRS), $(wildcard $(addprefix $(x)/*,.c)))

SRCS := $(CSRCS)


OBJS := $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(notdir $(basename $(SRCS)))))
DEPS := $(addprefix $(DEP_DIR)/, $(addsuffix .d, $(notdir $(basename $(SRCS)))))



CFLAGS := 			\
	-g				\
	-Wall


$(TARGET): $(OBJS) pdebug | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(INC_FLAGS) -c $< -o $@

$(DEP_DIR)/%.d: %.c | $(DEP_DIR)
	@echo DEPEND: $(CC) -MM $(CFLAGS) $<
	@set -e; rm -f $@; \
	$(CC) -MM $(CFLAGS) $(INC_FLAGS) $< > $@.$$$$; \
	sed 's,\($*\.o\)[ :]*,$(OBJ_DIR)/\1 $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

$(BIN_DIR) $(OBJ_DIR) $(DEP_DIR):
	mkdir -p $@


.PHONY: clean pdebug

pdebug:
	@echo "Source Directories = " $(SRC_DIRS)
	@echo "Include Directories = " $(INC_DIRS)

clean:
	rm -r $(BIN_DIR) $(OBJ_DIR) $(DEP_DIR)

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif