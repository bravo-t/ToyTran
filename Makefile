ifdef DEBUG
  ifeq ($(DEBUG), 1)
    PRE_CFLAGS = -g -O0 -DDEBUG=$(DEBUG)
  else
    PRE_CFLAGS = -O3
  endif
else 
  PRE_CFLAGS = -O3
endif

CC          = g++
LD          = g++ 
CFLAG       = -Wall -Wextra $(PRE_CFLAGS)
PROG_NAME   = trans

SRC_DIR     = ./src
BUILD_DIR   = ./build
BIN_DIR     = .

CFLAG+=-I$(SRC_DIR)/submodule/eigen

SRC_LIST = main.cpp NetworkAnalyzer.cpp Plotter.cpp TR0Writer.cpp Debug.cpp \
		   Measure.cpp PoleZero.cpp Simulator.cpp StepControl.cpp SimResult.cpp \
		   Circuit.cpp MNAStamper.cpp  NetlistParser.cpp LibData.cpp rpoly.cpp

SRC_LIST_TMP = $(patsubst %,./%,$(SRC_LIST))
SRC_FULL_LIST = $(patsubst %,$(SRC_DIR)/%,$(SRC_LIST))
OBJ_LIST = $(subst .cpp,.o,$(SRC_LIST_TMP))
OBJ_FULL_LIST = $(subst ./,$(BUILD_DIR)/,$(OBJ_LIST))
DEPS_FILE = .deps

default: $(PROG_NAME)

$(PROG_NAME): src/main.cpp $(OBJ_FULL_LIST)
	$(LD) $(OBJ_FULL_LIST) -o $(BIN_DIR)/$@

$(DEPS_FILE): $(SRC_FULL_LIST)
	@-$(foreach src, $(SRC_FULL_LIST), \
		$(CC) $(CFLAG) -MM $(src) -MT $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(src)) >> "$@"; \
	)

-include $(DEPS_FILE)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D) || true
	$(CC) $(CFLAG) -o $(BUILD_DIR)/$*.o -c $<

.PHONY: clean 
clean:
	-rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/* $(DEPS_FILE)
