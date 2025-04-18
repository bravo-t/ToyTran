ifdef DEBUG
  PRE_CFLAGS = -g -O0 -DDEBUG=$(DEBUG)
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

SRC_LIST = main.cpp  Plotter.cpp TR0Writer.cpp Debug.cpp Measure.cpp \
		   Simulator.cpp StepControl.cpp SimResult.cpp \
		   Circuit.cpp MNAStamper.cpp  NetlistParser.cpp 

SRC_LIST_TMP = $(patsubst %,./%,$(SRC_LIST))
OBJ_LIST = $(subst .cpp,.o,$(SRC_LIST_TMP))
OBJ_FULL_LIST = $(subst ./,./build/,$(OBJ_LIST))
HEADER_LIST = $(wildcard $(SRC_DIR)/*.h)

default: $(PROG_NAME)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADER_LIST) | $(BUILD_DIR)
	$(CC) $(CFLAG) -o $(BUILD_DIR)/$*.o -c $<

$(PROG_NAME): src/main.cpp $(OBJ_FULL_LIST) | $(BUILD_DIR)
	$(LD) $(OBJ_FULL_LIST) -o $(BIN_DIR)/$@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: clean
clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
