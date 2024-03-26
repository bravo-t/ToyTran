ifdef DEBUG
  PRE_CFLAGS = -g -O0 -DDEBUG=$(DEBUG)
 else
  PRE_CFLAGS = -O3
endif

CC          = g++
LD          = g++ 
CFLAG       = -Wall -Wextra $(PRE_CFLAGS)
PROG_NAME   = trans

CFLAG+=-Isubmodule/eigen

SRC_DIR     = .
BUILD_DIR   = ./build
BIN_DIR     = .

SRC_LIST = main.cpp  Plotter.cpp TR0Writer.cpp Simulator.cpp Circuit.cpp MNAStamper.cpp  NetlistParser.cpp Debug.cpp

SRC_LIST_TMP = $(patsubst %,./%,$(SRC_LIST))
OBJ_LIST = $(subst .cpp,.o,$(SRC_LIST_TMP))
OBJ_FULL_LIST = $(subst ./,./build/,$(OBJ_LIST))

default: $(PROG_NAME)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CC) $(CFLAG) -o $(BUILD_DIR)/$*.o -c $<

$(PROG_NAME): main.cpp $(OBJ_FULL_LIST) | $(BUILD_DIR)
	$(LD) $(OBJ_FULL_LIST) -o $(BIN_DIR)/$@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: clean
clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
