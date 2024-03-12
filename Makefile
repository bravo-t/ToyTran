ifdef DEBUG
  PRE_CFLAGS = -g -O0 -DDEBUG=$(DEBUG)
 else
  PRE_CFLAGS = -O3
endif

CC          = g++
LD          = g++ 
CFLAG       = -Wall $(PRE_CFLAGS)
PROG_NAME   = trans

CFLAG+=-Isubmodule/eigen

SRC_DIR     = .
BUILD_DIR   = ./build
BIN_DIR     = .
SRC_LIST = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_LIST = $(subst .cpp,.o,$(SRC_LIST))
OBJ_FULL_LIST = $(subst ./,./build/,$(OBJ_LIST))

default: $(PROG_NAME)

$(BUILD_DIR)/%.o: %.cpp
	$(CC) $(CFLAG) -o $(BUILD_DIR)/$*.o -c $<

$(PROG_NAME): main.cpp $(OBJ_FULL_LIST)
	$(LD) $(OBJ_FULL_LIST) -o $(BIN_DIR)/$@

.PHONY: clean
clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
