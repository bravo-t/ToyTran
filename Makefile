CC          = g++
LD          = g++ 
CFLAG       = -Wall -O3
PROG_NAME   = trans

SRC_DIR     = .
BUILD_DIR   = ./build
BIN_DIR     = .
SRC_LIST = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_LIST = $(subst .cpp,.o,$(SRC_LIST))
OBJ_FULL_LIST = $(subst ./,./build/,$(OBJ_LIST))

.PHONY: all clean $(PROG_NAME)

all: $(PROG_NAME)

%.o: %.cpp
	$(CC) $(CFLAG) -o $(BUILD_DIR)/$@ -c $<

$(PROG_NAME): main.cpp $(OBJ_LIST)
	$(LD) $(OBJ_FULL_LIST) -o $(BIN_DIR)/$@

clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
