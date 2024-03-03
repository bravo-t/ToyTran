CC          = g++
LD          = g++ 
CFLAG       = -Wall -O3
PROG_NAME   = trans

SRC_DIR     = .
BUILD_DIR   = ./build
BIN_DIR     = .
SRC_LIST = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_LIST = $(BUILD_DIR)/$(notdir $(SRC_LIST:.cpp=.o))

.PHONY: all clean $(PROG_NAME) compile

all: $(PROG_NAME)

compile: 
	$(CC) -c $(CFLAG) $(SRC_LIST) -o $(OBJ_LIST)

$(PROG_NAME): compile
	$(LD) $(OBJ_LIST) -o $(BIN_DIR)/$@

clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
