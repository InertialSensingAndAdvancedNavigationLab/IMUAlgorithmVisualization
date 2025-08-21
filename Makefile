# 编译器和标志
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
LDFLAGS =

# 目录
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
TARGET = $(BIN_DIR)/ahrs_tester

# 包含路径
INCLUDES = -I./$(SRC_DIR) -I/usr/include/eigen3

# 自动查找所有 C++ 源文件
SRCS = $(shell find $(SRC_DIR) -name '*.cpp')

# 根据源文件自动生成目标文件路径
# 例如: src/main.cpp -> obj/main.o
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# 根据目标文件自动生成依赖文件路径
# 例如: obj/main.o -> obj/main.d
DEPS = $(OBJS:.o=.d)

# 默认目标
all: $(TARGET)

# 链接可执行文件的规则
$(TARGET): $(OBJS)
	@echo "链接: $@"
	@mkdir -p $(@D)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# 编译 .cpp 文件为 .o 文件的规则
# -MMD -MP 会为每个源文件自动生成头文件依赖关系
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo "编译: $<"
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

# 清理生成的文件
clean:
	@echo "正在清理..."
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# 包含所有生成的依赖文件。
# 前缀 '-' 告诉 make，如果文件不存在也不要报错。
-include $(DEPS)

# 伪目标
.PHONY: all clean