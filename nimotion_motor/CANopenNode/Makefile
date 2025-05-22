# 定义编译器和编译选项
CXX = g++
CXXFLAGS = -Wall -lpthread
LDFLAGS = -lpthread

# 定义共享库路径
CONTROLCAN_LIB = /home/tianbot/Bachelor_ws/src/StewartBot/nimotion_motor/CANopenNode/libcontrolcan.so

# 定义所有目标
all: controlcan test

# 编译controlcan
controlcan: main.cpp
	$(CXX) -o $@ $< $(CONTROLCAN_LIB) $(LDFLAGS)

# 编译test
test: main_socketcan.cpp
	$(CXX) -o $@ $< $(CXXFLAGS)

# 清理生成文件
clean:
	rm -f controlcan test *.o
