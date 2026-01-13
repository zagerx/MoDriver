## Windows
- 工具1:`arm-none-abie-gcc`
  - 将 .c 源文件编译并链接生成目标文件.hex 文件
- 工具2:`CMake`
  - 统一管理与组织所有 .c 文件及其编译选项
- 工具3:`nijia`
  - 作为构建系统执行器
  - 根据 CMakeLists.txt 中生成的构建规则，完成实际的编译与链接过程
- 工具4:`JFlash`
  - 配合JLink，下载hex文件到单片机
- 工具5:clangd
- 工具6:clangd-format
---

## 单片机的编译下载基本原理
使用编译器将.c文件转为.hex文件，使用openocd+stlink/JFlash+jlink通过SWD接口将其下载到单片机内部
### 交叉编译器
常用交叉编译器示例：
- [`arm-none-abie-gcc`](https://developer.arm.com/downloads/-/gnu-rm)
#### 编译器目录结构及常用工具
假设安装路径为 arm-none-eabi-gcc/，主要目录和文件：
```
arm-none-eabi-gcc/
├── bin/          # 可执行文件目录
│   ├── arm-none-eabi-gcc       # 编译 C/CPP 文件
│   ├── arm-none-eabi-g++       # 编译 C++ 文件
│   ├── arm-none-eabi-ld        # 链接器
│   ├── arm-none-eabi-objcopy   # 生成 .hex/.bin 文件
│   ├── arm-none-eabi-objdump   # 反汇编工具
│   ├── arm-none-eabi-ar        # 静态库管理
│   └── arm-none-eabi-size      # 查看目标文件大小
├── include/      # 系统头文件
├── lib/          # 编译器运行库、链接脚本
└── share/        # 文档、支持文件
```
我们重点关注 bin/ 下的工具，尤其是 gcc、ld、objcopy 和 objdump，它们构成了单片机从源代码到可执行文件的核心流程。

#### 将bin目录添加到系统环境变量
- 在终端执行`arm-none-eabi-gcc --version`输出对应版本即为成功

## CMake和ninja
  - [CMake](https://cmake.org/download/)
    - CMakeList.txt描述那些.c文件需要编译的用什么样的规则编译，CMake读取CMakeList.txt，将其转为build.ninja
  - [ninja](https://github.com/ninja-build/ninja/releases)
    - build.ninja文件描述各个.c文件散落在何处，nijia调用编译器对这些源文件进行编译
## clangd
  - [Clangd](https://github.com/clangd/clangd/releases/tag/21.1.8)
  - Clangd根据compile_commands.json这个文件来完成下面功能
     - 实时语义补全
     - 即时诊断（红波浪线）
     - 跳转与查找
## clangd-format
  - [clangd-format](https://github.com/tqfx/clang-format/releases)
  - 代码格式化工具
  - 读取.clang-format文件来格式化代码

# vscode及插件
- CMake Tool插件
  - 可以使用图形界面的方式进行Cmake的一些操作

- Clangd插件
  1. vscode的clangd插件如何找到clangd
      - 注册clangd到系统变量
      -  `.vscode/settings.json`文件中写入绝对路径
          ```
          {
              "clangd.path": "D:\\LLVM\\bin\\clangd.exe"
          }
          ```
  2. clangd插件调用clangd如何找到compile_commands.json
      - `.vscode/settings.json`文件中追加
        ```
        {
            "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
        ```
- Clangd-format插件
  - vscode如何找到clangd-format
    - 注册到系统变量即可
  - 设置
    ```
    "[cpp]": {
        "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
        "editor.formatOnSave": true
    }
    ```
  - 该插件的功能已经集成到Clangd插件里面了