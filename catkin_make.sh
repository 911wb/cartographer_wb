#!/bin/bash

catkin_make_isolated --install --use-ninja

#函数跳转的编译
#catkin_make_isolated --install --use-ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
# 每次改完launch和lua文件，都要执行这个文件才能生效

source install_isolated/setup.bash
