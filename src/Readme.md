### 测试agvcallback和warncallback接口
### queryAgvStatus获得的AGV状态不一定能用来判断，尤其是底盘锁定判断，可以使用agvcallback进行（猜测执行任务结束为outbin）
### 注意lockBase(self)，UnlockBase(self)，heartbeat_monitoring(self)三个函数的判断

# 依赖

## 安装catkin工具和一些可能的依赖
```
sudo apt install python3-catkin-tool liblua5.2-dev libceres-dev
```

## 安装abseil库
运行以下脚本一键安装
```
src/platform/hf_mapping/carto/cartographer/scripts/install_abseil.sh
```
# 编译过程

## 编译整个工作空间
```
catkin build
```

按提示，缺哪些依赖直接用apt安装即可

