# http 接口文档

## 停止http服务

**GET**：/stop
```json
No parameters
```
```json
Stopping server...
```
## 测试http通信

**GET**：/hi
```json
No parameters
```
```json
Hello World!
```

## 底盘移动控制接口（摇杆控制，不带避障）

**POST**：/rockMove
```json
{
    "linear": 1.0,  // 线速度（正数为前进，负数为后退，数值为速度大小）
    "angular": 2.0  // 角速度（正数为左转，负数为右转，数值为角速度大小）
    // tips：指令默认控制时长为200ms（此接口暂不支持修改此默认值）
}
```
```json
    "status": 1,      // 1表示成功，0表示失败
    "message": "linear=1.0, angular=2.0"
```

## 底盘移动控制接口（点动，不带避障）

**POST**：/moveManual
```json
{
    "direction": 0  // 0：前进，1：后退，2：左转，3：右转。tips：指令默认控制时长为500ms
}
```
```json
{
    "status": 1,      // 1表示成功，0表示失败
    "message": "linear=1.0, angular=2.0"
}
```

## 设置底盘移动控制速度（仅作用于点动模式）

**POST**：/setSpeed
```json
{
    "trans_vel": 0.5, // 点动模式下线速度，默认0.5m/s
    "rot_vel": 0.5,   // 点动模式下角速度，默认0.5m/s
    "duration": 500   // 点动模式下控制指令持续时间，默认500ms
    // tips: 1、指令持续时间不宜设置过长，2、三个参数至少传入其一
}
```
```json
{
    "status": 1,      // 1表示成功，0表示失败
    "message": "xxx"
}
```


## 获取底盘位置

**GET**：/getCoordinate
```json
No parameters
```
```json
// 成功返回字段
{   
    "status": 1,    // 1：表示成功
    "x": 1.0,
    "y": 2.0,
    "yaw": 1.2,
}
// 失败返回字段
{
    "status": 0,    // 0: 表示失败
    "message": "xxx"
}
```

## 发送导航点并开始导航

**POST**：/sendGoal
```json
{
    "x": 1.0,
    "y": 2.0,
    "yaw": 1.2
}
```
```json
{   
    "status": 1,    // 1：表示成功，0表示失败
    "goal_id": "123e4567e89b12d3a456426614174000",   //  目标点的唯一id，失败时无此字段
    "message": "xxx"    // 成功时无此字段 
}
```

## 获取导航状态

**GET**：/navStatus
```json
No parameters
```
```json
{
    "goal_id": "53abe9078dbec68757bbdb8709d75cc2",
    "status": 1, // 1：成功，0：失败
    "message": "xxx",
    "data": 1   // 导航状态
    // data：
    // -1:              从未执行过导航任务
    //  0: unknown      未知
    //  1: accepted     已接收，未执行
    //  2: executing    正在执行
    //  3: canceling    正在取消
    //  4: succeeded    已成功完成
    //  5: canceled     已被取消
    //  6: aborted      被中止（执行过程中遇到不可恢复的错误）
}
```

## 建图时获取地图 

**GET**：/map
```json
No parameters
```
```json
{   
    "status": 1, // 1、表示成功，0、表示失败
    "message": "xxx",
    // 地图获取失败没有以下字段。tips：获取失败可重新获取，获取频率建议最大1hz（即获取间隔时间大于1s）
    "data": [],  // 地图数据  
    "info":
        {
            "frame_id": "map",      // 地图所在坐标系
            "resolution": 0.05,     // 地图分辨率（单位米）
            "width": 857,           // 地图宽度（857（个） * 0.05（米）= 42.85（米）
            "height": 652,          // 地图高度
            "origin":               // 地图原点在世界坐标系下位置（原点为地图左下角点）
                {
                    "position":     // 原点在世界坐标系下位置
                        {
                            "x": -24.845684814453126,
                            "y": -12.323123168945314,
                            "z": 0.0
                        },
                    "orientation":  // 原点在世界坐标系下的姿态（四元数）
                        {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "w": 1.0
                        }
                } 
        }
}
```

## 导航路径
**GET**：/path
```json
No parameters
```
```json
{   
    "status": 1,    // tips: 路径获取状态只有1，路径为导航状态下一个客观参考输出，以一定频率获取即可
    "message": "xxx",
    "x":[], // 路径点的x坐标
    "y":[], 
    "z":[]
}
```

## 保存地图
**POST**：/saveMap
```json
{
// 传空json，将地图保存到默认位置用于导航
}
// 下面传参形式用于将地图相关文件保存到自定义位置，方便拷贝给其它底盘。
{
    "pbstream_path": "/home/xxx/xxx.pbstream", // 保存的文件名，为全局路径格式
    "map_name": "/home/xxx/xxx",    // 地图格式地图的保存名，全局路径格式 
    "resolution": 0.05              // 分辨率
}
```
```json
{   
    "status": 1, //  1：表示成功，0：表示失败
    "save_status": -1,  // 地图保存状态，-1: 失败，0：未知，3：成功
    "message": "xxx"
    // tips：因为地图保存服务比较耗时，"save_status"返回-1或0不一定代表失败，可用"/saveMapStatus"稍后查询
}
```

## 查询地图保存状态
**GET**：/saveMapStatus
```json
No parameters
```
```json
{   
    "status": 1,
    "save_status": -1,
    "message": "xxx"
}
    // -1: 失败，0：未知，3：成功
    // 仅用于保存地图后查询地图保存状态，保存地图需要请求内部服务和状态，可能会因为多线程和锁的
    // 在导致http服务提前返回或中止，因此可用此请求轮询保存状态。tips：保存地图接口"/saveMap"
    // 的"save_status"字段返回成功可不轮询此接口
```

## 清除保存在默认位置的地图
**GET**：/clearMap
```json
No parameters
```
```json
{
    "status": 1,
    "message": "xxx"
}
```

## 进入地图续建模式
**POST**：/run
```json
{
    "data": 3
    // 3：进入地图续建
    // 4：加载禁行区，tips：在将绘制好的禁行区保存到默认位置后执行此操作可动态加载
    // 2：关闭所有程序（http接口程序除外，此时大部分接口不可用）// 尽量不用
    // 1：开启所有程序（调用‘关闭’接口后http程序保留时，此接口可用）// 尽量不用
}
```
```json
{
    "status": 1,
    "run_status": 1, // 1：成功，-1：失败，0：未知
    "messages": "xxx" // 附加日志信息
}
```