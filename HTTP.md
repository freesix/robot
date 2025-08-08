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

## 底盘移动控制接口

**POST**：/move
```json
{
    "linear": 1.0,  // 线速度
    "angular": 2.0  // 角速度
}
```
```json
{"status", "OK"}, {"linear", 1.0}, {"angular", 2.0}
```

## 获取底盘位置

**GET**：/robot_pose
```json
No parameters
```
```json
{
    "x": 1.0,
    "y": 2.0,
    "yaw": 1.2
}
```

## 发送导航点并开始导航

**POST**：/send_goal
```json
{
    "x": 1.0,
    "y": 2.0,
    "yaw": 1.2
}
```
```json
{
    "goal_id": "123e4567e89b12d3a456426614174000"
}
```

## 获取导航状态

**GET**：/nav_status
```json
No parameters
```
```json
{
    "goal_id":"53abe9078dbec68757bbdb8709d75cc2","status":4
    // status：
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


