#ifndef BEZIER_CURVE_HPP
#define BEZIER_CURVE_HPP

#include <vector>
#include <cmath>

class BezierCurve {
public:
    struct Point2D {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
        
        // 重载减法运算符
        Point2D operator-(const Point2D& other) const {
            return Point2D(x - other.x, y - other.y);
        }
        
        // 重载加法运算符
        Point2D operator+(const Point2D& other) const {
            return Point2D(x + other.x, y + other.y);
        }
        
        // 重载乘法运算符
        Point2D operator*(double scalar) const {
            return Point2D(x * scalar, y * scalar);
        }
        
        // 计算模长
        double magnitude() const {
            return sqrt(x * x + y * y);
        }
        
        // 归一化
        Point2D normalized() const {
            double mag = magnitude();
            if (mag > 1e-6) {
                return Point2D(x / mag, y / mag);
            }
            return Point2D(0, 0);
        }
    };

    // 构造函数，接受三个控制点：起点、控制点、终点
    BezierCurve(const Point2D& start, const Point2D& control, const Point2D& end);
    
    // 计算贝塞尔曲线上t参数对应的点 (t ∈ [0,1])
    Point2D getPoint(double t) const;
    
    // 计算贝塞尔曲线上t参数对应的切线方向（归一化）
    Point2D getTangent(double t) const;
    
    // 获取曲线长度的近似值
    double getLength() const;
    
    // 根据当前位置找到最近的t参数
    double findClosestT(const Point2D& current_pos) const;
    
    // 生成路径点
    std::vector<Point2D> generatePath(int num_points = 100) const;
    
    // 计算到曲线的最短距离
    double distanceToCurve(const Point2D& point) const;

private:
    Point2D start_, control_, end_;
    
    // 计算二次贝塞尔曲线的导数
    Point2D getDerivative(double t) const;
};

#endif // BEZIER_CURVE_HPP 