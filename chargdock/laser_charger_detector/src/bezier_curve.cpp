#include "bezier_curve.hpp"
#include <algorithm>
#include <limits>

BezierCurve::BezierCurve(const Point2D& start, const Point2D& control, const Point2D& end)
    : start_(start), control_(control), end_(end)
{
}

BezierCurve::Point2D BezierCurve::getPoint(double t) const
{
    // 二次贝塞尔曲线公式: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
    t = std::max(0.0, std::min(1.0, t)); // 限制t在[0,1]范围内
    
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double ut2 = 2.0 * u * t;
    
    Point2D result;
    result.x = uu * start_.x + ut2 * control_.x + tt * end_.x;
    result.y = uu * start_.y + ut2 * control_.y + tt * end_.y;
    
    return result;
}

BezierCurve::Point2D BezierCurve::getDerivative(double t) const
{
    // 二次贝塞尔曲线导数: B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁)
    t = std::max(0.0, std::min(1.0, t));
    
    Point2D p1_minus_p0 = control_ - start_;
    Point2D p2_minus_p1 = end_ - control_;
    
    Point2D derivative;
    derivative.x = 2.0 * (1.0 - t) * p1_minus_p0.x + 2.0 * t * p2_minus_p1.x;
    derivative.y = 2.0 * (1.0 - t) * p1_minus_p0.y + 2.0 * t * p2_minus_p1.y;
    
    return derivative;
}

BezierCurve::Point2D BezierCurve::getTangent(double t) const
{
    Point2D derivative = getDerivative(t);
    return derivative.normalized();
}

double BezierCurve::getLength() const
{
    // 使用数值积分近似计算曲线长度
    const int segments = 1000;
    double length = 0.0;
    Point2D prev_point = getPoint(0.0);
    
    for (int i = 1; i <= segments; ++i) {
        double t = static_cast<double>(i) / segments;
        Point2D current_point = getPoint(t);
        length += (current_point - prev_point).magnitude();
        prev_point = current_point;
    }
    
    return length;
}

double BezierCurve::findClosestT(const Point2D& current_pos) const
{
    // 使用二分搜索和梯度下降找到最近的t参数
    double best_t = 0.0;
    double min_distance = std::numeric_limits<double>::max();
    
    // 粗略搜索
    const int coarse_steps = 100;
    for (int i = 0; i <= coarse_steps; ++i) {
        double t = static_cast<double>(i) / coarse_steps;
        Point2D curve_point = getPoint(t);
        double distance = (curve_point - current_pos).magnitude();
        
        if (distance < min_distance) {
            min_distance = distance;
            best_t = t;
        }
    }
    
    // 精细搜索（在最佳点附近）
    double search_range = 1.0 / coarse_steps;
    double start_t = std::max(0.0, best_t - search_range);
    double end_t = std::min(1.0, best_t + search_range);
    
    const int fine_steps = 100;
    for (int i = 0; i <= fine_steps; ++i) {
        double t = start_t + (end_t - start_t) * i / fine_steps;
        Point2D curve_point = getPoint(t);
        double distance = (curve_point - current_pos).magnitude();
        
        if (distance < min_distance) {
            min_distance = distance;
            best_t = t;
        }
    }
    
    return best_t;
}

std::vector<BezierCurve::Point2D> BezierCurve::generatePath(int num_points) const
{
    std::vector<Point2D> path;
    path.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        path.push_back(getPoint(t));
    }
    
    return path;
}

double BezierCurve::distanceToCurve(const Point2D& point) const
{
    double closest_t = findClosestT(point);
    Point2D closest_point = getPoint(closest_t);
    return (point - closest_point).magnitude();
} 