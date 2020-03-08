#ifndef ISOMETRY_H
#define ISOMETRY_H

#include <cstdint>
#include <iostream>

namespace ekumen {
namespace math {

class Vector4{
    public:
        // Constructors
        Vector4(double x = 0, double y = 0, double z = 0, double w = 0) : x_{x}, y_{y}, z_{z}, w_{w} {}
        Vector4(const Vector4& v) : Vector4(v.x_, v.y_, v.z_, v.w_) {}
        Vector4(const std::initializer_list<double>& l) {
            if(l.size() != 4) {
                throw "Invalid initializer list size";
            }
            std::initializer_list<double>::iterator it = l.begin();
            x_ = *it++;
            y_ = *it++;
            z_ = *it++;
            w_ = *it;
        }

        // Operators
        double& operator [] (const uint8_t);
        const double& operator [] (const uint8_t) const;
        Vector4 operator + (const Vector4&) const;
        Vector4 operator - (const Vector4&) const;
        Vector4 operator * (const Vector4&) const;
        Vector4 operator * (const double&) const;
        Vector4 operator / (const Vector4&) const;
        Vector4& operator = (const Vector4&);
        Vector4& operator += (const Vector4&);
        Vector4& operator -= (const Vector4&);
        Vector4& operator *= (const Vector4&);
        Vector4& operator /= (const Vector4&);
        bool operator == (const Vector4&) const;
        bool operator != (const Vector4&) const;
        bool operator == (const std::initializer_list<double>& list) const;
        bool operator != (const std::initializer_list<double>& list) const;

        friend Vector4 operator * (const double d, const Vector4& v) {
            return Vector4(v.x_ * d, v.y_ * d, v.z_ * d, v.w_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Vector4& v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ", w: " << v.w() << ")";
            return os;
        }

        // Getters
        const double& x() const {return x_;};
        const double& y() const {return y_;};
        const double& z() const {return z_;};
        const double& w() const {return w_;};

        // Setters
        double& x() {return x_;};
        double& y() {return y_;};
        double& z() {return z_;};
        double& w() {return w_;};

        // Computations
        double dot(const Vector4&) const;
        double norm() const;

        // Class constants
        static const Vector4 kUnitX;
        static const Vector4 kUnitY;
        static const Vector4 kUnitZ;
        static const Vector4 kUnitW;
        static const Vector4 kZero;

    private:
        // Attributes
        double x_;
        double y_;
        double z_;
        double w_;
};

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
