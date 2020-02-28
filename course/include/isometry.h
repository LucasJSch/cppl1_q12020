#ifndef ISOMETRY_H
#define ISOMETRY_H

#include <cstdint>
#include <iostream>

namespace ekumen {
namespace math {

class Vector3{
    public:
        // Constructors
        Vector3(double x = 0, double y = 0, double z = 0) : x_{x}, y_{y}, z_{z} {}
        Vector3(const Vector3& v) : Vector3(v.x_, v.y_, v.z_) {}
        Vector3(const std::initializer_list<double>& l) {
            if(l.size() != 3) {
                throw "Invalid initializer list size";
            }
            std::initializer_list<double>::iterator it = l.begin();
            x_ = *it++;
            y_ = *it++;
            z_ = *it;
        }

        // Operators
        double& operator [] (const uint8_t);
        const double& operator [] (const uint8_t) const;
        Vector3 operator + (const Vector3&) const;
        Vector3 operator - (const Vector3&) const;
        Vector3 operator * (const Vector3&) const;
        Vector3 operator * (const double&) const;
        Vector3 operator / (const Vector3&) const;
        Vector3& operator = (const Vector3&);
        Vector3& operator += (const Vector3&);
        Vector3& operator -= (const Vector3&);
        Vector3& operator *= (const Vector3&);
        Vector3& operator /= (const Vector3&);
        bool operator == (const Vector3&) const;
        bool operator != (const Vector3&) const;
        bool operator == (const std::initializer_list<double>& list) const;
        bool operator != (const std::initializer_list<double>& list) const;

        friend Vector3 operator * (const double d, const Vector3& v) {
            return Vector3(v.x_ * d, v.y_ * d, v.z_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Vector3& v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ")";
            return os;
        }

        // Getters
        const double& x() const {return x_;};
        const double& y() const {return y_;};
        const double& z() const {return z_;};

        // Setters
        double& x() {return x_;};
        double& y() {return y_;};
        double& z() {return z_;};

        // Computations
        double dot(const Vector3&) const;
        double norm() const;
        Vector3 cross(const Vector3&) const;

        // Class constants
        static const Vector3 kUnitX;
        static const Vector3 kUnitY;
        static const Vector3 kUnitZ;
        static const Vector3 kZero;

    private:
        // Attributes
        double x_;
        double y_;
        double z_;
};

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
