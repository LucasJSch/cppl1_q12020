#ifndef ISOMETRY_H
#define ISOMETRY_H

#include <cstdint>
#include <iostream>
#include <ostream>

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

class Matrix4{
    public:
        // Constructors
        Matrix4(const Vector4& r1 = Vector4(), 
                const Vector4& r2 = Vector4(), 
                const Vector4& r3 = Vector4(),
                const Vector4& r4 = Vector4())
                 : r1_{r1}, r2_{r2}, r3_{r3}, r4_{r4} {}
        Matrix4(const Matrix4& m) : Matrix4(m.row(0), m.row(1), m.row(2), m.row(3)) {}
        Matrix4(const std::initializer_list<double>& list);
        Matrix4(const std::initializer_list<double>&,
                const std::initializer_list<double>&,
                const std::initializer_list<double>&,
                const std::initializer_list<double>&);
        Matrix4(Matrix4&& m) : Matrix4(m.row(0), m.row(1), m.row(2), m.row(3)){}

        // Operators
        Vector4& operator [] (const uint8_t);
        const Vector4& operator [] (const uint8_t) const;
        Matrix4 operator + (const Matrix4&) const;
        Matrix4 operator - (const Matrix4&) const;
        Matrix4 operator * (const Matrix4&) const;
        Matrix4 operator * (const double) const;
        Matrix4 operator / (const Matrix4&) const;
        Matrix4& operator = (const Matrix4&);
        Matrix4& operator += (const Matrix4&);
        Matrix4& operator -= (const Matrix4&);
        Matrix4& operator *= (const Matrix4&);
        Matrix4& operator /= (const Matrix4&);
        bool operator == (const Matrix4&);
        bool operator != (const Matrix4&);
        Vector4 operator * (const Vector4&);
        Matrix4& operator = (Matrix4&&);

        friend Matrix4 operator * (const double d, const Matrix4& m) {
                return Matrix4(m.r1_ * d, m.r2_ * d, m.r3_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Matrix4& m) {
            os << std::string("[");
            for(int i = 0; i < 3; i++) {
                os << m.row(i);
            }
            return os;
        }

        // Getters
        Vector4 row(uint8_t index) const;
        Vector4 col(uint8_t index) const;

        // Computations
        double det() const;

        // Class constants
        static const Matrix4 kIdentity;
        static const Matrix4 kZero;
        static const Matrix4 kOnes;

    private:
        // Ordered by rows.
        Vector4 r1_, r2_, r3_, r4_;

};

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
