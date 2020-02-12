#ifndef ISOMETRY_H
#define ISOMETRY_H

#include <cstdint>
#include <iostream>
#include <ostream>

namespace ekumen {
namespace math {

class Vector3{
    public:
        // Constructors
        Vector3() : x_(0), y_(0), z_(0) {};
        Vector3(double x, double y = 0, double z = 0) : x_(x), y_(y), z_(z) {};
        Vector3(const Vector3& v) : x_(v.x()), y_(v.y()), z_(v.z()) {} ;

        // Destructor
        ~Vector3() {}; // is this necessary?

        // Operators
        double& operator [] (const int);
        double& operator [] (const int) const;
        Vector3 operator + (const Vector3&) const;
        Vector3 operator - (const Vector3&) const;
        Vector3 operator * (const Vector3&) const;
        Vector3 operator / (const Vector3&) const;
        Vector3 operator = (const Vector3&);
        Vector3& operator += (const Vector3&);
        Vector3& operator -= (const Vector3&);
        Vector3& operator *= (const Vector3&);
        Vector3& operator /= (const Vector3&);
        bool operator == (const Vector3&) const;
        bool operator != (const Vector3&) const;
        bool operator == (const std::initializer_list<double>& list) const;
        bool operator != (const std::initializer_list<double>& list) const;

        friend std::ostream& operator << (std::ostream& os, const Vector3& v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ")";
            return os;
        }

        // Getters
        inline double x() const {return x_;};
        inline double y() const {return y_;};
        inline double z() const {return z_;};

        // Setters
        inline double& x() {return x_;};
        inline double& y() {return y_;};
        inline double& z() {return z_;};

        // Computations
        double dot(const Vector3&) const;
        double norm() const;
        Vector3 cross(const Vector3&) const;

        // Class constants

        static const Vector3 kUnitX;
        static const Vector3 kUnitY;
        static const Vector3 kUnitZ;
        static const Vector3 kZero;
        static const int MAX_INDEX = 2;

    private:
        // Attributes
        double x_;
        double y_;
        double z_;
        // Vector data in array-form, for more efficient implementation of [] operator
        double* data_[3] = {&x_, &y_, &z_};
};

class Matrix3{
    public:
        // Constructors
        Matrix3() : r1_(), r2_(), r3_() {};
        Matrix3(Vector3& r1, Vector3& r2, Vector3& r3) : r1_(r1), r2_(r2), r3_(r3) {};
        Matrix3(const Matrix3& m) : r1_(m.row(0)), r2_(v.y()), r3_(v.z()) {} ;

        // Destructor
        ~Matrix3() {}; // is this necessary?

        // Operators
        Vector3 operator [] (const index) const;
        Matrix3 operator + (const Matrix3&) const;
        Matrix3 operator - (const Matrix3&) const;
        Matrix3 operator * (const Matrix3&) const;
        Matrix3 operator / (const Matrix3&) const;
        Matrix3 operator = (const Matrix3&);
        Matrix3& operator += (const Matrix3&);
        Matrix3& operator -= (const Matrix3&);
        Matrix3& operator *= (const Matrix3&);
        Matrix3& operator /= (const Matrix3&);

        friend std::ostream& operator << (std::ostream& os, const Matrix3& m);

        // Getters
        Vector3 row(int index) const;
        Vector3 col(int index) const;


        // Computations
        double det() const;

        // Class constants
        static const Matrix3 kIdentity;
        static const Matrix3 kZero;
        static const Matrix3 kOnes;

    private:
        // Ordered by rows.
        double data[9];

};

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
