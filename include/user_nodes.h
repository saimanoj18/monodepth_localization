/**
 * @file user_nodes.h
 * @brief User defined nodes
 *
 */
#include <isam/isam.h>

/* NOTE: When you add a new node, modify gparser.cpp/h to enable proper graph loading */
/* Unfortunately, currently no loading from native isam is supported. */

#include "lib_irp_math/dm.h"

#pragma once
typedef Eigen::Matrix< double , 9 , 1> Vector9d;
typedef Eigen::Matrix< double , 6 , 1> Vector6d;
typedef Eigen::Matrix< double , 5 , 1> Vector5d;
typedef Eigen::Matrix< double , 4 , 1> Vector4d;
typedef Eigen::Matrix< double , 3 , 1> Vector3d;
typedef Eigen::Matrix< double , 2 , 1> Vector2d;

namespace isam {

    class Plane3d {
        friend std::ostream& operator<< (std::ostream& out, const Plane3d& s) {
            s.write (out);
            return out;
        }

        double _x, _y, _z;
        double _d;                  /* distance to plane - can be determined from x, y, z*/
      public:
        static const int dim = 3;
        static const char* name() {
            return "Plane3d";
        }

        Plane3d () {}
        Plane3d (double nx, double ny, double nz) : _x (nx), _y (ny), _z (nz) {
            _d = sqrt (nx*nx + ny*ny + nz*nz);
        }
        Plane3d (const Eigen::VectorXd& vec) : _x (vec (0)), _y (vec (1)), _z (vec (2)) {
            _d = sqrt (vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
        }

        double x () const {return _x;}
        double y () const {return _y;}
        double z () const {return _z;}
        double d () const {return _d;}

        Plane3d exmap(const Vector3d& delta) {
            Plane3d res = *this;
            res._x += delta (0);
            res._y += delta (1);
            res._z += delta (2);
            return res;
        }

        Eigen::VectorXd vector () const {
            Eigen::VectorXd tmp (Plane3d::dim);
            tmp << x (), y (), z ();
            return tmp;
        }

        void set (double nx, double ny, double nz) {
            _x = nx;
            _y = ny;
            _z = nz;
            _d = sqrt (nx*nx + ny*ny + nz*nz);
        }
        void set (const Vector3d& v) {
            _x = v (0);
            _y = v (1);
            _z = v (2);
            _d = sqrt (_x*_x + _y*_y + _z*_z);
        }

        /* given a global vehicle pose and local plane measurement in that vehicle's
         * frame, predict the plane the global frame */
        Plane3d oplus (const Pose3d& pose) const {
            Pose3d poseInv = Pose3d (pose.oTw ());
            return ominus (poseInv);
        }

        /* given a vehicle pose and plane (both in global frame), compute the plane in
         * vehicle's frame */
        Plane3d ominus (const Pose3d& pose) const {
            Vector3d n, nUnit, nPoseFrame;
            n (0) = x (); n (1) = y (); n (2) = z ();
            double d = n.norm ();
            nUnit = n / d;
            Eigen::Matrix4d H = pose.wTo ();
            Eigen::Vector3d t = H.col(3).head(3);
            Eigen::Matrix3d R = H.topLeftCorner(3,3).transpose ();
            nPoseFrame = R * nUnit;
            double dPoseFrame = t.dot (nUnit) + d;
            /* std::cout << wRo << std::endl; */
            /* std::cout << nPoseFrame << std::endl; */
            /* std::cout << dPoseFrame << std::endl; */
            nPoseFrame *= dPoseFrame;
            return Plane3d (nPoseFrame);
        }

        Eigen::VectorXd root_shift (Eigen::VectorXd x_w_r) {
            Pose3d X_w_r(x_w_r);
            return ominus(X_w_r).vector();
        }

        Eigen::VectorXb is_angle() const {
            Eigen::VectorXb isang (dim);
            isang << false, false, false;
            return isang;
        }

        void write (std::ostream &out) const {
            out << "(" << x () << ", " << y () << ", " << z () << ")";
        }

    }; /* class Plane3d */

    typedef NodeT<Plane3d> Plane3d_Node;

    class Wall3d { // added by rohs 2015.12.24
        friend std::ostream& operator<<(std::ostream& out, const Wall3d& p) {
            p.write(out);
            return out;
        }

        Plane3d _p; //3
        double _d;
        Point3d _c; //3
        double _w; //1
        double _h; //1

    public:
        static const int dim = 9;
        static const char* name() {
            return "Plane3d";
        }
        Wall3d() : _p(0., 0., 0.), _d(0.), _c(0., 0., 0.), _w(0.), _h(0.) {}
        Wall3d(double nx, double ny, double nz, double d, double cx, double cy, double cz, double w, double h) : _p(nx, ny, nz), _d(d), _c(cx, cy, cz), _w(w), _h(h) {}
        Wall3d(const Eigen::VectorXd& vec) : _d(vec(3)), _w(vec(7)), _h(vec(8)) { _p = Plane3d(vec(0), vec(1), vec(2)); _c = Point3d(vec(4), vec(5), vec(6)); }
        double nx() const { return _p.x(); }
        double ny() const { return _p.y(); }
        double nz() const { return _p.z(); }
        double cx() const { return _c.x(); }
        double cy() const { return _c.y(); }
        double cz() const { return _c.z(); }
        double d()  const { return _d; }


        Plane3d normal() const { return _p; }
        Point3d center() const { return _c; }

        Eigen::VectorXd vector() const {
            Eigen::VectorXd tmp(9);
            tmp << nx(), ny(), nz(), _d, cx(), cy(), cz(), _w, _h;
            return tmp;
        }

        void set(double nx, double ny, double nz, double d, double cx, double cy, double cz, double w, double h) {
            _p = Plane3d(nx, ny, nz);
            _d = d;
            _c = Point3d(cx, cy, cz);
            _w = w;
            _h = h;
        }

        void set(const Eigen::VectorXd& v) {
            _p = Plane3d(v(0), v(1), v(2));
            _d = v(3);
            _c = Point3d(v(4), v(5), v(6));
            _w = v(7);
            _h = v(8);
        }


        Point3d ptxfer(const Pose3d& p12) const {
            Vector3d ctr1, ctr2;

            Eigen::Matrix4d H12 = p12.wTo();
            Eigen::Vector3d t12 = H12.col(3).head(3);
            Eigen::Matrix3d R12 = H12.topLeftCorner(3, 3).transpose();

            ctr1 = _c.vector() - t12;
            
            ctr2 = R12*ctr1;

            return Point3d(ctr2(0), ctr2(1), ctr2(2));
        }


        void write(std::ostream &out) const {

            out << "(" << nx() << ", " << ny() << ", " << nz() << ", " << d() << ", " << cx() << ", " << cy() << ", " << cz() << ", " << _w << ", " << _h << ")";
        }
    };

} /* namespace isam */
