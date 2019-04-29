/**
 * @file slam_factors.h
 * @brief User defined factors
 *
 */
#include <isam/isam.h>

#include "user_nodes.h"

// NOTE: When you add a new factor, modify gparser.cpp/h to enable proper graph loading
// Unfortunately, currently no loading from native isam is supported.

#pragma once
namespace Eigen {
    typedef Matrix<double, 1, 1> Matrix1d;
}


namespace isam {

    class Sonar3d {
      public:
        static const int dim = 3;
        static const char* name() {
            return "Sonar2d";
        }
      private:
        Pose2d m_pose;
    };

    class Plane3d_Factor : public FactorT<Plane3d> {
        Plane3d_Node* _plane;
        
    public:
        Plane3d_Factor (Plane3d_Node *plane,
                        const Plane3d& measure, const Noise& noise)
            : FactorT<Plane3d> ("Plane3d_Factor", Plane3d::dim, noise, measure),
            _plane (plane) {
                _nodes.resize(1);
                _nodes[0] = plane;
        }

        void initialize () {
            if (!_plane->initialized ()) {
                Plane3d predict (_measure);
                _plane->init (predict);
            }
        }

        Eigen::VectorXd basic_error (Selector s = ESTIMATE) const {
            const Plane3d& plane = _plane->value (s);
            Eigen::Vector3d err = plane.vector () - _measure.vector ();
            return err;
        }

      private:
    };

    class Pose3d_Plane3d_Factor : public FactorT<Plane3d> {
        Pose3d_Node* _pose;
        Plane3d_Node* _plane;
        
    public:
        Pose3d_Plane3d_Factor (Pose3d_Node *pose, Plane3d_Node *plane,
                               const Plane3d& measure, const Noise& noise)
            : FactorT<Plane3d> ("Pose3d_Plane3d_Factor", Plane3d::dim, noise, measure),
            _pose (pose), 
            _plane (plane) {
                _nodes.resize(2);
                _nodes[0] = pose;
                _nodes[1] = plane;
        }

        void initialize () {
            require(_pose->initialized (),
                    "Pose3d_Plane3d_Factor requires pose to be initialized");
            if (!_plane->initialized ()) {
                Pose3d pose = _pose->value ();
                Plane3d predict (_measure.oplus (pose));
                /* std::cout << "measure: " << std::endl; */
                /* std::cout << _measure << std::endl; */
                /* std::cout << "pose: " << std::endl; */
                /* std::cout << pose << std::endl; */
                /* std::cout << "predict: " << std::endl; */
                /* std::cout << predict << std::endl; */
                _plane->init (predict);
            }
        }

        Eigen::VectorXd basic_error (Selector s = ESTIMATE) const {
            const Pose3d& thisPose = _pose->value (s);
            const Plane3d& plane = _plane->value (s);

            Plane3d predict = plane.ominus (thisPose);
            Eigen::Vector3d err = predict.vector () - _measure.vector ();

            return err;
        }

      private:
    };

    /* Pose i and pose j observe two planar patches, k and l respectively, that are
     * coplanar */
    class Piecewise_Planar_Factor : public Factor {
        Pose3d_Node *_posei;
        Pose3d_Node *_posej;
        Plane3d_Node *_planek;
        Plane3d_Node *_planel;
      public:
        Piecewise_Planar_Factor (Pose3d_Node* posei, Pose3d_Node* posej,
                                 Plane3d_Node* planek, Plane3d_Node* planel,
                                 const Noise& noise,
                                 Anchor3d_Node *anchor1=NULL, Anchor3d_Node *anchor2=NULL)
            : Factor ("Piecewise_Planar_Factor", Plane3d::dim, noise),
            _posei (posei),
            _posej (posej),
            _planek (planek),
            _planel (planel) {
                require((anchor1==NULL && anchor2==NULL) || (anchor1!=NULL && anchor2!=NULL),
                        "slam3d: Piecewise_Planar_Factor requires either 0 or 2 anchor nodes");
                if (anchor1) {
                    _nodes.resize(6);
                    _nodes[4] = anchor1;
                    _nodes[5] = anchor2;
                } else {
                    _nodes.resize(4);
                }
                _nodes[0] = posei;
                _nodes[1] = posej;
                _nodes[2] = planek;
                _nodes[3] = planel;
            }
        
        void initialize () {
            if (_nodes.size () == 4) {
                require (_posei->initialized () && _posej->initialized () &&
                         _planek->initialized () && _planel->initialized (),
                         "Piecewise_Planar_Factor requires both poses and both planes to be initialized");
            }
            else if (_nodes.size () == 6) {
                require (_posei->initialized () && _posej->initialized () &&
                         _planek->initialized () && _planel->initialized () &&
                         _nodes[4]->initialized () && _nodes[5]->initialized (),
                         "Piecewise_Planar_Factor requires both poses, both planes, and both anchors to be initialized");
            }
        }

        Eigen::VectorXd basic_error (Selector s = ESTIMATE) const {
            Vector3d err;
            if (_nodes.size () == 4) {
                /* std::cout << std::endl; */
                const Pose3d& posei = _posei->value (s);
                const Pose3d& posej = _posej->value (s);
                const Plane3d& planek = _planek->value (s);
                const Plane3d& planel = _planel->value (s);
                
#ifdef NON_GLOBAL_PLANES
                Plane3d planeik = planek;
                Plane3d planejl = planel;
#else
                /* return planek.vector () - planel.vector (); */
                Plane3d planeik = planek.ominus (posei);
                /* Plane3d planeil = planel.ominus (posei); */
                Plane3d planejl = planel.ominus (posej);
#endif
                Pose3d poseji = posei.ominus (posej);
                
                /* std::cout << "CHECK" << std::endl; */
                Plane3d planejlPred = planeik.oplus (poseji);
                /* std::cout << "DONE" << std::endl; */

                err = planejl.vector () - planejlPred.vector ();
                /* const Pose3d& posei = _posei->value (s); */
                /* const Pose3d& posej = _posej->value (s); */
                /* const Plane3d& planek = _planek->value (s); */
                /* const Plane3d& planel = _planel->value (s); */
                
                /* /\* Plane3d planeik = planek.oplus (posei); *\/ */
                /* /\* Plane3d planeil = planel.oplus (posei); *\/ */
                /* /\* err = planeik.vector () - planeil.vector (); *\/ */

                /* /\* err = planek.vector () - planel.vector (); *\/ */

                /* Plane3d planeik = planek.ominus (posei); */
                /* Plane3d planejl = planel.ominus (posej); */

                /* /\* std::cout << "plane ik: " << std::endl; *\/ */
                /* /\* std::cout << planeik << std::endl; *\/ */

                /* /\* std::cout << "plane k: " << std::endl; *\/ */
                /* /\* std::cout << planek << std::endl; *\/ */

                /* /\* std::cout << "plane l: " << std::endl; *\/ */
                /* /\* std::cout << planel << std::endl; *\/ */

                /* /\* std::cout << "plane ik: " << std::endl; *\/ */
                /* /\* std::cout << planeik << std::endl; *\/ */

                /* /\* std::cout << "plane jl: " << std::endl; *\/ */
                /* /\* std::cout << planejl << std::endl; *\/ */

                /* Pose3d poseij = posej.ominus (posei); */
                /* Plane3d planejlPred = planeik.ominus (poseij); */
                /* err = planejlPred.vector () - planejl.vector (); */

                /* /\* std::cout << "plane jl pred: " << std::endl; *\/ */
                /* /\* std::cout << planejlPred << std::endl; *\/ */

                /* std::cout << std::endl; */
                /* std::cout << posei << std::endl; */
                /* std::cout << posej << std::endl; */
                /* std::cout << poseij << std::endl; */
                /* std::cout << planejl << std::endl; */
                /* std::cout << planelPred << std::endl; */
                /* std::cout << err << std::endl; */
                /* std::cout << "------------------------------" << std::endl; */
            }
            else if (_nodes.size () == 6) {
                /* std::cout << std::endl; */
                const Pose3d& a1 = dynamic_cast<Pose3d_Node*>(_nodes[4])->value(s);
                const Pose3d& a2 = dynamic_cast<Pose3d_Node*>(_nodes[5])->value(s);
                Pose3d a1Inv = Pose3d (a1.oTw ());
                Pose3d a2Inv = Pose3d (a2.oTw ());
                const Pose3d& posei = _posei->value (s);
                const Pose3d& posej = _posej->value (s);
                const Plane3d& planek = _planek->value (s);
                const Plane3d& planel = _planel->value (s);
                
#ifdef GLOBAL_PLANES
                Plane3d planeik = planek.oplus (a1);
                Plane3d planejl = planel.oplus (a2);
#else
                /* return planek.vector () - planel.vector (); */
                Plane3d planeik = planek.ominus (a1Inv).ominus (a1.oplus (posei));
                /* Plane3d planeil = planel.ominus (posei); */
                Plane3d planejl = planel.ominus (a2Inv).ominus (a2.oplus (posej));
#endif
                Pose3d poseji = a1.oplus (posei).ominus (a2.oplus (posej));
                
                /* std::cout << "CHECK" << std::endl; */
                Plane3d planejlPred = planeik.oplus (poseji);
                /* std::cout << "DONE" << std::endl; */

                err = planejl.vector () - planejlPred.vector ();
            }
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " " << " " << noise_to_string(_noise);
        }
    };


// A 2d constraint between sonar frames
    class Sonar2d_Factor : public FactorT<Pose2d> {
      const Pose3d_Node* _pose1;
      const Pose3d_Node* _pose2;
      Pose3d _sonar_frame1;
      Pose3d _sonar_frame2;
    public:

      /**
       * Constructor.
       * @param pose1 The pose from which the measurement starts.
       * @param pose2 The pose to which the measurement extends.
       * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
       * @param noise The 3x3 square root information matrix (upper triangular).
       * @sonar_frame1 the transformation from pose1 to the sonar frame.
       * @sonar_frame2 the transfomration from pose2 to the sonar frame.
       */
      Sonar2d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2,
                     const Pose2d& measure, const Noise& noise,
                     Pose3d sonar_frame1, Pose3d sonar_frame2)
        : FactorT<Pose2d>("Sonar2d_Factor", 3, noise, measure), 
          _pose1(pose1), 
          _pose2(pose2),
          _sonar_frame1(sonar_frame1),
          _sonar_frame2(sonar_frame2) 
      {
        _nodes.resize(2);
        _nodes[0] = pose1;
        _nodes[1] = pose2;
      }

      void initialize() {
      }

      Eigen::VectorXd basic_error(Selector s = LINPOINT) const {

        // @todo  add predicted sonar transformation
        // @todo  add sonar transformation from vehicle body

        const Pose3d& p1 = _pose1->value(s);
        const Pose3d& p2 = _pose2->value(s);
        Pose3d predicted = (p2.oplus(_sonar_frame2)).ominus(p1.oplus(_sonar_frame1));

        Eigen::VectorXd p(3);
        p << predicted.x(), predicted.y(), predicted.yaw(); // Predicted
        Eigen::VectorXd err(3);
        err << p(0)-_measure.x(), p(1)-_measure.y(), p(2)-_measure.t();
        err(2) = standardRad(err(2));

        return err;
      }

      void write(std::ostream &out) const {
        Factor::write(out);
        out << " " << _measure << " " << noise_to_string(_noise) << " " << _sonar_frame1  << " " << _sonar_frame2;
      }

    };

// xyz factor
    class Pose3d_xyz_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector3d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 3x3 square root information matrix (upper triangular).
         */
      Pose3d_xyz_Factor(Pose3d_Node* pose, const Eigen::Vector3d& pose_partial, const Noise& noise)
          : Factor("Pose3d_xyz_Factor", 3, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(3);
            err << pose.x() - _pose_partial(0), pose.y() - _pose_partial(1), pose.z() - _pose_partial(2);
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ", "<< _pose_partial(1) << ", " << _pose_partial(2) << ") " << noise_to_string(_noise);
        }

    };
// xyh factor
    class Pose3d_xyh_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector3d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 3x3 square root information matrix (upper triangular).
         */
      Pose3d_xyh_Factor(Pose3d_Node* pose, const Eigen::Vector3d& pose_partial, const Noise& noise)
          : Factor("Pose3d_xyh_Factor", 3, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(3);
            err << pose.x() - _pose_partial(0), pose.y() - _pose_partial(1), pose.yaw() - _pose_partial(2);
            err(2) = standardRad(err(2));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ", "<< _pose_partial(1) << ", " << _pose_partial(2) << ") " << noise_to_string(_noise);
        }

    };


// h factor
    class Pose3d_h_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Matrix1d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 1x1 square root information matrix (upper triangular).
         */
      Pose3d_h_Factor(Pose3d_Node* pose, const Eigen::Matrix1d& pose_partial, const Noise& noise)
          : Factor("Pose3d_h_Factor", 1, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(1);
            err << pose.yaw() - _pose_partial(0);
            err(0) = standardRad(err(0));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ") " << noise_to_string(_noise);
        }

    };

// z factor
    class Pose3d_z_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Matrix1d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 1x1 square root information matrix (upper triangular).
         */
      Pose3d_z_Factor(Pose3d_Node* pose, const Eigen::Matrix1d& pose_partial, const Noise& noise)
          : Factor("Pose3d_z_Factor", 1, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(1);
            err << pose.z() - _pose_partial(0);
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ") " << noise_to_string(_noise);
        }

    };




// rp factor
    class Pose3d_rp_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector2d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 2x2 square root information matrix (upper triangular).
         */
      Pose3d_rp_Factor(Pose3d_Node* pose, const Eigen::Vector2d& pose_partial, const Noise& noise)
          : Factor("Pose3d_rp_Factor", 2, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(2);
            err << pose.roll() - _pose_partial(0), pose.pitch() - _pose_partial(1);
            err(0) = standardRad(err(0));
            err(1) = standardRad(err(1));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ", "<< _pose_partial(1) << ") " << noise_to_string(_noise);
        }

    };

// xy factor
    class Pose3d_xy_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector2d _pose_partial;
        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 2x2 square root information matrix (upper triangular).
         */
      Pose3d_xy_Factor(Pose3d_Node* pose, const Eigen::Vector2d& pose_partial, const Noise& noise)
          : Factor("Pose3d_xy_Factor", 2, noise), _pose(pose), _pose_partial(pose_partial)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(2);
            err << pose.x() - _pose_partial(0), pose.y() - _pose_partial(1);
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ", "<< _pose_partial(1) << ") " << noise_to_string(_noise);
        }

    };


    class Pose3d_MaxMix_xy_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector2d _pose_partial;
        Noise _noise1;
        Noise _noise2;
        double _w1;
        double _w2;

        Eigen::Matrix2d _L1;
        Eigen::Matrix2d _L2;
        double _c1;
        double _c2;


        /**
         * Constructor.
         * @param pose The pose node the pose_partial acts on.
         * @param pose_partial The actual pose_partial measurement.
         * @param sqrtinf The 2x2 square root information matrix (upper triangular).
         */
      Pose3d_MaxMix_xy_Factor(Pose3d_Node* pose, const Eigen::Vector2d& pose_partial, const Noise& noise1, const Noise& noise2, double w1, double w2)
          : Factor("Pose3d_MaxMix_xy_Factor", 2, Information(Eigen::Matrix2d::Identity())),
            _pose(pose), _pose_partial(pose_partial), _noise1(noise1), _noise2(noise2), _w1(w1), _w2(w2)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
            _L1 = _noise1.sqrtinf().transpose() * _noise1.sqrtinf();
            _L2 = _noise1.sqrtinf().transpose() * _noise2.sqrtinf();
            _c1 = _w1/(2.0*M_PI*sqrt(1.0/_L1.determinant()));
            _c2 = _w2/(2.0*M_PI*sqrt(1.0/_L2.determinant()));
        }

        void initialize() {
            // Partial pose_partial is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(2);
            err << pose.x() - _pose_partial(0), pose.y() - _pose_partial(1);
            
            double d1 = (err.transpose()*_L1*err);
            double p1 = _c1 * exp (-0.5*d1);
            double d2 = (err.transpose()*_L2*err);
            double p2 = _c2 * exp (-0.5*d2);

            if (p1 > p2) {
                return _noise1.sqrtinf() * err;
            } else {
                return _noise2.sqrtinf() * err;
            }
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _pose_partial(0) << ", "<< _pose_partial(1) << ") ";
            out << _w1 << " " << noise_to_string(_noise1);
            out << _w2 << " " << noise_to_string(_noise1);
        }

    };

    /**
    * This class is for the plane measurement
    * @author Hyunchul Roh (rohs_@kaist.ac.kr)
    */

    class Wall3d_Wall3d_Factor : public FactorT<Point3d> {
        Pose3d_Node* _pose1;
        Pose3d_Node* _pose2;

    public:

		Wall3d_Wall3d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2, const Point3d& measure, const Noise& noise, Wall3d *wall1, Wall3d *wall2,
            Anchor3d_Node *anchor1 = NULL, Anchor3d_Node *anchor2 = NULL)
			: FactorT<Point3d>("Wall3d_Wall3d_Factor", 3, noise, measure), _pose1(pose1), _pose2(pose2) {
                require((wall1 == NULL && wall2 == NULL) || (wall1 != NULL && wall2 != NULL),
                    "slam3d: Wall3d_Wall3d_Factor requires either 0 or 2 wall");
                require((anchor1 == NULL && anchor2 == NULL) || (anchor1 != NULL && anchor2 != NULL),
                    "slam3d: Wall3d_Wall3d_Factor requires either 0 or 2 anchor nodes");
                if (anchor1) {
                    _nodes.resize(4);
                    _nodes[2] = anchor1;
                    _nodes[3] = anchor2;
                }
                else {
                    _nodes.resize(2);
                }
                _nodes[0] = pose1;
                _nodes[1] = pose2;

                _wall1 = wall1;
                _wall2 = wall2;

            }

        void initialize() {
            if (_nodes.size() == 2) {
                require(_nodes[0]->initialized() && _nodes[1]->initialized(), "Plane3d_Plane3d_Factor: both nodes have to be initialized");
            }
            else if (_nodes.size() == 4) {
                require(_nodes[0]->initialized() && _nodes[1]->initialized() && _nodes[2]->initialized() && _nodes[3]->initialized(), "Plane3d_Plane3d_Factor: both nodes and both anchors have to be initialized");
            }
        }

        Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {

            const Pose3d& p1 = _pose1->value(s);
            const Pose3d& p2 = _pose2->value(s);

            Pose3d p21;
            p21 = p1.ominus(p2);
            Plane3d plane1 = _wall1->normal();
            Plane3d plane2 = _wall2->normal();

            Plane3d plane2_cvt_to_plane1 = plane2.ominus(p21);

            double dot_product = (plane2_cvt_to_plane1.x()*plane1.x() + plane2_cvt_to_plane1.y()*plane1.y() + plane2_cvt_to_plane1.z()*plane1.z()) / (plane2_cvt_to_plane1.d()*plane1.d());
            double err_angle = 1.0 - fabs(dot_product);

            Point3d wall2_cvt_to_wall1 = _wall2->ptxfer(p21);

            //double err_d1 = (plane1.x() * wall2_cvt_to_wall1.x() + plane1.y() * wall2_cvt_to_wall1.y() + plane1.z() * wall2_cvt_to_wall1.z() + _wall1->d()) / sqrt(plane1.x()*plane1.x() + plane1.y()*plane1.y() + plane1.z()*plane1.z());

            double tmpd = -(plane2_cvt_to_plane1.x() * wall2_cvt_to_wall1.x() + plane2_cvt_to_plane1.y() * wall2_cvt_to_wall1.y() + plane2_cvt_to_plane1.z() * wall2_cvt_to_wall1.z());
            double err_d2 = (plane2_cvt_to_plane1.x() * _wall1->cx() + plane2_cvt_to_plane1.y() * _wall1->cy() + plane2_cvt_to_plane1.z() * _wall1->cz() + tmpd) / sqrt(plane2_cvt_to_plane1.x()*plane2_cvt_to_plane1.x() + plane2_cvt_to_plane1.y()*plane2_cvt_to_plane1.y() + plane2_cvt_to_plane1.z()*plane2_cvt_to_plane1.z());

            //printf("%2.3f %2.3f %2.3f\n", err_angle, err_d1, err_d2);

            Eigen::VectorXd err(3);// = predicted_plane.vector() - _measure.vector();
            err(0) = standardRad(err_angle);
            err(1) = 0;// abs(err_d1);
            err(2) = fabs(err_d2);

            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " " << _measure << " " << noise_to_string(_noise);
        }

    private:
        Wall3d* _wall1;
        Wall3d* _wall2;

    };

} // namespace isam
