// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#pragma once

// GTSAM headers
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>

// Boost headers
#include <boost/concept/assert.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/serialization.hpp> // Ensure the serialization core is included

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenFactorWithAnchoring: public NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    BOOST_CONCEPT_ASSERT((IsTestable<VALUE>));
    BOOST_CONCEPT_ASSERT((IsLieGroup<VALUE>));

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactorWithAnchoring<VALUE> This;
    typedef NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenFactorWithAnchoring> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenFactorWithAnchoring() {}

    /** Constructor */
    BetweenFactorWithAnchoring(
          // 1 for robot 1, and 2 for robot 2
          Key key1, Key key2, Key anchor_key1, Key anchor_key2,
          const VALUE& measured,
          const SharedNoiseModel& model = nullptr) :
      Base(model, key1, key2, anchor_key1, anchor_key2), measured_(measured) {
    }

    virtual ~BetweenFactorWithAnchoring() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenFactorWithAnchoring("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */




    // some useful link, giseop 
    // line 384, https://gtsam.org/doxygen/a00317_source.html
    // https://gtsam.org/doxygen/a02091.html
    // betweenfactor https://gtsam.org/doxygen/a00935_source.html
    // line 224 https://gtsam.org/doxygen/a00053_source.html
    // isam ver. line 233, https://people.csail.mit.edu/kaess/isam/doc/slam2d_8h_source.html



/** vector of errors */
gtsam::Vector evaluateError(
        const gtsam::Pose3& p1,
        const gtsam::Pose3& p2,
        const gtsam::Pose3& anchor_p1,
        const gtsam::Pose3& anchor_p2,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> anchor_H1 = boost::none,
        boost::optional<gtsam::Matrix&> anchor_H2 = boost::none
) const override {

    // Step 1: Compute the composed poses with anchor nodes
    VALUE hx1 = traits<VALUE>::Compose(anchor_p1, p1, anchor_H1, H1); 
    VALUE hx2 = traits<VALUE>::Compose(anchor_p2, p2, anchor_H2, H2); 

    // Step 2: Compute the relative pose between the two composed poses
    Matrix H_between_1, H_between_2;
    VALUE hx = traits<VALUE>::Between(hx1, hx2, H_between_1, H_between_2);

    // Step 3: Update the Jacobians using the chain rule (if provided)
    if (H1) {
        *H1 = H_between_1 * (*H1); // chain rule: d(hx)/d(p1) = d(Between)/d(hx1) * d(Compose)/d(p1)
    }
    if (H2) {
        *H2 = H_between_2 * (*H2); // chain rule: d(hx)/d(p2) = d(Between)/d(hx2) * d(Compose)/d(p2)
    }
    if (anchor_H1) {
        *anchor_H1 = H_between_1 * (*anchor_H1); // chain rule for anchor Jacobians
    }
    if (anchor_H2) {
        *anchor_H2 = H_between_2 * (*anchor_H2);
    }

    // Step 4: Compute and return the local coordinate error between the measured pose and the computed relative pose
    return traits<VALUE>::Local(measured_, hx);
}





    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 4;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor4",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }

    

	//   // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
	//   enum { NeedsToAlign = (sizeof(VALUE) % 16) == 0 };
  //   public:
  //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; // \class BetweenFactorWithAnchoring


  // traits
  template<class VALUE>
  struct traits<BetweenFactorWithAnchoring<VALUE> > : public Testable<BetweenFactorWithAnchoring<VALUE> > {};

} /// namespace gtsam
