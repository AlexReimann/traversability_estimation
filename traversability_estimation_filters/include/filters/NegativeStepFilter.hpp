/*
 * StepFilter.hpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef NEGATIVESTEPFILTER_HPP
#define NEGATIVESTEPFILTER_HPP

#include <filters/filter_base.h>

#include <string>

namespace filters {

/*!
 * Step Filter class to compute the step traversability value of an elevation map.
 */
template<typename T>
class NegativeStepFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  NegativeStepFilter();

  /*!
   * Destructor.
   */
  virtual ~NegativeStepFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the step traversability value based on an elevation map and
   * saves it as additional grid map layer.
   * The step traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and step traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed step depth.
  double max_allowed_step_depth_;

  //! Window sizes for step filter
  //double firstWindowRadius_, secondWindowRadius_;

  //! Distance in cells looking for negative steps
  int sample_distance_cells_;

  //! Critical number of cells greater than maximums allowed step.
  int nCellCritical_;

  //! Step map type.
  std::string type_;
};

} /* namespace */

#endif
