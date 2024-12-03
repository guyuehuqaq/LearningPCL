#pragma once

#include "basic/logger.h"

#include <cstddef>
#include <vector>

namespace moxun {

// Abstract base class for sampling methods.
class Sampler {
 public:
  Sampler() = default;
  explicit Sampler(size_t num_samples);
  virtual ~Sampler() = default;

  // Initialize the sampler, before calling the `Sample` method.
  virtual void Initialize(size_t total_num_samples) = 0;

  // Maximum number of unique samples that can be generated.
  virtual size_t MaxNumSamples() = 0;

  // Sample `num_samples` elements from all samples.
  virtual void Sample(std::vector<size_t>* sampled_idxs) = 0;

  // Sample elements from `X` into `X_rand`.
  //
  // Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
  // should equal `num_samples`.
  template <typename X_t>
  void SampleX(const X_t& X, X_t* X_rand);

  // Sample elements from `X` and `Y` into `X_rand` and `Y_rand`.
  //
  // Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
  // should equal `num_samples`. The same applies for `Y` and `Y_rand`.
  template <typename X_t, typename Y_t>
  void SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand);
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename X_t>
void Sampler::SampleX(const X_t& X, X_t* X_rand) {
  thread_local std::vector<size_t> sampled_idxs;
  Sample(&sampled_idxs);
  for (size_t i = 0; i < X_rand->size(); ++i) {
    (*X_rand)[i] = X[sampled_idxs[i]];
  }
}

template <typename X_t, typename Y_t>
void Sampler::SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand) {
  if (X.size() != Y.size()) {
    AlgLogger::GetAlgLogger()->critical("[file:{} line:{} fun:{}]:{}",
                                        FILENAME,
                                        __LINE__,
                                        __FUNCTION__,
                                        " X.size()!=Y.size().");
    exit(1);
  }
  if (X_rand->size() != Y_rand->size()) {
    AlgLogger::GetAlgLogger()->critical("[file:{} line:{} fun:{}]:{}",
                                        FILENAME,
                                        __LINE__,
                                        __FUNCTION__,
                                        "X_rand->size()!=Y_rand->size().");
    exit(1);
  }
  thread_local std::vector<size_t> sampled_idxs;
  Sample(&sampled_idxs);
  for (size_t i = 0; i < X_rand->size(); ++i) {
    (*X_rand)[i] = X[sampled_idxs[i]];
    (*Y_rand)[i] = Y[sampled_idxs[i]];
  }
}

}  // namespace moxun
