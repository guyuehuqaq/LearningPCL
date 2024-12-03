#include "random_sampler.h"

#include "basic/logger.h"

#include <numeric>

#include "math/random.h"

namespace moxun {

RandomSampler::RandomSampler(const size_t num_samples)
    : num_samples_(num_samples) {}

void RandomSampler::Initialize(const size_t total_num_samples) {
  if (num_samples_ > total_num_samples) {
    AlgLogger::GetAlgLogger()->critical("[file:{} line:{} fun:{}]:{}",
                                        FILENAME,
                                        __LINE__,
                                        __FUNCTION__,
                                        "num_samples_ > total_num_samples.");
    return;
  }
  sample_idxs_.resize(total_num_samples);
  std::iota(sample_idxs_.begin(), sample_idxs_.end(), 0);
}

size_t RandomSampler::MaxNumSamples() {
  return std::numeric_limits<size_t>::max();
}

void RandomSampler::Sample(std::vector<size_t>* sampled_idxs) {
  Shuffle(static_cast<uint32_t>(num_samples_), &sample_idxs_);

  sampled_idxs->resize(num_samples_);
  for (size_t i = 0; i < num_samples_; ++i) {
    (*sampled_idxs)[i] = sample_idxs_[i];
  }
}

}  // namespace moxun
