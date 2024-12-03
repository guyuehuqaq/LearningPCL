#include "ransac/random_sampler.h"

#include <gtest/gtest.h>

#include <unordered_set>

namespace moxun {
namespace {

TEST(RandomSampler, LessSamples) {
  RandomSampler sampler(2);
  sampler.Initialize(5);
  EXPECT_EQ(sampler.MaxNumSamples(), std::numeric_limits<size_t>::max());
  for (size_t i = 0; i < 100; ++i) {
    std::vector<size_t> samples;
    sampler.Sample(&samples);
    EXPECT_EQ(samples.size(), 2);
    EXPECT_EQ(std::unordered_set<size_t>(samples.begin(), samples.end()).size(),
              2);
  }
}

TEST(RandomSampler, EqualSamples) {
  RandomSampler sampler(5);
  sampler.Initialize(5);
  EXPECT_EQ(sampler.MaxNumSamples(), std::numeric_limits<size_t>::max());
  for (size_t i = 0; i < 100; ++i) {
    std::vector<size_t> samples;
    sampler.Sample(&samples);
    EXPECT_EQ(samples.size(), 5);
    EXPECT_EQ(std::unordered_set<size_t>(samples.begin(), samples.end()).size(),
              5);
  }
}

}  // namespace
}  // namespace moxun
