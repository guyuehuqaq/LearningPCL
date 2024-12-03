#include "ransac/support_measurement.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "math/math.h"

namespace moxun {
namespace {

TEST(InlierSupportMeasurer, Nominal) {
  InlierSupportMeasurer::Support support1;
  EXPECT_EQ(support1.num_inliers, 0);
  EXPECT_EQ(support1.residual_sum, std::numeric_limits<double>::max());
  InlierSupportMeasurer measurer;
  std::vector<double> residuals = {-1.0, 0.0, 1.0, 2.0};
  support1 = measurer.Evaluate(residuals, 1.0);
  EXPECT_EQ(support1.num_inliers, 3);
  EXPECT_EQ(support1.residual_sum, 0.0);
  InlierSupportMeasurer::Support support2;
  support2.num_inliers = 2;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum = support1.residual_sum;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.num_inliers = support1.num_inliers;
  support2.residual_sum += 0.01;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum -= 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum -= 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_TRUE(measurer.Compare(support2, support1));
}

TEST(UniqueInlierSupportMeasurer, Nominal) {
  UniqueInlierSupportMeasurer::Support support1;
  EXPECT_EQ(support1.num_inliers, 0);
  EXPECT_EQ(support1.num_unique_inliers, 0);
  EXPECT_EQ(support1.residual_sum, std::numeric_limits<double>::max());

  UniqueInlierSupportMeasurer measurer;
  const std::vector<size_t> sample_ids = {1, 2, 2, 3};
  measurer.SetUniqueSampleIds(sample_ids);
  const std::vector<double> residuals = {-1.0, 0.0, 1.0, 2.0};
  support1 = measurer.Evaluate(residuals, 1.0);
  EXPECT_EQ(support1.num_inliers, 3);
  EXPECT_EQ(support1.num_unique_inliers, 2);
  EXPECT_EQ(support1.residual_sum, 0.0);

  UniqueInlierSupportMeasurer::Support support2;
  support2.num_unique_inliers = support1.num_unique_inliers - 1;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.num_inliers = support1.num_inliers + 1;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.num_inliers = support1.num_inliers;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum = support1.residual_sum - 0.01;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum = support1.residual_sum;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.num_unique_inliers = support1.num_unique_inliers;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.residual_sum = support1.residual_sum - 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_TRUE(measurer.Compare(support2, support1));
  support2.num_inliers = support1.num_inliers + 1;
  support2.residual_sum = support1.residual_sum + 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_TRUE(measurer.Compare(support2, support1));
  support2.num_unique_inliers = support1.num_unique_inliers + 1;
  support2.num_inliers = support1.num_inliers - 1;
  support2.residual_sum = support1.residual_sum + 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_TRUE(measurer.Compare(support2, support1));
}

TEST(MEstimatorSupportMeasurer, Nominal) {
  MEstimatorSupportMeasurer::Support support1;
  EXPECT_EQ(support1.num_inliers, 0);
  EXPECT_EQ(support1.score, std::numeric_limits<double>::max());
  MEstimatorSupportMeasurer measurer;
  std::vector<double> residuals = {-1.0, 0.0, 1.0, 2.0};
  support1 = measurer.Evaluate(residuals, 1.0);
  EXPECT_EQ(support1.num_inliers, 3);
  EXPECT_EQ(support1.score, 1.0);
  MEstimatorSupportMeasurer::Support support2 = support1;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.num_inliers -= 1;
  support2.score += 0.01;
  EXPECT_TRUE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.score -= 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_FALSE(measurer.Compare(support2, support1));
  support2.score -= 0.01;
  EXPECT_FALSE(measurer.Compare(support1, support2));
  EXPECT_TRUE(measurer.Compare(support2, support1));
}

}  // namespace
}  // namespace moxun
