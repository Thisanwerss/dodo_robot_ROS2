#ifndef DODO_RL_PPO_MODEL_HPP
#define DODO_RL_PPO_MODEL_HPP

#include <string>
#include <vector>

namespace dodo_rl
{

class PPOModel
{
public:
  PPOModel(const std::string & model_path);
  virtual ~PPOModel();

  // Get action from observation
  std::vector<double> getAction(const std::vector<double> & observation);

  // Load model from file
  bool loadModel(const std::string & model_path);

private:
  // Model parameters (placeholder for actual implementation)
  std::string model_path_;
  bool is_loaded_;
};

}  // namespace dodo_rl

#endif  // DODO_RL_PPO_MODEL_HPP