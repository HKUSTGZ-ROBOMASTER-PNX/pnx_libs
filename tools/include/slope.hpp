#pragma once

namespace tools {

class Slope {
 public:
  Slope() = default;
  Slope(float default_value, float path);

  void set_default(float value);
  void set_path(float path);
  void set_increase_path(float path);
  void set_decrease_path(float path);

  float value() const;
  bool reached() const;
  float update_value(float new_value);

 private:
  float value_ = 0.0f;
  float increase_path_ = 0.0f;
  float decrease_path_ = 0.0f;
  bool reached_ = false;
};

}  // namespace tools
