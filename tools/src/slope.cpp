#include "slope.hpp"

namespace tools {

Slope::Slope(float default_value, float path)
    : value_(default_value), increase_path_(path), decrease_path_(path) {}

void Slope::set_default(float value) { value_ = value; }

void Slope::set_path(float path) {
  increase_path_ = path;
  decrease_path_ = path;
}

void Slope::set_increase_path(float path) { increase_path_ = path; }

void Slope::set_decrease_path(float path) { decrease_path_ = path; }

float Slope::value() const { return value_; }

bool Slope::reached() const { return reached_; }

float Slope::update_value(float new_value) {
  const float delta = new_value - value_;
  if ((delta >= 0.0f && delta < increase_path_) || (delta < 0.0f && delta > -decrease_path_)) {
    value_ = new_value;
    reached_ = true;
  } else {
    reached_ = false;
    if (new_value < value_) {
      value_ -= decrease_path_;
    } else {
      value_ += increase_path_;
    }
  }
  return value_;
}

}  // namespace tools
