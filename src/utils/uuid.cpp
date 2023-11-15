#include "uuid.hpp"

std::string generate_uuid_v4() {
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> uni(0, 15);
  std::uniform_int_distribution<int> uni8(8, 11);

  std::stringstream ss;
  ss << std::hex;
  for (int i = 0; i < 8; i++) ss << uni(rng);
  ss << "-";
  for (int i = 0; i < 4; i++) ss << uni(rng);
  ss << "-4"; // The 4 indicates the UUID version
  for (int i = 0; i < 3; i++) ss << uni(rng);
  ss << "-";
  ss << uni8(rng); // The variant
  for (int i = 0; i < 3; i++) ss << uni(rng);
  ss << "-";
  for (int i = 0; i < 12; i++) ss << uni(rng);

  return ss.str();
}