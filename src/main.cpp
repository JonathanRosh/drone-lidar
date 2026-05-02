#include <iostream>
#include <mp-units/systems/si/unit_symbols.h>

using mp_units::si::unit_symbols::deg;

int main() {
  auto a = 90.0 * deg;

  std::cout << a << "\n";
}