#include <string>
#include <iostream>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>

int main()
{
  std::string input = "& & G p0 p1 p2";
  spot::parsed_formula pf = spot::parse_prefix_ltl(input);
  if (pf.format_errors(std::cerr))
    return 1;
  spot::formula f = pf.f;
  print_latex_psl(std::cout, f) << '\n';
  print_lbt_ltl(std::cout, f) << '\n';
  print_spin_ltl(std::cout, f, true) << '\n';
  return 0;
}