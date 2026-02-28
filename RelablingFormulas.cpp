#include <string>
#include <iostream>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include <spot/tl/relabel.hh>

int main()
{
  std::string input = "\"Proc@Here\" U (\"var > 10\" | \"var < 4\")";
  spot::parsed_formula pf = spot::parse_infix_psl(input);
  if (pf.format_errors(std::cerr))
    return 1;
  spot::formula f = pf.f;
  spot::relabeling_map m;
  f = spot::relabel(f, spot::Pnn, &m);
  for (auto& i: m)
    {
      std::cout << "#define " << i.first << " (";
      print_spin_ltl(std::cout, i.second, true) << ")\n";
    }
  print_spin_ltl(std::cout, f, true) << '\n';
  return 0;
}