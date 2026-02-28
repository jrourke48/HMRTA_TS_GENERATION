#include <string>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <utility>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/product.hh>
#include <spot/twa/twagraph.hh>
#include <bddx.h>

class SpotTransitionSystem {
public:
  SpotTransitionSystem(int w, int h) : width_(w), height_(h) {}

  int numStates() const { return width_ * height_; }

  int toId(int x, int y) const { return y * width_ + x; }

  std::pair<int, int> fromId(int id) const {
    return {id % width_, id / width_};
  }

  std::vector<int> initialStates() const {
    return {toId(0, 0)};
  }

  std::vector<int> successors(int stateId) const {
    std::vector<int> out;
    auto [x, y] = fromId(stateId);

    const int dx[5] = {0, 0, -1, 1, 0};
    const int dy[5] = {1, -1, 0, 0, 0}; // UP, DOWN, LEFT, RIGHT, WAIT

    for (int i = 0; i < 5; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
        out.push_back(toId(nx, ny));
      }
    }
    return out;
  }

  std::unordered_set<std::string> label(int stateId) const {
    auto [x, y] = fromId(stateId);
    std::unordered_set<std::string> aps;

    if ((x == 1 && y == 1) || (x == 2 && y == 2)) {
      aps.insert("c");
    }
    if (x == 3 && y == 3) {
      aps.insert("d");
    }
    return aps;
  }

private:
  int width_;
  int height_;
};

static bdd labelToBdd(const std::unordered_set<std::string>& labels,
                      int cVar,
                      int dVar) {
  bdd cond = bddtrue;
  cond &= (labels.count("c") > 0) ? bdd_ithvar(cVar) : bdd_nithvar(cVar);
  cond &= (labels.count("d") > 0) ? bdd_ithvar(dVar) : bdd_nithvar(dVar);
  return cond;
}

static spot::twa_graph_ptr buildTsAutomaton(const SpotTransitionSystem& ts,
                                            const spot::twa_graph_ptr& ba) {
  auto tsAut = spot::make_twa_graph(ba->get_dict());

  int cVar = tsAut->register_ap("c");
  int dVar = tsAut->register_ap("d");

  tsAut->set_acceptance(0, spot::acc_cond::acc_code::t());
  tsAut->new_states(static_cast<unsigned>(ts.numStates()));

  auto init = ts.initialStates();
  if (!init.empty()) {
    tsAut->set_init_state(static_cast<unsigned>(init.front()));
  }

  for (int s = 0; s < ts.numStates(); ++s) {
    for (int dst : ts.successors(s)) {
      bdd cond = labelToBdd(ts.label(dst), cVar, dVar);
      tsAut->new_edge(static_cast<unsigned>(s), static_cast<unsigned>(dst), cond);
    }
  }

  return tsAut;
}

int main()
{
  // Repeatedly visit c (not d), then leave c until reaching d: c->d->c->d...
  std::string input = "G F (c & !d & X(!c U d))";
  
  // Parse the LTL formula
  spot::parsed_formula pf = spot::parse_infix_psl(input);
  if (pf.format_errors(std::cerr))
    return 1;
  spot::formula f = pf.f;
  
  // Print the formula in various formats
  std::cout << "=== Input Formula ===" << std::endl;
  std::cout << "Infix: " << spot::str_psl(f) << std::endl;
  std::cout << "LaTeX: ";
  print_latex_psl(std::cout, f) << std::endl;
  
  // Create a translator object with default settings
  spot::translator trans;
  
  // Translate LTL formula to Büchi automaton
  spot::twa_graph_ptr aut = trans.run(f);
  

  // Output the automaton in various formats
  std::cout << "\n=== Büchi Automaton ===" << std::endl;
  
  // HOA format (Human-Oriented Format for Automata)
  std::cout << "\nHOA Format:" << std::endl;
  spot::print_hoa(std::cout, aut);
  
  // Statistics
  std::cout << "\n=== Automaton Statistics ===" << std::endl;
  std::cout << "Number of states: " << aut->num_states() << std::endl;
  std::cout << "Number of edges: " << aut->num_edges() << std::endl;
  std::cout << "Initial state: " << aut->get_init_state_number() << std::endl;
  
  // Print atomic propositions
  std::cout << "Atomic propositions: ";
  for (auto ap : aut->ap())
    std::cout << ap << " ";
  std::cout << std::endl;
  
  // Print accepting sets
  std::cout << "Number of accepting sets: " << aut->num_sets() << std::endl;

  // Build a sample TS automaton and use Spot's built-in synchronous product
  SpotTransitionSystem ts(2, 3);
  spot::twa_graph_ptr tsAut = buildTsAutomaton(ts, aut);
   std::cout << "\n=== TS Automaton ===" << std::endl;
  std::cout << "TS states: " << tsAut->num_states() << std::endl;
  std::cout << "TS edges: " << tsAut->num_edges() << std::endl;
  spot::twa_graph_ptr product = spot::product(tsAut, aut);

  std::cout << "\n=== Product Automaton via Spot::product ===" << std::endl;
  std::cout << "Product states: " << product->num_states() << std::endl;
  std::cout << "Product edges: " << product->num_edges() << std::endl;
  std::cout << "Product acceptance sets: " << product->num_sets() << std::endl;
  
  return 0;
}
