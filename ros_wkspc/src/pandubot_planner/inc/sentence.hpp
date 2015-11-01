#include<string>
#include<utility>
#include<vector>

using std::string;
using std::vector;
using std::pair;

// Class to parse subjects and predicate from a given sentence
// Eg: "Goto vending machine and get pepsi" parsing gives a vector of std::pairs:
//      [<goto,vending_machine>
//       <get,pepsi>]

class Sentence {
  private:
    string sentence_;  
    vector<string> joiners_;
  public:
    Sentence(string sentence);
    vector<pair<string,string> > GetSubjectPredicates();  
};
