#include "sentence.hpp"
#include <ctype.h>
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;
using std::tolower;
using std::size_t;
using std::stringstream;
using std::getline;
using std::make_pair;

vector<string> split_string(string input_str,char delim);

Sentence::Sentence(string sentence)
: sentence_(sentence)
{
  // make the string to lower case before assigning
  for(size_t i=0;i<sentence.size();i++){
    sentence_[i] = tolower(sentence[i]);
  }
  cout<< sentence_<<endl;
  joiners_.push_back("and");
}

/*
  Splits the sentence into pairs of subjects and predicates and
  returns them as a vector of pairs

  Note: will only work for: "goto x","goto x and get y" type sentences
    
  This is very simplified right now. Modify this function in the 
  future to do complex sentence parsing.
 */
vector<pair<string,string> > Sentence::GetSubjectPredicates(){
  string joiner_and = joiners_[0] ; //"and"
  string substr_1; // see the note in the comment above.  only two 
  string substr_2; // substrings possible
  vector<pair<string,string> > subject_predicates;
  vector<string> splitted_sentence = split_string(sentence_,' ');
  if(splitted_sentence.size()==2){
    cout<< "2 element str"<<endl;
    pair<string,string> first_entry;
    first_entry = make_pair(splitted_sentence[0],splitted_sentence[1]);
    subject_predicates.push_back(first_entry);
  }
  else if(splitted_sentence.size()==5) {
    // Assuming that element at index 2 is the word "and" which is to be ignored
    cout<<"5 element str"<< endl;
    pair<string,string> first_entry, second_entry;
    first_entry = make_pair(splitted_sentence[0],splitted_sentence[1]);
    second_entry = make_pair(splitted_sentence[3],splitted_sentence[4]);
    subject_predicates.push_back(first_entry);
    subject_predicates.push_back(second_entry);
  }
  else{
    cout<<"Unknown sentence type. Return empty" <<endl;
  }
  
  return subject_predicates;

}


vector<string> split_string(string input_string, char delim) { 
  stringstream input_stream(input_string);
  vector<string> splitted_string;
  string substring;
  while(getline(input_stream, substring, delim)){
    splitted_string.push_back(substring);
  }
  return splitted_string;
}
