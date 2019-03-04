// constructing maps
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <sstream>
using namespace std;
bool fncomp (char lhs, char rhs) {return lhs<rhs;}

struct classcomp {
  bool operator() (const char& lhs, const char& rhs) const
  {return lhs<rhs;}
};

vector<int> parse_string_ids(std::string str)
{
  vector<int> result;
  stringstream ss(str);
  while( ss.good() )
  {
      string substr;
      getline( ss, substr, ',' );

      result.push_back(std::stoi(substr));
  }
  return result;
}


typedef std::map<std::string, std::vector<int>> markers_params;
using namespace std;
int main ()
{
  std::vector<int> vec {10,20};

  std::map<std::string, std::vector<int>> first;
  std::string a = "aa";
  first[a]=vec;
  first["marker"]=vec;
  for(auto const& imap: first)
    cout << imap.first << " "<<imap.second[0] << endl;
  
}


