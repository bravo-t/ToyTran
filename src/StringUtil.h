#ifndef _NA_STRUTIL_H_
#define _NA_STRUTIL_H_

#include <vector>
#include <string>
#include <cstring>

namespace NA {

inline void
splitWithAny(const std::string& src, const char *delim,
             std::vector<std::string>& strs)
{
  char *data = strdup(src.data());
  strs.clear();
  char *saveptr(nullptr);
  if (char* res1 = strtok_r(data, delim, &saveptr)) {
    strs.push_back(res1);
    while(char *res2 = strtok_r(nullptr, delim, &saveptr)) {
      std::string tempStr(res2);
      size_t eraseLength = 0;
      for (size_t i=0; i<tempStr.size(); ++i) {
        if (isspace(tempStr[i]) || iscntrl(tempStr[i])) {
          ++eraseLength;
        } else {
          break;
        }
      }
      if (eraseLength > 0) {
        tempStr.erase(0, eraseLength);
      }
      if (tempStr.empty()) {
        continue;
      }
      char c = tempStr.back();
      while (isspace(c) || iscntrl(c)) {
        tempStr.pop_back();
        if (tempStr.empty()) {
          break;
        }
        c = tempStr.back();
      }
      if (tempStr.empty() == false) {
        strs.push_back(std::string(tempStr));
      } 
    }
  }
  ::free(data);
}

inline std::string 
trim(const std::string& str,
     const std::string& whitespace = " \t\n\r")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

inline bool 
ichar_equals(char a, char b)
{
  return std::tolower(static_cast<unsigned char>(a)) ==
         std::tolower(static_cast<unsigned char>(b));
}

inline bool 
iequals(const std::string& a, const std::string& b)
{
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), ichar_equals);
}

inline std::string
fileNameWithoutSuffix(const char* fname)
{
  size_t nameLength = strlen(fname);
  size_t nameEnd = 0;
  for (size_t i=0; i<nameLength; ++i) {
    nameEnd = i;
    if (fname[i] == '.') {
      break;
    }
  }
  return std::string(fname, nameEnd);
}


}

#endif