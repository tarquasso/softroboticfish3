// util.hpp

#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <stdio.h>

std::string pkg_path(int p)
// p is number of parent directory levels to climb
{
	std::string s(__FILE__);
	
	int i = s.length() - 1; // index to last char in string
	while (i > 0)
	{
		if (s[i] == '/' || s[i] == '\\')
		{
			// Decrement parent dir counter
			if (p > 0)
			{
				--p;
			}
			else
			{
				// erase rest of the string
				s.erase(i);
			}
		}
		++i;
	}

	if (p > 0)	
		printf("Unable to climb specified number of levels in file path.");

	return s;
}



#endif