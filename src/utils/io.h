#pragma once

#include <iostream>

void printUsage()
{
	printf("\nUsage: <program> <file> <Options>.\n");
	printf("\t Option -h: Shows the usage manual and ends the program.\n");
	printf("\t Option -r: Performs joint regression of the 6890 model points. The result will contain the 24 joint points\n");

	printf("NOTE: If no <Options> are specified, the default result will contain the 6890 model points\n");
	printf("NOTE: Option -h will show the manual and end the programm regardless of other options specified.\n\n");
}