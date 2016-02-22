//serial_test.cpp


#include <iostream>
#include <cstdio>

int main(int argc, char const *argv[])
{
	FILE* arduino_in = fopen("/dev/ttyACM0", "w+");

	if(!arduino_in)
	{
		std::cout << "failed" << std::endl;
		return -1;
	}

	char input;

	while(true)
	{
		std::cout << ">> ";
		std::cin >> input;
		fprintf(arduino_in, "%c\n", input);
	}

	fclose(arduino_in);

	return 0;
}
