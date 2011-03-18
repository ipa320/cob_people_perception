#include <iostream>

#ifdef __LINUX__
	#include "cob_people_detection/CuiPeopleDetector.h"
#else
	#include "cob_vision/cob_people_detection/common/include/cob_people_detection/CuiPeopleDetector.h"
#endif


int main()
{
	/// Calibration GUI
	CuiPeopleDetector cui;
	cui.Init();
	while(cui.ConsoleGUI() & ipa_Utils::RET_OK)
	{
		/// Void
	}

	/// Terminate
	char c;
	while(std::cin.get(c) && c != '\n');
	std::cout << "INFO - Main:" << std::endl;
	std::cout << "\t ... Press ENTER to quit ... \n";
	getchar();
	return 0;
}