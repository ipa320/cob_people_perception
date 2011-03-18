/// @file CuiPeopleDetector.h
/// Interface for GUI-Options
/// @author Daniel Seitz, modified Richard Bormann
/// @date Sep, 2009.

#ifndef __CUIPEOPLEDETECTOR_H__
#define __CUIPEOPLEDETECTOR_H__


#include <iostream>
#ifdef __LINUX__
	#include "cob_people_detection/PeopleDetectorControlFlow.h"
#else
	#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetectorControlFlow.h"
#endif


class CuiPeopleDetector
{
public:
	/// Constructor.
	CuiPeopleDetector(void); ///< Constructor
	~CuiPeopleDetector(void); ///< Destructor

	/// Initialize the Pointer
	unsigned long Init();

	/// Opens a console menu to run the functions of this class
	/// @return >=0 if a menu item was selected, -1 otherwise (used for termination)
	unsigned long ConsoleGUI();

	/// Opens a console menu to run the functions of this class
	/// @return >=0 if a menu item was selected, -1 otherwise (used for termination)
	unsigned long Train();

	/// Opens a console menu to run the functions of this class
	/// @return >=0 if a menu item was selected, -1 otherwise (used for termination)
	unsigned long Recognize();

	/// Opens a console menu to run the functions of this class
	/// @return >=0 if a menu item was selected, -1 otherwise (used for termination)
	unsigned long ShowEigenfaces();

	/// Opens a console menu to run the functions of this class
	/// @return >=0 if a menu item was selected, -1 otherwise (used for termination)
	unsigned long ShowAvgImage();

private:
	ipa_PeopleDetector::PeopleDetectorControlFlow* m_DetectorControlFlow; ///< Describe
};

#endif // __CUIPEOPLEDETECTOR_H__