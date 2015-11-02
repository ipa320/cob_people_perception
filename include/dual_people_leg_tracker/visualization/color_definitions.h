#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_COLOR_DEFINITIONS_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_COLOR_DEFINITIONS_H_

#define USE_BASH_COLORS

    #ifdef USE_BASH_COLORS

        #define RESET   "\033[0m"
        #define BLACK   "\033[30m"      /* Black */
        #define RED     "\033[31m"      /* Red */
        #define GREEN   "\033[32m"      /* Green */
        #define YELLOW  "\033[33m"      /* Yellow */
        #define BLUE    "\033[34m"      /* Blue */
        #define MAGENTA "\033[35m"      /* Magenta */
        #define CYAN    "\033[36m"      /* Cyan */
        #define WHITE   "\033[37m"      /* White */
        #define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
        #define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
        #define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
        #define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
        #define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
        #define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
        #define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
        #define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
        #define CLEAR       "\033[2J"              // Clear

    #else

        #define RESET   ""
        #define BLACK   ""      /* Black */
        #define RED     ""      /* Red */
        #define GREEN   ""      /* Green */
        #define YELLOW  ""      /* Yellow */
        #define BLUE    ""      /* Blue */
        #define MAGENTA ""      /* Magenta */
        #define CYAN    ""      /* Cyan */
        #define WHITE   ""      /* White */
        #define BOLDBLACK   ""      /* Bold Black */
        #define BOLDRED     ""      /* Bold Red */
        #define BOLDGREEN   ""      /* Bold Green */
        #define BOLDYELLOW  ""      /* Bold Yellow */
        #define BOLDBLUE    ""      /* Bold Blue */
        #define BOLDMAGENTA ""      /* Bold Magenta */
        #define BOLDCYAN    ""      /* Bold Cyan */
        #define BOLDWHITE   ""      /* Bold White */
        #define CLEAR ""

    #endif
#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_COLOR_DEFINITIONS_H_ */
