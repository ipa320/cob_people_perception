/*
 * timer.h
 *
 *  Created on: Mar 19, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_BENCHMARKING_TIMER_H_
#define PEOPLE_LEG_DETECTOR_INCLUDE_BENCHMARKING_TIMER_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace benchmarking{

/**
 *  \brief A simple Timer Class
 *
 *  The timer class can be used for benchmarking purposes.
 */
class Timer{
  public:
    Timer():running(false){}

    /** Starts the timer */
    void start(){
        running = true;
        time_start = boost::posix_time::microsec_clock::local_time();
    }

    /** Stops the timer */
    void stop(){
        time_end = boost::posix_time::microsec_clock::local_time();
        running = false;
    }
    boost::posix_time::time_duration getDuration(){
        boost::posix_time::time_duration duration;
        if(running){
            duration = boost::posix_time::microsec_clock::local_time() - time_start;
        }
        else
        {
            duration = time_end - time_start;
        }
        return duration;
    }

    /** Stop the timer and get the duration */
    boost::posix_time::time_duration stopAndGetTime(){
        stop();
        return getDuration();
    }

    /** Stop the timer and get the Elapsed Time in MilliSeconds */
    double stopAndGetTimeMs(){
        return (double) (stopAndGetTime().total_microseconds()/1000.0);
    }

    /** Get the Elapsed Time in Milliseconds */
    double getElapsedTimeMs(){
        return (double) (getDuration().total_microseconds()/1000.0);
    }

  private:
    boost::posix_time::ptime time_start; /**< Start Time */
    boost::posix_time::ptime time_end; /**< End Time */

    bool running; /**< Running Flag */
};
}

#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_BENCHMARKING_TIMER_H_ */
