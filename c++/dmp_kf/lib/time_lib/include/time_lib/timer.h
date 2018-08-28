#ifndef TIME_LIB_TIMER_64_H
#define TIME_LIB_TIMER_64_H

#include <ctime>
#include <chrono>

namespace as64_
{

namespace tm_
{

/**
 * Implements a timer class.
 */
class TIMER
{
public:
	/** \brief Starts the timer
	 *  Can be used also to reset the timer.
	 */
	void start();

	/** \brief Pauses the timer
	 */
	void pause();

	/** \brief Unpauses the timer
	 *
	 *  \note This function does nothing if pause is not called first.
	 */
	void resume();

	/** \brief Stops the timer
	 * @return The time elapsed from starting till stopping the timer, excluding the pausing time
	 *
	 *  \note If pause is called and resume is not called before stop, then stop returns the
	 *        elapsed time between start and pause.
	 */
	double stop();
private:
	std::chrono::high_resolution_clock::time_point t_start; ///< stores the time when \a start was called
	std::chrono::high_resolution_clock::time_point t_pause; ///< stores the time when \a pause was called
	std::chrono::high_resolution_clock::time_point t_resume; ///< stores the time when \a resume was called
	std::chrono::high_resolution_clock::time_point t_stop; ///< stores the time when \a stop was called

	bool pause_pressed; ///< flag indicating whether \a pause was called
	bool resume_pressed; ///< flag indicating whether \a resume was called
};

} // namespace tm_

} // namespace as64_


#endif // TIME_LIB_TIMER_64_H
