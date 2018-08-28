#include <time_lib/timer.h>

namespace as64_
{

namespace tm_
{

// *********************************
// ********  Timer class  **********
// *********************************
void TIMER::start()
{
	resume_pressed = pause_pressed = false;
	t_start = t_pause = t_resume = std::chrono::high_resolution_clock::now();
}

void TIMER::pause()
{
	t_pause = std::chrono::high_resolution_clock::now();
	pause_pressed = true;
}

void TIMER::resume()
{
	if (!pause_pressed) return;

	t_resume = std::chrono::high_resolution_clock::now();
	resume_pressed = true;
}

double TIMER::stop()
{
	if (pause_pressed && !resume_pressed)
	{
		t_stop = t_resume = t_pause;
	}
	else
	{
		t_stop = std::chrono::high_resolution_clock::now();
	}

  return (std::chrono::duration_cast<std::chrono::duration<double>>( t_stop - t_start - (t_resume - t_pause) )).count();
}

// ********************************
// ********************************

} // namespace tm_

} // namespace as64_
