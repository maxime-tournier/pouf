#ifndef THREAD_WORKER_H
#define THREAD_WORKER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace thread {

  // simple worker thread with its own work queue
  class worker {
	std::condition_variable cv;

	typedef std::mutex mutex_type;
	mutex_type mutex;

	typedef std::unique_lock<mutex_type> lock_type;

	void loop();
  public:
	
	worker();
	~worker();
	
	typedef std::function< void () > task_type;
	void push(const task_type& );
	
  private:
	std::queue<task_type> queue;

	// make sure this is the last initialized !
	std::thread thread;
	
  };



}


#endif
