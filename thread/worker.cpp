#include "worker.h"

#include <iostream>

namespace thread {


  struct exit { };
  

  worker::worker()
	: thread( [&] { this->loop(); }) {
	lock_type lock(mutex);
	cv.wait(lock);
  }


  void worker::loop() {
	try {
	  cv.notify_one();

	  task_type task;
	  while( true ) {
		{
		  lock_type lock(mutex);
		  cv.wait(lock, [&] { return !queue.empty();});
		  task = queue.front();
		  queue.pop();
		}
		
		task();
	  }
	  
	}
	catch( exit e) { }
  }

  void worker::push(const task_type& task) {
	{
	  lock_type lock(mutex);
	  queue.push( task );
	}
	cv.notify_one();
  }


  worker::~worker() {

	push([&] { throw exit(); });
	thread.join();
	
  }

}
